/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Spectrum and waterfall on the M5Stack Cardputer-Adv's 240x135 display, receive only.
 *
 * Each row is a Welch average of Hann-windowed 1024-point FFTs over one 4096-sample burst
 * (16, 40 or 80 MS/s). The 240 pixels show a view (centre, zoom) of that span: each pixel takes
 * the strongest bin under it. Scrolling past the captured span retunes the radio. No PSRAM:
 * the history is one byte per pixel (palette index), converted to RGB565 a few rows at a time
 * on the way to the display, and remapped when the view pans or zooms.
 *
 * Keys (Cardputer arrows are ; . , /):
 *   , /   scroll left / right        ; .   span 80/40/16/8/4/2 MHz narrower / wider
 *   h     back to 2440 MHz, 16 MHz span
 *   w     Wi-Fi channel overlay on/off
 *   p     bursts per row (peak hold): 8 4 2 1 32 16, faster to the right, then slower
 *   - =   darker / brighter (floor +2 / -2 dB)    [ ]   colour range -5 / +5 dB
 *   a     automatic floor on/off     n m   gain -1 / +1 (fixed)   g  gain auto
 *   x     the receiver's own narrow lines cleared (width_filter()) on/off
 *   c     bursts that clipped taken again (clip_drop) on/off
 *   r     band labels and number keys: Europe/UK or US
 *   v     auto-rotate on/off: held upright, the portrait layout, scrolled by the panel
 *
 * The USB console takes the same keys, and lines starting with $ as commands for a host
 * (tools/wf_serial.py): full 1024-bin spectra and raw I/Q bursts come back as binary frames
 * between the text lines, see struct frame_hdr. $help lists the commands.
 */

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/mipi_dbi.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>

#include <soc/rtc_cntl_reg.h>

#include <esp_sdr/esp_sdr.h>
#include <esp_sdr/esp_sdr_dsp.h>

#include "flat_tables.h"

LOG_MODULE_REGISTER(waterfall, LOG_LEVEL_INF);

/*
 * Firmware that used USB OTG (such as the Cardputer's factory image) leaves the PHY switched
 * to OTG across resets; give it back to the USB-Serial-JTAG (console, esptool) at boot.
 */
static int usj_phy_select(void)
{
	REG_SET_BIT(RTC_CNTL_USB_CONF_REG, RTC_CNTL_SW_HW_USB_PHY_SEL);
	REG_CLR_BIT(RTC_CNTL_USB_CONF_REG, RTC_CNTL_SW_USB_PHY_SEL);
	return 0;
}
SYS_INIT(usj_phy_select, PRE_KERNEL_1, 0);

#define W          240
#define H          135
#define HDR_H      10
#define SCALE_Y    HDR_H
#define SCALE_H    9 /* MHz numbers over ticks */
#define PSD_Y      (SCALE_Y + SCALE_H)
#define PSD_H      30
#define WF_Y       (PSD_Y + PSD_H)
#define WF_H       (H - WF_Y)
#define CHUNK_ROWS HDR_H /* the header is drawn in one chunk */

/* History rows: the portrait layout's waterfall (more than the landscape one's WF_H). */
#define HIST_ROWS 175
/* The ST7789's frame memory has 320 lines, scrolled in hardware; the 240 shown start at 40. */
#define PANEL_LINES 320
#define PANEL_X0    40

#define NFFT      1024U
#define SAMPLES   4096U

/*
 * Landscape redraws after every row (or zoom rows): at a frame rate the waterfall stepped by
 * one row or two at a time, unevenly, which shimmered; a whole redraw is about 6.5 ms at
 * 80 MHz SPI. The parts portrait redraws (settings, trace) follow at FRAME_MS. Input redraws
 * at once.
 */
#define FRAME_MS 66U

/*
 * One ladder of screen spans for the zoom keys: each takes the narrowest sample rate that
 * covers it (the finest bins), and the narrow ones a part of the 16 MS/s capture.
 */
/*
 * The spans: the capture rate, the screen's span, and the zoom, the captured rows per redraw
 * (time resolution grows with frequency resolution below 16 MHz). 16 MHz is captured at
 * 40 MS/s: at 16 MS/s the analog filters pass about +-10 MHz, wider than the capture, and
 * whatever lies 8 to 10 MHz out folded back in at the other edge (Wi-Fi at 2402 MHz showed
 * 21 dB up at 2386, tuned to 2393). Its 39 kHz bins are still finer than the pixels. Likewise
 * 40 MHz at 80 MS/s: at 40 MS/s the filters' slow roll-off past +-18 MHz let a strong signal
 * just beyond one edge fold in at the other (a 2675 MHz mast at 2635, tuned to 2655).
 */
static const struct {
	enum esp_sdr_rate rate;
	int span_mhz;
	int zoom;
} spans[] = {
	{ESP_SDR_RATE_80MSPS, 1000, 1}, /* swept, down to 40: sweeping() */
	{ESP_SDR_RATE_80MSPS, 640, 1},
	{ESP_SDR_RATE_80MSPS, 320, 1},
	{ESP_SDR_RATE_80MSPS, 160, 1},
	{ESP_SDR_RATE_80MSPS, 80, 1},
	{ESP_SDR_RATE_80MSPS, 40, 1},
	{ESP_SDR_RATE_40MSPS, 16, 1},
	{ESP_SDR_RATE_40MSPS, 8, 2},
	{ESP_SDR_RATE_16MSPS, 4, 4},
	{ESP_SDR_RATE_16MSPS, 2, 8},
};
#define SPAN_HOME 6 /* 16 MHz */
static bool sweeping(void);
/* Wi-Fi is bursty: with peak hold each row is the per-bin maximum over this many bursts. */
#define PEAK_BURSTS 8 /* the default; p steps through 8, 4, 2, 1, 32, 16 */
#define PEAK_MAX    32
#define FULLSCALE 512.0f
#define FLOOR_DB  (-150.0f)

/* The 40 MHz PHY mode's analog filters pass about +-18 MHz (doc/hardware-quirks.md). */
#define PASS_KHZ 18000

#define FREQ_MIN_MHZ     1800
#define FREQ_MAX_MHZ     2800
#define FREQ_DEFAULT_MHZ 2440

/* The panel takes RGB565 most significant byte first. */
#define RGB565(r, g, b)                                                                            \
	sys_cpu_to_be16((uint16_t)((((r) & 0xf8) << 8) | (((g) & 0xfc) << 3) | ((b) >> 3)))

static const struct device *const display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
static const struct device *const console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

static float window[NFFT], psd[NFFT], peak[NFFT];
static float tw_cos[NFFT / 2], tw_sin[NFFT / 2];

/*
 * The FFT: esp-dsp's ESP32-S3 kernel (west project esp-dsp), interleaved complex in natural
 * order, bins out in bit-reversed order. Checked against the C one at boot (fft_check()),
 * which takes over if they disagree.
 */
extern int dsps_fft2r_fc32_aes3_(float *data, int n, float *w);
static float cbuf[2 * NFFT] __aligned(16);
static float dsp_w[NFFT] __aligned(16); /* cos, sin of tau i / NFFT, i bit reversed */
static uint16_t rev[NFFT];
static bool use_dsp;
static void flat_set(bool on);
static bool overloaded(void);
static uint16_t ovl_color(void);
static bool clip_drop = true; /* bursts that clipped taken again (c, $clip) */
static void shot_rect(unsigned int x, unsigned int y, unsigned int w, unsigned int h,
		      const uint16_t *buf);
static float window_gain;

static float row_db[W], trace_db[W];
/* The per-pixel maximum since the last console dump. */
static float dump_db[W];
static bool dump_fresh = true;
static uint8_t hist[HIST_ROWS][W];
static unsigned int head;
static uint16_t palette[256];
static uint16_t fb[W * CHUNK_ROWS];

static struct {
	int lo_mhz;     /* the radio's LO, MHz */
	int lo_ofs_khz; /* and above that, 0 to 999 kHz (offset tuning, zoomed in) */
	int view_khz; /* the centre of the screen */
	int span_i;   /* the step in spans[] */
	int zoom;     /* the screen spans the sample rate / zoom */
	bool trace_reset;
	bool channels; /* the 2.4 GHz Wi-Fi channel overlay */
	int peak; /* bursts per row, their per-bin maximum (1: one burst's average) */
	bool width_filter; /* narrow lines cleared, the receiver's own among them (x) */
	bool shot_req, shot; /* $shot: the next redraw also goes to the host (shot_rect()) */
	bool flat; /* the receiver's shape divided out (flat_apply()) */
	enum esp_sdr_rate rate;
	int gain; /* ESP_SDR_RX_GAIN_AUTO or a table index */
	bool help;       /* the key help covers the screen */
	bool help_drawn;
	bool auto_floor;
	float floor_db, range_db;
	bool tune_warn;
	bool dirty_radio;
	bool text;     /* the periodic spec and status lines on the console */
	bool screen;   /* drawing; off leaves more time for the host's captures */
	int bins_left; /* rows whose full spectrum goes to the console, -1 for all */
	int iq_left;   /* bursts whose raw I/Q goes to the console */
	bool bins_post; /* send the spectra after the flattening and the filters, as drawn */
	int orient;       /* enum orient: held upright, the portrait layout */
	bool auto_rotate; /* the orientation follows the accelerometer */
	bool side_dirty;  /* the portrait layout needs drawing in full */
	bool region_us;   /* the band labels: Europe/UK (false) or the US */
	int bw_mhz;       /* the analog low-pass filter at 16 MS/s, two-sided MHz */
} st = {
	.lo_mhz = FREQ_DEFAULT_MHZ,
	.view_khz = FREQ_DEFAULT_MHZ * 1000,
	.span_i = SPAN_HOME,
	.zoom = 1,
	.peak = PEAK_BURSTS,
	.rate = ESP_SDR_RATE_40MSPS, /* spans[SPAN_HOME] */
	/* Fixed mid gain: the AGC re-levels each burst and flattens bursty Wi-Fi into the noise. */
	.gain = 42,
	.auto_floor = true,
	.floor_db = -100.0f,
	.range_db = 40.0f,
	.dirty_radio = true,
	/*
	 * Off: the screen hitches each time one goes out (a pause every few seconds, cause not
	 * found; the binary frames, larger and more frequent, do not do it).
	 */
	.text = false,
	.screen = true,
	.auto_rotate = true,
	.bw_mhz = 16,
	.width_filter = true,
	.flat = true,
};

/* 5x7 font, ASCII 0x20 to 0x5d, columns LSB on top. */
static const uint8_t font5x7[][5] = {
	{0x00, 0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x5f, 0x00, 0x00},
	{0x00, 0x07, 0x00, 0x07, 0x00}, {0x14, 0x7f, 0x14, 0x7f, 0x14},
	{0x24, 0x2a, 0x7f, 0x2a, 0x12}, {0x23, 0x13, 0x08, 0x64, 0x62},
	{0x36, 0x49, 0x55, 0x22, 0x50}, {0x00, 0x05, 0x03, 0x00, 0x00},
	{0x00, 0x1c, 0x22, 0x41, 0x00}, {0x00, 0x41, 0x22, 0x1c, 0x00},
	{0x08, 0x2a, 0x1c, 0x2a, 0x08}, {0x08, 0x08, 0x3e, 0x08, 0x08},
	{0x00, 0x50, 0x30, 0x00, 0x00}, {0x08, 0x08, 0x08, 0x08, 0x08},
	{0x00, 0x60, 0x60, 0x00, 0x00}, {0x20, 0x10, 0x08, 0x04, 0x02},
	{0x3e, 0x51, 0x49, 0x45, 0x3e}, {0x00, 0x42, 0x7f, 0x40, 0x00},
	{0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4b, 0x31},
	{0x18, 0x14, 0x12, 0x7f, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39},
	{0x3c, 0x4a, 0x49, 0x49, 0x30}, {0x01, 0x71, 0x09, 0x05, 0x03},
	{0x36, 0x49, 0x49, 0x49, 0x36}, {0x06, 0x49, 0x49, 0x29, 0x1e},
	{0x00, 0x36, 0x36, 0x00, 0x00}, {0x00, 0x56, 0x36, 0x00, 0x00},
	{0x00, 0x08, 0x14, 0x22, 0x41}, {0x14, 0x14, 0x14, 0x14, 0x14},
	{0x41, 0x22, 0x14, 0x08, 0x00}, {0x02, 0x01, 0x51, 0x09, 0x06},
	{0x32, 0x49, 0x79, 0x41, 0x3e}, {0x7e, 0x11, 0x11, 0x11, 0x7e},
	{0x7f, 0x49, 0x49, 0x49, 0x36}, {0x3e, 0x41, 0x41, 0x41, 0x22},
	{0x7f, 0x41, 0x41, 0x22, 0x1c}, {0x7f, 0x49, 0x49, 0x49, 0x41},
	{0x7f, 0x09, 0x09, 0x01, 0x01}, {0x3e, 0x41, 0x41, 0x51, 0x32},
	{0x7f, 0x08, 0x08, 0x08, 0x7f}, {0x00, 0x41, 0x7f, 0x41, 0x00},
	{0x20, 0x40, 0x41, 0x3f, 0x01}, {0x7f, 0x08, 0x14, 0x22, 0x41},
	{0x7f, 0x40, 0x40, 0x40, 0x40}, {0x7f, 0x02, 0x04, 0x02, 0x7f},
	{0x7f, 0x04, 0x08, 0x10, 0x7f}, {0x3e, 0x41, 0x41, 0x41, 0x3e},
	{0x7f, 0x09, 0x09, 0x09, 0x06}, {0x3e, 0x41, 0x51, 0x21, 0x5e},
	{0x7f, 0x09, 0x19, 0x29, 0x46}, {0x46, 0x49, 0x49, 0x49, 0x31},
	{0x01, 0x01, 0x7f, 0x01, 0x01}, {0x3f, 0x40, 0x40, 0x40, 0x3f},
	{0x1f, 0x20, 0x40, 0x20, 0x1f}, {0x7f, 0x20, 0x18, 0x20, 0x7f},
	{0x63, 0x14, 0x08, 0x14, 0x63}, {0x03, 0x04, 0x78, 0x04, 0x03},
	{0x61, 0x51, 0x49, 0x45, 0x43}, {0x00, 0x7f, 0x41, 0x41, 0x00},
	{0x02, 0x04, 0x08, 0x10, 0x20}, {0x00, 0x41, 0x41, 0x7f, 0x00},
};

/* ---- display ---- */

static void blit(unsigned int y, unsigned int rows)
{
	struct display_buffer_descriptor desc = {
		.buf_size = W * rows * sizeof(uint16_t),
		.width = W,
		.height = rows,
		.pitch = W,
	};

	display_write(display, 0, y, &desc, fb);
	if (st.shot) {
		shot_rect(0, y, W, rows, fb);
	}
}

static void build_palette(void)
{
	/* Black, blue, cyan, yellow, red, white. */
	static const uint8_t stops[][3] = {
		{0, 0, 0}, {0, 0, 160}, {0, 200, 220}, {240, 240, 0}, {255, 32, 0}, {255, 255, 255},
	};
	const unsigned int segs = ARRAY_SIZE(stops) - 1;

	for (unsigned int k = 0; k < 256; k++) {
		float x = (float)k / 255.0f * (float)segs;
		unsigned int s = MIN((unsigned int)x, segs - 1);
		float t = x - (float)s;
		uint8_t c[3];

		for (int j = 0; j < 3; j++) {
			c[j] = (uint8_t)((float)stops[s][j] +
					 t * ((float)stops[s + 1][j] - (float)stops[s][j]));
		}
		palette[k] = RGB565(c[0], c[1], c[2]);
	}
}

static void draw_text_w(uint16_t *buf, unsigned int stride, unsigned int x, unsigned int y,
			const char *s, uint16_t color)
{
	for (; *s != '\0' && x + 5 <= stride; s++, x += 6) {
		int ch = toupper((unsigned char)*s);

		if (ch < 0x20 || ch > 0x5d) {
			ch = '?';
		}
		for (unsigned int col = 0; col < 5; col++) {
			uint8_t bits = font5x7[ch - 0x20][col];

			for (unsigned int row = 0; row < 7; row++) {
				if (bits & BIT(row)) {
					buf[(y + row) * stride + x + col] = color;
				}
			}
		}
	}
}

static void draw_text(uint16_t *buf, unsigned int x, unsigned int y, const char *s,
		      uint16_t color)
{
	draw_text_w(buf, W, x, y, s, color);
}

static int fs_khz(void)
{
	return (int)(esp_sdr_rx_rate_hz(st.rate) / 1000U);
}

static int span_khz(void)
{
	return spans[st.span_i].span_mhz * 1000;
}

static uint8_t *hist_row(unsigned int age)
{
	return hist[(head + age) % HIST_ROWS];
}

/*
 * Re-lay the history for a new view so old rows stay where their frequencies are, and
 * stretch it in time with the zoom (anchored at the newest row), as a zoomed-in row covers
 * 1/zoom of the time. In place: zooming in reads rows at or above the one written, so go down
 * from the bottom; zooming out reads rows at or below, so go up from the top.
 */
static void hist_remap(int old_view_khz, int old_span_khz, int old_zoom)
{
	static uint8_t tmp[W];
	int new_span = span_khz();
	int d = (st.view_khz - new_span / 2) - (old_view_khz - old_span_khz / 2);

	if (st.zoom > old_zoom) {
		for (int r = HIST_ROWS - 1; r > 0; r--) {
			memcpy(hist_row(r), hist_row(r * old_zoom / st.zoom), W);
		}
	} else if (st.zoom < old_zoom) {
		for (unsigned int r = 1; r < HIST_ROWS; r++) {
			unsigned int src = r * old_zoom / st.zoom;

			if (src < HIST_ROWS) {
				memcpy(hist_row(r), hist_row(src), W);
			} else {
				memset(hist_row(r), 0, W);
			}
		}
	}

	for (unsigned int r = 0; r < HIST_ROWS; r++) {
		for (unsigned int c = 0; c < W; c++) {
			float x = ((float)d + ((float)c + 0.5f) * (float)new_span / (float)W) *
				  (float)W / (float)old_span_khz;

			tmp[c] = (x >= 0.0f && x < (float)W) ? hist[r][(unsigned int)x] : 0U;
		}
		memcpy(hist[r], tmp, W);
	}
	st.trace_reset = true;
	/*
	 * The portrait layout redraws in full only now, when the history moved: any key used
	 * to, and the whole picture jumped (the scroll reset and 175 columns rewritten).
	 */
	st.side_dirty = true;
}

/* Retune when the view leaves the span the radio captures. */
static int lo_khz(void)
{
	return st.lo_mhz * 1000 + st.lo_ofs_khz;
}

static void set_lo_khz(int khz)
{
	khz = CLAMP(khz, FREQ_MIN_MHZ * 1000, FREQ_MAX_MHZ * 1000);
	if (khz != lo_khz()) {
		st.lo_mhz = khz / 1000;
		st.lo_ofs_khz = khz % 1000;
		st.dirty_radio = true;
	}
}

/*
 * The receiver's own spur that moves with the LO: at LO + (LO - 2400 MHz) / 3, measured
 * within 5 kHz at LOs from 2380 to 2421 MHz (2386.67 at 2390, 2416 at 2412, 2428 at 2421).
 */
static int lo_spur_khz(int lo)
{
	return lo + (lo - 2400000) / 3;
}

/*
 * Retune when the view leaves the span the radio captures. Zoomed in (8 MHz of the 40 MS/s
 * capture, 4 and 2 MHz of the 16 MS/s one), offset tuning: the LO sits 1 MHz outside the
 * view, so its line and the skirt around it (+3 dB out to about 110 kHz, the same at every
 * LO) stay off the screen, and the view stays clear of the capture's edges.
 * Below the view, unless that puts lo_spur_khz() in it: then above.
 */
static void follow_view(void)
{
	int half = fs_khz() / 2, span = span_khz(), lo = lo_khz();

	if (st.zoom > 1) {
		int a = st.view_khz - span / 2, b = st.view_khz + span / 2;
		int gap = 1000, below = a - gap;
		int spur = lo_spur_khz(lo);

		/*
		 * Keep the tuning while it still serves (fewer retunes): the LO 200 kHz or more off the view, the view within the capture
		 * (or as far over its edge as the 8 MHz placement goes), its spur off the view.
		 */
		/*
		 * The clean part of the capture: at 40 MS/s the flat passband (2 MHz inside the
		 * filters' +-18), at 16 MS/s 2 MHz inside the edges, where nothing folds in.
		 */
		int clean = (st.rate == ESP_SDR_RATE_16MSPS ? half : PASS_KHZ) - 2000;

		if ((lo < a - 200 || lo > b + 200) && a >= lo - clean && b <= lo + clean &&
		    (spur < a - 150 || spur > b + 150)) {
			return;
		}
		spur = lo_spur_khz(below);
		set_lo_khz(spur > a - 150 && spur < b + 150 ? b + gap : below);
	} else {
		/*
		 * Not zoomed in: the LO at the view centre, so the analog passband (and its
		 * dimmed edges at 80 MHz) stays centred while scrolling; 40 and 16 MHz capture more
		 * than they show, and a LO left where it was would drift across the screen.
		 */
		ARG_UNUSED(half);
		ARG_UNUSED(lo);
		set_lo_khz((st.view_khz + 500) / 1000 * 1000);
	}
}

static void draw_header(void)
{
	char line[48];
	char gain[8];
	uint16_t bg = RGB565(16, 16, 40), fg = RGB565(255, 255, 255);

	if (st.gain == ESP_SDR_RX_GAIN_AUTO) {
		snprintf(gain, sizeof(gain), "AGC");
	} else {
		snprintf(gain, sizeof(gain), "%d", st.gain);
	}
	snprintf(line, sizeof(line), "%4d.%03d%s %2dM P%-2d%c%c%c G:%s %c%d/%d %s", st.view_khz / 1000,
		 st.view_khz % 1000, st.tune_warn ? "!" : " ", span_khz() / 1000, st.peak,
		 st.width_filter ? 'X' : ' ', st.flat ? 'F' : ' ', clip_drop ? 'C' : ' ', gain, st.auto_floor ? 'A' : 'L', (int)st.floor_db,
		 (int)st.range_db, overloaded() ? "OVL" : st.region_us ? "US" : "EU");
	if (!st.screen) {
		snprintf(line, sizeof(line), "SCREEN PAUSED BY HOST");
	}
	for (unsigned int k = 0; k < W * HDR_H; k++) {
		fb[k] = bg;
	}
	draw_text(fb, 2, 1, line, fg);
	if (st.screen && overloaded() && strstr(line, "OVL") != NULL) {
		draw_text(fb, 2 + 6 * (unsigned int)(strstr(line, "OVL") - line), 1, "OVL", ovl_color());
	}
	blit(0, HDR_H);
}

/*
 * The frequency scale: ticks at least 24 pixels apart (taller on whole MHz) and, over the
 * ticks of a coarser step that leaves room for the text, their frequency in MHz.
 */
static void draw_scale(void)
{
	static const int steps[] = {10,    20,    50,    100,   200,    500,    1000,
				    2000,  5000,  10000, 20000, 50000, 100000, 200000};
	int span = span_khz(), left = st.view_khz - span / 2;
	int step = steps[ARRAY_SIZE(steps) - 1], label = step;
	uint16_t fg = RGB565(200, 200, 200);
	int next = 0;

	for (size_t i = 0; i < ARRAY_SIZE(steps); i++) {
		if (steps[i] * W >= 24 * span) {
			step = steps[i];
			break;
		}
	}
	/* Text is 6 pixels a character; "2440.25" at worst, plus a gap. */
	for (size_t i = 0; i < ARRAY_SIZE(steps); i++) {
		int chars = steps[i] >= 1000 ? 4 : steps[i] >= 100 ? 6 : 7;

		if (steps[i] >= step && steps[i] % step == 0 &&
		    (int64_t)steps[i] * W >= (int64_t)(6 * chars + 6) * span) {
			label = steps[i];
			break;
		}
	}

	for (unsigned int k = 0; k < W * SCALE_H; k++) {
		fb[k] = RGB565(0, 0, 0);
	}
	for (int f = (left + step - 1) / step * step; f < left + span; f += step) {
		unsigned int x = (unsigned int)((int64_t)(f - left) * W / span);

		for (unsigned int y = (f % 1000 == 0) ? SCALE_H - 2U : SCALE_H - 1U;
		     y < SCALE_H && x < W; y++) {
			fb[y * W + x] = RGB565(160, 160, 160);
		}
		if (f % label == 0) {
			char num[12];
			int len, lx;

			if (label >= 1000) {
				len = snprintf(num, sizeof(num), "%d", f / 1000);
			} else if (label >= 100) {
				len = snprintf(num, sizeof(num), "%d.%d", f / 1000, f % 1000 / 100);
			} else {
				len = snprintf(num, sizeof(num), "%d.%02d", f / 1000, f % 1000 / 10);
			}
			lx = (int)x - (6 * len - 1) / 2;
			if (lx >= next && lx >= 0 && lx + 6 * len <= (int)W) {
				draw_text(fb, (unsigned int)lx, 0, num, fg);
				next = lx + 6 * len + 2;
			}
		}
	}
	/* The view centre (white) and the tuning centre, the LO artefact (red; none in a sweep). */
	fb[(SCALE_H - 1) * W + W / 2] = RGB565(255, 255, 255);
	int lo_x = sweeping() ? -1 : (int)(((int64_t)lo_khz() - left) * W / span);

	if (lo_x >= 0 && lo_x < (int)W) {
		for (unsigned int y = SCALE_H - 3U; y < SCALE_H; y++) {
			fb[y * W + (unsigned int)lo_x] = RGB565(255, 0, 0);
		}
	}
	blit(SCALE_Y, SCALE_H);
}


/*
 * Per pixel: the Wi-Fi channel centred there (0: none; always, for the labels), the Bluetooth
 * LE advertising channel there (37 to 39, 0: none) and, with the overlay on, whether it is
 * inside Wi-Fi channel 1, 6 or 11 (20 MHz).
 */
#define LABEL_H 10 /* one label row (text and tick); Wi-Fi first, Bluetooth below */
#define DIM(c)   (((c) >> 1) & 0x7befU) /* RGB565 at half brightness */
static int pass_a, pass_b; /* pixels pass_a to pass_b - 1 are inside the analog filters */
#define BLE_COLOR RGB565(0, 220, 255)
static uint8_t ch_at[W];
static uint8_t ble_at[W];
static bool ble_data[W]; /* a Bluetooth LE data channel (every 2 MHz, 2404 to 2478) */
static bool ch_band[W];

static const struct {
	uint8_t ch;
	int khz;
} ble_adv[] = {{37, 2402000}, {38, 2426000}, {39, 2480000}};

/*
 * The services in the S3's tuning range (approximate; region allocations differ). Drawn as a
 * strip with its name under the Bluetooth labels: orange where masts transmit, blue where
 * phones do, violet where both take turns (TDD), pink DECT, grey satellites. ISM (Wi-Fi, BLE,
 * Zigbee: labelled anyway) has no strip. The number keys 1 to 9 and 0 jump to them in order.
 */
#define MAST   RGB565(255, 140, 0)
#define PHONE  RGB565(100, 160, 255)
#define TDD    RGB565(200, 120, 255)
#define DECTC  RGB565(255, 100, 180)
#define SAT    RGB565(170, 170, 170)
struct band {
	int lo_khz, hi_khz;
	const char *name; /* readable, if it fits */
	uint16_t color;   /* 0: no strip */
	const char *code; /* where it does not (NULL: the name only) */
};
static const struct band bands_eu[] = {
	{1805000, 1880000, "4G MAST", MAST, "B3 DL"},   {1880000, 1900000, "DECT", DECTC},
	{1920000, 1980000, "4G USER", PHONE, "B1 UL"}, {2110000, 2170000, "4G MAST", MAST, "B1 DL"},
	{2300000, 2400000, "4G/5G", TDD, "B40"},        {2400000, 2483500, "ISM", 0},
	{2483500, 2500000, "GSTAR", SAT},               {2500000, 2570000, "4G USER", PHONE, "B7 UL"},
	{2570000, 2620000, "4G/5G", TDD, "B38"},                 {2620000, 2690000, "4G MAST", MAST, "B7 DL"},
};
static const struct band bands_us[] = {
	{1850000, 1915000, "4G USER", PHONE, "B25 UL"}, {1920000, 1930000, "DECT6", DECTC},
	{1930000, 1995000, "4G MAST", MAST, "B25 DL"},   {2110000, 2200000, "4G MAST", MAST, "B66 DL"},
	{2305000, 2315000, "4G USER", PHONE, "B30 UL"}, {2320000, 2345000, "SXM", SAT},
	{2350000, 2360000, "4G MAST", MAST, "B30 DL"},   {2400000, 2483500, "ISM", 0},
	{2483500, 2500000, "GSTAR", SAT},                {2496000, 2690000, "B41", TDD},
};
BUILD_ASSERT(ARRAY_SIZE(bands_eu) == 10 && ARRAY_SIZE(bands_us) == 10);

static const struct band *region_bands(void)
{
	return st.region_us ? bands_us : bands_eu;
}

/*
 * The bands in the Wi-Fi row (LABEL_H rows of @p buf, @p n pixels from @p left_khz over
 * @p span_khz): a line through the middle of the text, broken around the name where the
 * Wi-Fi numbers (@p taken) and the other names leave room. They never overlap 2.4 GHz Wi-Fi.
 */
static void draw_bands(uint16_t *buf, unsigned int stride, unsigned int n, int left_khz,
		       int span_khz, bool *taken)
{
	const struct band *b = region_bands();

	for (size_t i = 0; i < ARRAY_SIZE(bands_eu); i++) {
		int a = MAX((int)(((int64_t)b[i].lo_khz - left_khz) * n / span_khz), 0);
		int z = MIN((int)(((int64_t)b[i].hi_khz - left_khz) * n / span_khz), (int)n - 1);
		const char *cand[] = {b[i].name, b[i].code}, *text = NULL;
		int len = 0, lx = 0;
		bool free = false;

		if (b[i].color == 0 || z < a) {
			continue;
		}
		/* The readable name, else the short code, whichever fits first. */
		for (size_t t = 0; t < ARRAY_SIZE(cand) && !free && cand[t] != NULL; t++) {
			text = cand[t];
			len = 6 * (int)strlen(text) - 1;
			lx = (a + z) / 2 - len / 2;
			free = len + 2 <= z - a + 1 && lx >= 1 && lx + len < (int)n - 1;
			for (int x = MAX(lx - 1, 0); free && x <= MIN(lx + len, (int)n - 1); x++) {
				free = !taken[x];
			}
		}
		if (free) {
			draw_text_w(buf, stride, (unsigned int)lx, 1, text, b[i].color);
			for (int x = lx; x < lx + len; x++) {
				taken[x] = true;
			}
		}
		/* The line through the middle of the text row, broken around the name. */
		for (int x = a; x <= z; x++) {
			if (!free || x < lx - 2 || x > lx + len + 1) {
				buf[4U * stride + (unsigned int)x] = b[i].color;
			}
		}
	}
}

static int chan_khz(int ch)
{
	return ch == 14 ? 2484000 : 2407000 + 5000 * ch;
}

static void build_overlay(void)
{
	int span = span_khz(), left = st.view_khz - span / 2;

	/*
	 * Nothing to dim: every span stays inside the analog filters now (40 MHz flattened at its
	 * edges, wider spans swept), so the whole width counts.
	 */
	pass_a = 0;
	pass_b = W;
	memset(ch_at, 0, sizeof(ch_at));
	memset(ble_at, 0, sizeof(ble_at));
	memset(ble_data, 0, sizeof(ble_data));
	for (int khz = 2404000; khz <= 2478000; khz += 2000) {
		int x = (int)(((int64_t)khz - left) * W / span);

		if (khz != 2426000 && x >= 0 && x < (int)W) {
			ble_data[x] = true;
		}
	}
	memset(ch_band, 0, sizeof(ch_band));
	for (size_t i = 0; i < ARRAY_SIZE(ble_adv); i++) {
		int x = (int)(((int64_t)ble_adv[i].khz - left) * W / span);

		if (x >= 0 && x < (int)W) {
			ble_at[x] = ble_adv[i].ch;
		}
	}
	for (int ch = 1; ch <= 12; ch++) { /* 13 and 14: not labelled */
		int x = (int)(((int64_t)chan_khz(ch) - left) * W / span);

		if (x >= 0 && x < (int)W) {
			ch_at[x] = (uint8_t)ch;
		}
		if (st.channels && (ch == 1 || ch == 6 || ch == 11)) {
			int a = (int)(((int64_t)chan_khz(ch) - 10000 - left) * W / span);
			int b = (int)(((int64_t)chan_khz(ch) + 10000 - left) * W / span);

			for (int i = MAX(a, 0); i <= MIN(b, (int)W - 1); i++) {
				ch_band[i] = true;
			}
		}
	}
}

static unsigned int level(float db)
{
	float x = (db - st.floor_db) / st.range_db;

	return (unsigned int)CLAMP(x * 255.0f, 0.0f, 255.0f);
}

static void draw_trace(void)
{
	uint16_t bg = RGB565(0, 0, 0), grid = RGB565(200, 200, 200);
	uint16_t band = RGB565(0, 24, 48), mark = RGB565(110, 110, 110);

	build_overlay();
	for (unsigned int y0 = 0; y0 < PSD_H; y0 += CHUNK_ROWS) {
		unsigned int rows = MIN(CHUNK_ROWS, PSD_H - y0);

		for (unsigned int x = 0; x < W; x++) {
			unsigned int lv = level(trace_db[x]);
			/* At least the white tip: at the floor the trace still shows, on the bottom. */
			unsigned int h = MAX(lv * PSD_H / 256U, 1U);

			for (unsigned int r = 0; r < rows; r++) {
				unsigned int y = y0 + r; /* 0 at the top */
				unsigned int from_bottom = PSD_H - 1U - y;
				/* The view centre: a short tick in the Bluetooth row, level with its ticks. */
				uint16_t c = x == W / 2 && y >= 2U * LABEL_H - 3U && y < 2U * LABEL_H ? grid
					     : ch_band[x] ? band : bg;

				if (ch_at[x] != 0 &&
				    ((y >= LABEL_H - 2U && y < LABEL_H) ||
				     (st.channels && y >= 2U * LABEL_H && (y & 1U) == 0))) {
					c = mark;
				}
				if (ble_data[x] && y == 2U * LABEL_H - 1U) {
					c = RGB565(0, 110, 128);
				}
				if (ble_at[x] != 0 &&
				    ((y >= 2U * LABEL_H - 2U && y < 2U * LABEL_H) ||
				     (st.channels && y >= 2U * LABEL_H && (y & 1U) == 1))) {
					c = BLE_COLOR;
				}

				if (from_bottom < h) {
					c = from_bottom + 1U == h ? RGB565(255, 255, 255)
								  : palette[lv / 2U];
				}
				fb[r * W + x] = ((int)x < pass_a || (int)x >= pass_b) ? DIM(c) : c;
			}
		}
		if (y0 == 0) {
			static bool taken[W];
			int next = 0; /* the first free pixel for a label */

			memset(taken, 0, sizeof(taken));
			for (unsigned int x = 0; x < W; x++) {
				char num[4];
				int lx;

				if (ch_at[x] == 0 || span_khz() > 160000) {
					continue; /* wider, the channels are a pixel or two apart */
				}
				snprintf(num, sizeof(num), "%u", ch_at[x]);
				lx = (int)x - (6 * (int)strlen(num) - 1) / 2; /* centred on the channel */
				if (lx < next || lx < 0 || lx + 6 * (int)strlen(num) > (int)W) {
					continue;
				}
				draw_text(fb, (unsigned int)lx, 1, num, RGB565(255, 220, 0));
				next = lx + 6 * (int)strlen(num) + 1;
				for (int i = lx; i < next - 1; i++) {
					taken[i] = true;
				}
			}
			draw_bands(fb, W, W, st.view_khz - span_khz() / 2, span_khz(), taken);
		}
		if (y0 == LABEL_H) {
			int next = 0; /* labels that would overlap are skipped (wide spans) */

			for (unsigned int x = 0; x < W; x++) {
				char name[8];
				int lx = (int)x - 8; /* "B37" is 17 pixels, centred */

				if (ble_at[x] == 0) {
					continue;
				}
				snprintf(name, sizeof(name), "B%u", ble_at[x]);
				lx = CLAMP(lx, 0, (int)W - 17);
				if (lx < next) {
					continue;
				}
				draw_text(fb, (unsigned int)lx, 1, name, BLE_COLOR);
				next = lx + 19;
			}
		}
		blit(PSD_Y + y0, rows);
	}
}

static void draw_help(void)
{
	static const char *const lines[] = {
		"KEYS                  ESC: CLOSE",
		",  /   SCROLL       (LEFT/RIGHT)",
		";  .   SPAN 1000-2 MHZ (UP/DOWN)",
		"H  HOME     1-9 0  JUMP TO BANDS",
		"N  M   GAIN -/+       G  AGC",
		"-  =   DARKER/BRIGHTER  A  AUTO",
		"[  ]   COLOUR RANGE   W  CH. LINES",
		"P      SPEED: PEAK OF 8,4,2,1,32,16",
		"X  NARROW LINES  F FLAT  C CLIPS",
		"V      AUTO-ROTATE  R  EU/US BANDS",
		"RED: TUNING CENTRE",
		"YELLOW WI-FI 1-12  CYAN BLUETOOTH LE",
		"ORANGE MAST  BLUE USER  VIOLET TDD",
	};
	uint16_t bg = RGB565(16, 16, 40);

	for (unsigned int y0 = 0; y0 < H; y0 += CHUNK_ROWS) {
		unsigned int rows = MIN(CHUNK_ROWS, H - y0), line = y0 / CHUNK_ROWS;

		for (unsigned int k = 0; k < W * rows; k++) {
			fb[k] = bg;
		}
		if (line < ARRAY_SIZE(lines) && rows > 8U) {
			draw_text(fb, 3, 1, lines[line],
				  line == 0 ? RGB565(255, 220, 0) : RGB565(255, 255, 255));
		}
		blit(y0, rows);
	}
	st.help_drawn = true;
}

static void draw_waterfall(void)
{
	for (unsigned int y0 = 0; y0 < WF_H; y0 += CHUNK_ROWS) {
		unsigned int rows = MIN(CHUNK_ROWS, WF_H - y0);

		for (unsigned int r = 0; r < rows; r++) {
			const uint8_t *src = hist_row(y0 + r);

			for (unsigned int x = 0; x < W; x++) {
				uint16_t c = palette[src[x]];

				fb[r * W + x] = ((int)x < pass_a || (int)x >= pass_b) ? DIM(c) : c;
			}
			if (st.channels && ((y0 + r) % 4U) == 0) {
				for (unsigned int x = 0; x < W; x++) {
					if (ch_at[x] != 0) {
						fb[r * W + x] = RGB565(200, 200, 200);
					}
				}
			}
			if (st.channels && ((y0 + r) % 4U) == 2U) {
				for (unsigned int x = 0; x < W; x++) {
					if (ble_at[x] != 0) {
						fb[r * W + x] = BLE_COLOR;
					}
				}
			}
		}
		blit(WF_Y + y0, rows);
	}
}

/* ---- portrait layout: hardware scrolling ---- */

/*
 * Held upright the screen is 135 wide and 240 tall, laid out as in landscape: the settings
 * (two lines), the scale and the trace on top, the waterfall under them with the newest row at
 * the top. On the panel a row is one column, so the waterfall scrolls in hardware: a new row
 * costs one column and a scroll command instead of a redraw. Portrait coordinates are u (0 to
 * PH - 1, left to right, low to high frequency) and v (0 to PV - 1, top to bottom).
 */
#define PH        H  /* portrait width */
#define PV        W  /* portrait height */
#define P_HDR_H   20 /* two lines of settings */
#define P_SCALE_Y P_HDR_H
#define P_TRACE_Y (P_SCALE_Y + SCALE_H)
#define P_TRACE_H 36
#define TOP       (P_TRACE_Y + P_TRACE_H)
#define STRIP     P_TRACE_H /* the tallest piece of the top drawn at once */
#define SIDE_COLS (PV - TOP)
BUILD_ASSERT(SIDE_COLS <= HIST_ROWS);

enum orient {
	ORIENT_LANDSCAPE,
	ORIENT_LEFT_DOWN,  /* the Cardputer's left edge at the bottom */
	ORIENT_RIGHT_DOWN, /* its right edge at the bottom */
};

static const struct device *const dbi = DEVICE_DT_GET(DT_PARENT(DT_CHOSEN(zephyr_display)));
static const struct mipi_dbi_config dbi_cfg =
	MIPI_DBI_CONFIG_DT(DT_CHOSEN(zephyr_display), SPI_WORD_SET(8) | SPI_OP_MODE_CONTROLLER, 0);

#define ST7789_VSCRDEF 0x33
#define ST7789_VSCSAD  0x37
#define ST7789_FRCTRL2 0xc6

/*
 * The panel's refresh: RTNA 0 is about 119 Hz, its default 15 about 60, 31 about 39. With
 * no TE line to time writes against (the Cardputer-Adv wires none), a write and the panel's
 * scan cross somewhere in most frames (tearing); at 119 Hz each torn refresh shows for half
 * as long.
 */
static void panel_refresh(uint8_t rtna)
{
	mipi_dbi_command_write(dbi, &dbi_cfg, ST7789_FRCTRL2, &rtna, 1);
}

static bool scroll_on;
static unsigned int scroll_off; /* the scroll area's first line, relative to its start */
static unsigned int side_rows;  /* rows pushed, for the dotted channel lines */
static uint16_t pbuf[PH * STRIP];  /* a strip in portrait coordinates */
static uint16_t strip[STRIP * PH]; /* the same on the panel */
static uint16_t col_px[PH];

/* Left edge down is a quarter turn anticlockwise: the panel's right edge is on top. */
static bool right_on_top(void)
{
	return st.orient == ORIENT_LEFT_DOWN;
}

/* The panel column of portrait row v. */
static unsigned int panel_x(unsigned int v)
{
	return right_on_top() ? W - 1U - v : v;
}

/* The panel row of portrait column u. */
static unsigned int panel_y(unsigned int u)
{
	return right_on_top() ? u : H - 1U - u;
}

static void panel_cmd(uint8_t cmd, uint16_t a, uint16_t b, uint16_t c, size_t words)
{
	uint8_t d[6] = {a >> 8, a & 0xff, b >> 8, b & 0xff, c >> 8, c & 0xff};

	mipi_dbi_command_write(dbi, &dbi_cfg, cmd, d, 2U * words);
}

/* Back to the landscape layout: the whole frame memory as one unscrolled area. */
static void scroll_stop(void)
{
	if (scroll_on) {
		panel_cmd(ST7789_VSCRDEF, 0, PANEL_LINES, 0, 3);
		panel_cmd(ST7789_VSCSAD, 0, 0, 0, 1);
		scroll_on = false;
	}
}

/* The scroll area's first frame memory line: the waterfall's columns on the panel. */
static unsigned int scroll_tfa(void)
{
	return PANEL_X0 + (right_on_top() ? 0U : TOP);
}

static unsigned int scroll_on_tfa; /* the scroll area scroll_start() set up */

static void scroll_start(void)
{
	unsigned int tfa = scroll_tfa();

	panel_cmd(ST7789_VSCRDEF, tfa, SIDE_COLS, PANEL_LINES - tfa - SIDE_COLS, 3);
	scroll_off = 0;
	panel_cmd(ST7789_VSCSAD, tfa, 0, 0, 1);
	scroll_on = true;
	scroll_on_tfa = tfa;
}

static void blit_rect(unsigned int x, unsigned int y, unsigned int w, unsigned int h,
		      const uint16_t *buf)
{
	struct display_buffer_descriptor desc = {
		.buf_size = w * h * sizeof(uint16_t),
		.width = w,
		.height = h,
		.pitch = w,
	};

	display_write(display, x, y, &desc, buf);
	if (st.shot) {
		shot_rect(x, y, w, h, buf);
	}
}

/* pbuf, rows v0 to v0 + n - 1 of the portrait screen, onto the panel. */
static void blit_strip(unsigned int v0, unsigned int n)
{
	for (unsigned int v = 0; v < n; v++) {
		for (unsigned int u = 0; u < PH; u++) {
			unsigned int x = right_on_top() ? n - 1U - v : v;

			strip[panel_y(u) * n + x] = pbuf[v * PH + u];
		}
	}
	blit_rect(MIN(panel_x(v0), panel_x(v0 + n - 1U)), 0, n, H, strip);
}

/* The landscape pixels c0 to c1 - 1 under portrait column u. */
static void side_span(unsigned int u, unsigned int *c0, unsigned int *c1)
{
	*c0 = u * W / PH;
	*c1 = MAX((u + 1U) * W / PH, *c0 + 1U);
}

static int side_u(int khz)
{
	int span = span_khz();

	return (int)(((int64_t)khz - (st.view_khz - span / 2)) * PH / span);
}

static bool side_dim(unsigned int c0)
{
	return (int)c0 < pass_a || (int)c0 >= pass_b;
}

/* One history row onto the panel column for portrait row v. */
static void side_row(const uint8_t *src, unsigned int n, unsigned int x)
{
	for (unsigned int u = 0; u < PH; u++) {
		unsigned int c0, c1, lv = 0;
		bool chan = false;

		side_span(u, &c0, &c1);
		for (unsigned int c = c0; c < c1; c++) {
			lv = MAX(lv, src[c]);
			chan |= ch_at[c] != 0;
		}
		uint16_t px = side_dim(c0) ? DIM(palette[lv]) : palette[lv];

		if (st.channels && chan && n % 4U == 0U) {
			px = RGB565(200, 200, 200);
		}
		col_px[panel_y(u)] = px;
	}
	blit_rect(x, 0, 1, H, col_px);
}

/* On top: the Wi-Fi channel numbers and the trace, growing up from the waterfall. */
static void side_trace(void)
{
	static uint8_t wifi[PH], ble[PH]; /* the channel centred there, 0: none */
	static bool ble_d[PH], taken[PH];
	int next = 0;

	memset(wifi, 0, sizeof(wifi));
	memset(ble, 0, sizeof(ble));
	memset(ble_d, 0, sizeof(ble_d));
	memset(taken, 0, sizeof(taken));
	for (int ch = 1; ch <= 12; ch++) {
		int u = side_u(chan_khz(ch));

		if (u >= 0 && u < (int)PH) {
			wifi[u] = (uint8_t)ch;
		}
	}
	for (size_t i = 0; i < ARRAY_SIZE(ble_adv); i++) {
		int u = side_u(ble_adv[i].khz);

		if (u >= 0 && u < (int)PH) {
			ble[u] = ble_adv[i].ch;
		}
	}
	for (int khz = 2404000; khz <= 2478000; khz += 2000) {
		int u = side_u(khz);

		if (khz != 2426000 && u >= 0 && u < (int)PH) {
			ble_d[u] = true;
		}
	}

	for (unsigned int u = 0; u < PH; u++) {
		unsigned int c0, c1, lv = 0;
		bool band = false;

		side_span(u, &c0, &c1);
		for (unsigned int c = c0; c < c1; c++) {
			lv = MAX(lv, level(trace_db[c]));
			band |= ch_band[c];
		}
		unsigned int h = MAX(lv * (P_TRACE_H - LABEL_H) / 256U, 1U);

		for (unsigned int v = 0; v < P_TRACE_H; v++) {
			unsigned int from_bottom = P_TRACE_H - 1U - v;
			uint16_t c = band ? RGB565(0, 24, 48) : RGB565(0, 0, 0);

			/* As in landscape: ticks under the labels, dotted lines with the overlay. */
			if (wifi[u] != 0 && ((v >= LABEL_H - 2U && v < LABEL_H) ||
					     (st.channels && v >= 2U * LABEL_H && (v & 1U) == 0))) {
				c = RGB565(110, 110, 110);
			}
			if (ble_d[u] && v == 2U * LABEL_H - 1U) {
				c = RGB565(0, 110, 128);
			}
			if (ble[u] != 0 && ((v >= 2U * LABEL_H - 2U && v < 2U * LABEL_H) ||
					    (st.channels && v >= 2U * LABEL_H && (v & 1U) == 1))) {
				c = BLE_COLOR;
			}
			if (from_bottom < h) {
				c = from_bottom + 1U == h ? RGB565(255, 255, 255) : palette[lv / 2U];
			}
			pbuf[v * PH + u] = side_dim(c0) ? DIM(c) : c;
		}
	}
	for (unsigned int u = 0; u < PH; u++) {
		char num[4];
		int len, lu;

		if (wifi[u] == 0 || span_khz() > 160000) {
			continue;
		}
		len = snprintf(num, sizeof(num), "%u", wifi[u]);
		lu = (int)u - (6 * len - 1) / 2;
		if (lu >= next && lu >= 0 && lu + 6 * len <= (int)PH) {
			draw_text_w(pbuf, PH, (unsigned int)lu, 1, num, RGB565(255, 220, 0));
			next = lu + 6 * len + 1;
			for (int i = lu; i < next - 1; i++) {
				taken[i] = true;
			}
		}
	}
	next = 0;
	for (unsigned int u = 0; u < PH; u++) {
		char name[8];
		int lu = CLAMP((int)u - 8, 0, (int)PH - 17);

		if (ble[u] != 0 && lu >= next) {
			snprintf(name, sizeof(name), "B%u", ble[u]);
			draw_text_w(pbuf, PH, (unsigned int)lu, LABEL_H + 1U, name, BLE_COLOR);
			next = lu + 19;
		}
	}
	draw_bands(pbuf, PH, PH, st.view_khz - span_khz() / 2, span_khz(), taken);
	blit_strip(P_TRACE_Y, P_TRACE_H);
}

/* The settings, two lines: the view and span, then the flags, gain and colour scale. */
static void side_header(void)
{
	char text[32], gain[8];

	for (unsigned int k = 0; k < PH * P_HDR_H; k++) {
		pbuf[k] = RGB565(16, 16, 40);
	}
	if (st.gain == ESP_SDR_RX_GAIN_AUTO) {
		snprintf(gain, sizeof(gain), "AGC");
	} else {
		snprintf(gain, sizeof(gain), "G:%d", st.gain);
	}
	snprintf(text, sizeof(text), "%d.%03d%s %dM P%d%c%c%c", st.view_khz / 1000, st.view_khz % 1000,
		 st.tune_warn ? "!" : " ", span_khz() / 1000, st.peak, st.width_filter ? 'X' : ' ',
		 st.flat ? 'F' : ' ', clip_drop ? 'C' : ' ');
	draw_text_w(pbuf, PH, 2, 1, text, RGB565(255, 255, 255));
	snprintf(text, sizeof(text), "%s %c%d/%d %s", gain, st.auto_floor ? 'A' : 'L',
		 (int)st.floor_db, (int)st.range_db,
		 overloaded() ? "OVL" : st.region_us ? "US" : "EU");
	draw_text_w(pbuf, PH, 2, 11, text, RGB565(255, 255, 255));
	if (overloaded() && strstr(text, "OVL") != NULL) {
		draw_text_w(pbuf, PH, 2 + 6 * (unsigned int)(strstr(text, "OVL") - text), 11, "OVL",
			    ovl_color());
	}
	blit_strip(0, P_HDR_H);
}

/* The scale, as in landscape: MHz on top, ticks under them (taller on whole MHz), markers. */
static void side_scale(void)
{
	static const int ticks[] = {100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000};
	static const int labels[] = {1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000};
	int span = span_khz(), left = st.view_khz - span / 2;
	int tick = ticks[ARRAY_SIZE(ticks) - 1], label = labels[ARRAY_SIZE(labels) - 1];
	char text[12];
	int next = 0;

	for (size_t i = 0; i < ARRAY_SIZE(ticks); i++) {
		if (ticks[i] * (int)PH >= 4 * span) {
			tick = ticks[i];
			break;
		}
	}
	for (size_t i = 0; i < ARRAY_SIZE(labels); i++) {
		if (labels[i] * (int)PH >= 30 * span) {
			label = labels[i];
			break;
		}
	}
	for (unsigned int k = 0; k < PH * SCALE_H; k++) {
		pbuf[k] = RGB565(0, 0, 0);
	}
	for (int f = (left + tick - 1) / tick * tick; f < left + span; f += tick) {
		int u = side_u(f);

		for (unsigned int v = f % 1000 == 0 ? SCALE_H - 2U : SCALE_H - 1U;
		     v < SCALE_H && u >= 0 && u < (int)PH; v++) {
			pbuf[v * PH + (unsigned int)u] = RGB565(160, 160, 160);
		}
		if (f % label == 0) {
			int len = snprintf(text, sizeof(text), "%d", f / 1000);
			int lu = u - (6 * len - 1) / 2;

			if (lu >= next && lu >= 0 && lu + 6 * len <= (int)PH) {
				draw_text_w(pbuf, PH, (unsigned int)lu, 0, text, RGB565(200, 200, 200));
				next = lu + 6 * len + 2;
			}
		}
	}
	int lo_u = sweeping() ? -1 : side_u(lo_khz());

	for (unsigned int v = SCALE_H - 3U; v < SCALE_H && lo_u >= 0 && lo_u < (int)PH; v++) {
		pbuf[v * PH + (unsigned int)lo_u] = RGB565(255, 0, 0);
	}
	pbuf[(SCALE_H - 1U) * PH + PH / 2U] = RGB565(255, 255, 255);
	blit_strip(P_SCALE_Y, SCALE_H);
}

/* Everything, the history unscrolled: newest under the trace. */
/*
 * Everything, newest row first. The scroll stays where it is (a reset showed the old picture
 * shifted until rewritten, a jump): each row goes where it scrolled to, so the redraw
 * replaces the picture in place, top down. Only a new scroll area (rotated) resets it.
 */
static void side_full(void)
{
	unsigned int tfa = scroll_tfa(), top = PANEL_X0 + panel_x(TOP);
	unsigned int back = right_on_top() ? SIDE_COLS - 1U : 1U; /* one row older */

	build_overlay();
	if (!scroll_on || scroll_on_tfa != tfa) {
		scroll_start();
	}
	side_header();
	side_scale();
	side_trace();
	for (unsigned int age = 0; age < SIDE_COLS; age++) {
		side_row(hist_row(age), side_rows - age,
			 tfa + (top - tfa + scroll_off + age * back) % SIDE_COLS - PANEL_X0);
	}
	st.side_dirty = false;
}

/* The newest row: scroll the waterfall down by one and write the row at its top. */
static void side_push(void)
{
	unsigned int tfa = scroll_tfa();
	unsigned int top = PANEL_X0 + panel_x(TOP); /* its frame memory line, unscrolled */

	scroll_off = (scroll_off + (right_on_top() ? 1U : SIDE_COLS - 1U)) % SIDE_COLS;
	panel_cmd(ST7789_VSCSAD, tfa + scroll_off, 0, 0, 1);
	side_row(hist_row(0), ++side_rows, tfa + (top - tfa + scroll_off) % SIDE_COLS - PANEL_X0);
}

/* ---- orientation: BMI270 accelerometer ---- */

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(bmi270))
static const struct device *const imu = DEVICE_DT_GET(DT_NODELABEL(bmi270));
#else
static const struct device *const imu;
#endif
static bool imu_ok;
static float acc[3]; /* m/s^2 */

static void imu_init(void)
{
	struct sensor_value odr = {.val1 = 25}, range = {.val1 = 2}; /* 25 Hz, +-2 g */

	if (imu == NULL || !device_is_ready(imu)) {
		LOG_WRN("IMU not ready");
		return;
	}
	if (sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr) != 0 ||
	    sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &range) != 0) {
		LOG_WRN("IMU setup failed");
		return;
	}
	imu_ok = true;
	LOG_INF("IMU ready");
}

static void imu_poll(void)
{
	struct sensor_value v[3];

	if (!imu_ok || sensor_sample_fetch(imu) != 0 ||
	    sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, v) != 0) {
		return;
	}
	for (int i = 0; i < 3; i++) {
		acc[i] = (float)sensor_value_to_double(&v[i]);
	}
}

/*
 * Gravity along +x: the left edge is down; -x the right edge; +y upright in landscape. With
 * hysteresis, as a phone does: portrait past 60 degrees of tilt, landscape again only within
 * 30 degrees of upright (or upside down); lying flat or in between keeps the layout. A change
 * must hold for three readings.
 */
static void orient_update(void)
{
	static int candidate, count;
	float ax = acc[0], ay = acc[1];
	float tilt = atan2f(fabsf(ax), ay) * (180.0f / 3.14159265f); /* 0 upright, 180 upside down */
	int o = st.orient;

	if (!imu_ok || !st.auto_rotate || sqrtf(ax * ax + ay * ay) < 5.0f) {
		count = 0;
		return;
	}
	if (tilt > 60.0f && tilt < 150.0f) {
		o = ax > 0.0f ? ORIENT_LEFT_DOWN : ORIENT_RIGHT_DOWN;
	} else if (tilt < 30.0f || tilt > 160.0f) {
		o = ORIENT_LANDSCAPE;
	}
	if (o == st.orient) {
		count = 0;
		return;
	}
	count = o == candidate ? count + 1 : 1;
	candidate = o;
	if (count >= 3) {
		st.orient = o;
		st.side_dirty = true;
		count = 0;
	}
}

/*
 * The arrow keys (Fn with ; . , /) as seen on the rotated screen: up and down step the span,
 * left and right scroll.
 */
static char rotate_key(char k)
{
	static const char keys[] = ";/.,"; /* up, right, down, left: clockwise */
	const char *p = strchr(keys, k);
	int turn = st.orient == ORIENT_LEFT_DOWN ? 3 : st.orient == ORIENT_RIGHT_DOWN ? 1 : 0;

	return (p == NULL || k == '\0') ? k : keys[(p - keys + turn) % 4];
}

/* ---- keyboard: TCA8418 on the internal I2C bus ---- */

#if DT_NODE_EXISTS(DT_ALIAS(kbd_i2c))
#define TCA8418_ADDR       0x34
#define TCA8418_CFG        0x01
#define TCA8418_INT_STAT   0x02
#define TCA8418_KEY_LCK_EC 0x03
#define TCA8418_KEY_EVENT  0x04
#define TCA8418_KP_GPIO1   0x1d
#define TCA8418_KP_GPIO2   0x1e
#define TCA8418_KP_GPIO3   0x1f

static const struct device *const kbd_bus = DEVICE_DT_GET(DT_ALIAS(kbd_i2c));
static bool kbd_ok;

/* Cardputer layout, 4 rows of 14 (0: modifier). */
static const char keymap[4][14] = {
	{'`', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '-', '=', '\b'},
	{'\t', 'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p', '[', ']', '\\'},
	{0, 0, 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', ';', '\'', '\n'},
	{0, 0, 0, 'z', 'x', 'c', 'v', 'b', 'n', 'm', ',', '.', '/', ' '},
};

static void kbd_init(void)
{
	uint8_t v;

	if (!device_is_ready(kbd_bus)) {
		LOG_WRN("keyboard bus not ready");
		return;
	}
	/* Rows 0-6 and columns 0-7 in the key matrix, key event interrupts. */
	if (i2c_reg_write_byte(kbd_bus, TCA8418_ADDR, TCA8418_KP_GPIO1, 0x7f) != 0 ||
	    i2c_reg_write_byte(kbd_bus, TCA8418_ADDR, TCA8418_KP_GPIO2, 0xff) != 0 ||
	    i2c_reg_write_byte(kbd_bus, TCA8418_ADDR, TCA8418_KP_GPIO3, 0x00) != 0 ||
	    i2c_reg_write_byte(kbd_bus, TCA8418_ADDR, TCA8418_CFG, 0x01) != 0) {
		LOG_WRN("TCA8418 not found at 0x%02x", TCA8418_ADDR);
		return;
	}
	for (int n = 0; n < 16; n++) {
		if (i2c_reg_read_byte(kbd_bus, TCA8418_ADDR, TCA8418_KEY_EVENT, &v) != 0 ||
		    v == 0) {
			break;
		}
	}
	i2c_reg_write_byte(kbd_bus, TCA8418_ADDR, TCA8418_INT_STAT, 0x1f);
	kbd_ok = true;
	LOG_INF("keyboard ready");
}

/* @return The next pressed key, 0 if none. */
static char kbd_poll(void)
{
	uint8_t ev;

	if (!kbd_ok) {
		return 0;
	}
	for (;;) {
		unsigned int code, row, col, x, y;

		if (i2c_reg_read_byte(kbd_bus, TCA8418_ADDR, TCA8418_KEY_EVENT, &ev) != 0 ||
		    ev == 0) {
			i2c_reg_write_byte(kbd_bus, TCA8418_ADDR, TCA8418_INT_STAT, 0x1f);
			return 0;
		}
		if (!(ev & 0x80)) {
			continue; /* release */
		}
		code = (ev & 0x7fU) - 1U;
		row = code / 10U;
		col = code % 10U;
		/* As M5Cardputer's TCA8418 reader: 7x8 matrix to the 4x14 layout. */
		x = row * 2U + (col > 3U ? 1U : 0U);
		y = col % 4U;
		if (x >= 14U || y >= 4U) {
			LOG_INF("key raw 0x%02x (row %u col %u)", ev, row, col);
			continue;
		}
		LOG_INF("key raw 0x%02x -> '%c'", ev,
			isprint((unsigned char)keymap[y][x]) ? keymap[y][x] : '?');
		if (keymap[y][x] != 0) {
			return keymap[y][x];
		}
	}
}
#else
static void kbd_init(void)
{
}

static char kbd_poll(void)
{
	return 0;
}
#endif

static bool sweeping(void)
{
	return spans[st.span_i].span_mhz >= 80;
}

static void set_span(int i)
{
	if ((spans[i].span_mhz >= 80) != sweeping()) {
		st.dirty_radio = true; /* a sweep leaves the radio wherever it ended */
	}
	if (spans[i].rate != st.rate) {
		st.rate = spans[i].rate;
		st.dirty_radio = true; /* the analog filters follow the rate */
	}
	st.span_i = i;
	st.zoom = spans[i].zoom;
}

/* After the view or span changed: keep it in range and the radio and history with it. */
static void view_changed(int old_view, int old_span, int old_zoom)
{
	st.view_khz = CLAMP(st.view_khz, FREQ_MIN_MHZ * 1000, FREQ_MAX_MHZ * 1000);
	if (sweeping()) { /* a sweep stays within the tuning range */
		st.view_khz = CLAMP(st.view_khz, FREQ_MIN_MHZ * 1000 + span_khz() / 2,
				    FREQ_MAX_MHZ * 1000 - span_khz() / 2);
	}
	follow_view();
	if (st.view_khz != old_view || span_khz() != old_span) {
		hist_remap(old_view, old_span, old_zoom);
	}
}

static void handle_key(char k)
{
	int gmax = esp_sdr_rx_gain_max();
	int old_view = st.view_khz, old_span = span_khz(), old_zoom = st.zoom;

	/* Esc is Fn+` on the Cardputer; ` alone works too. Any other key closes the help. */
	if (k == '`' || k == 0x1b) {
		st.help = !st.help;
		st.help_drawn = false;
		return;
	}
	if (st.help) {
		st.help = false;
	}

	switch (k) {
	case ',':
		st.view_khz -= old_span / 8;
		break;
	case '/':
		st.view_khz += old_span / 8;
		break;
	case ';':
		set_span(MIN(st.span_i + 1, (int)ARRAY_SIZE(spans) - 1));
		break;
	case '.':
		set_span(MAX(st.span_i - 1, 0));
		break;
	case '1': case '2': case '3': case '4': case '5':
	case '6': case '7': case '8': case '9': case '0': {
		const struct band *b = &region_bands()[k == '0' ? 9 : k - '1'];

		st.view_khz = (b->lo_khz + b->hi_khz) / 2;
		break;
	}
	case 'h':
		st.view_khz = FREQ_DEFAULT_MHZ * 1000;
		set_span(SPAN_HOME);
		break;
	case 'p':
		/* Halve: faster, steadier rows lost; from 1 back to the slowest. */
		st.peak = st.peak > 1 ? st.peak / 2 : PEAK_MAX;
		return;
	case '-': /* darker: the floor up */
		st.auto_floor = false;
		st.floor_db += 2.0f;
		return;
	case '=': /* brighter (+): the floor down */
		st.auto_floor = false;
		st.floor_db -= 2.0f;
		return;
	case '[':
		st.range_db = MAX(st.range_db - 5.0f, 10.0f);
		return;
	case ']':
		st.range_db = MIN(st.range_db + 5.0f, 90.0f);
		return;
	case 'a':
		st.auto_floor = !st.auto_floor;
		return;
	case 'x':
		st.width_filter = !st.width_filter;
		break;
	case 'f':
		flat_set(!st.flat);
		return;
	case 'c': /* bursts that clipped: taken again or kept */
		clip_drop = !clip_drop;
		return;
	case 'w':
		st.channels = !st.channels;
		return;
	case 'v':
		st.auto_rotate = !st.auto_rotate;
		return;
	case 'r': /* the band table: Europe/UK or US */
		st.region_us = !st.region_us;
		st.side_dirty = true;
		return;
	case 'g':
		st.gain = ESP_SDR_RX_GAIN_AUTO;
		st.dirty_radio = true;
		return;
	case 'n':
	case 'm':
		if (st.gain == ESP_SDR_RX_GAIN_AUTO) {
			st.gain = gmax / 2;
		} else {
			st.gain = CLAMP(st.gain + (k == 'm' ? 1 : -1), 0, gmax);
		}
		st.dirty_radio = true;
		return;
	default:
		return;
	}
	view_changed(old_view, old_span, old_zoom);
}

static void apply_radio(void)
{
	bool wide = st.rate != ESP_SDR_RATE_16MSPS;
	uint32_t bw_min, bw_max;
	int ret;

	/*
	 * The PHY's 20 MHz mode cuts the receiver off at about +-10 MHz; its 40 MHz mode (2)
	 * passes about +-18 MHz (doc/hardware-quirks.md). Wide rates take that and the widest
	 * low-pass filter; 16 MS/s keeps the 20 MHz mode with a 16 MHz filter.
	 */
	esp_sdr_rx_bandwidth_range(&bw_min, &bw_max);
	ret = esp_sdr_set_channel_bw(wide ? 2U : 0U);
	if (ret != 0) {
		LOG_WRN("channel_bw: %d", ret);
	}
	ret = esp_sdr_rx_set_bandwidth(wide ? 0U : CLAMP((uint32_t)st.bw_mhz, bw_min, bw_max));
	if (ret != 0) {
		LOG_WRN("rx bandwidth: %d", ret);
	}

	ret = esp_sdr_set_freq_offset(st.lo_ofs_khz);
	if (ret != 0 && ret != -ERANGE) {
		LOG_WRN("freq offset %d: %d", st.lo_ofs_khz, ret);
	}
	ret = esp_sdr_set_freq(st.lo_mhz);

	st.tune_warn = ret == -ERANGE;
	if (ret != 0 && ret != -ERANGE) {
		LOG_WRN("set_freq %d: %d", st.lo_mhz, ret);
	}
	ret = esp_sdr_rx_set_gain(st.gain);
	if (ret != 0) {
		LOG_WRN("set_gain %d: %d", st.gain, ret);
		st.gain = ESP_SDR_RX_GAIN_AUTO;
		esp_sdr_rx_set_gain(ESP_SDR_RX_GAIN_AUTO);
	}
	st.dirty_radio = false;
}

/* ---- binary frames to the host ---- */

/*
 * A frame: this header (little endian), the payload, then the CRC-32 (IEEE, as zlib's) of
 * everything after the magic. The host finds frames between text lines by the magic and
 * drops any the CRC rejects (another thread's log output can land in the middle).
 */
struct frame_hdr {
	char magic[4];   /* "ESDR" */
	uint8_t type;    /* FRAME_BINS or FRAME_IQ */
	uint8_t version; /* FRAME_VERSION */
	uint16_t count;  /* bins or complex samples */
	uint32_t seq;    /* frames sent since boot */
	uint32_t ms;     /* uptime */
	int32_t lo_khz;
	uint32_t fs_hz;
	int16_t gain;   /* ESP_SDR_RX_GAIN_AUTO (-1) or the table index */
	uint8_t bursts; /* peak held over this many bursts (bins) */
	uint8_t segments; /* Welch average over this many FFTs per burst (bins) */
	uint32_t len;   /* payload bytes */
} __packed;
BUILD_ASSERT(sizeof(struct frame_hdr) == 32);

#define FRAME_BINS    'B' /* int16 centi-dBFS per bin, from -fs/2 up (the LO at count/2) */
#define FRAME_IQ      'I' /* int16 I, int16 Q per sample, raw units (-512 to 511) */
#define FRAME_RECT    'R' /* $shot: uint16 x, y, w, h, then the RGB565 pixels as drawn (big endian) */
#define FRAME_SHOT    'S' /* $shot done: struct shot_info */
#define FRAME_VERSION 1U
#define FRAME_STALL_MS 200U

static uint32_t frame_seq, frame_crc;
static bool frame_ok;

/*
 * Whole FIFO loads to the console, waiting while the host catches up. uart_poll_out() sends
 * every byte in a USB packet of its own, and drops one when the FIFO is full after a pause.
 * @return false when nobody has read for FRAME_STALL_MS.
 */
static bool con_write(const void *data, size_t len)
{
	const uint8_t *p = data;
	uint32_t t0 = k_uptime_get_32();

	while (len > 0U) {
		int n = uart_fifo_fill(console, p, (int)MIN(len, 64U));

		if (n > 0) {
			p += n;
			len -= (size_t)n;
			t0 = k_uptime_get_32();
		} else if (k_uptime_get_32() - t0 > FRAME_STALL_MS) {
			return false;
		}
	}
	return true;
}

static void con_printf(const char *fmt, ...)
{
	static char line[512]; /* the status line passed 256 */
	va_list ap;
	int n;

	va_start(ap, fmt);
	n = vsnprintf(line, sizeof(line), fmt, ap);
	va_end(ap);
	if (n > 0) {
		con_write(line, MIN((size_t)n, sizeof(line) - 1U));
	}
}

static void frame_write(const void *data, size_t len)
{
	frame_crc = crc32_ieee_update(frame_crc, data, len);
	if (frame_ok && !con_write(data, len)) {
		frame_ok = false; /* nobody reading: give up the requests */
		st.bins_left = 0;
		st.iq_left = 0;
	}
}

static void frame_begin(uint8_t type, uint16_t count, uint32_t len, unsigned int bursts,
			unsigned int segments)
{
	struct frame_hdr h = {
		.magic = {'E', 'S', 'D', 'R'},
		.type = type,
		.version = FRAME_VERSION,
		.count = sys_cpu_to_le16(count),
		.seq = sys_cpu_to_le32(frame_seq++),
		.ms = sys_cpu_to_le32(k_uptime_get_32()),
		.lo_khz = (int32_t)sys_cpu_to_le32((uint32_t)lo_khz()),
		.fs_hz = sys_cpu_to_le32(esp_sdr_rx_rate_hz(st.rate)),
		.gain = (int16_t)sys_cpu_to_le16((uint16_t)st.gain),
		.bursts = (uint8_t)bursts,
		.segments = (uint8_t)segments,
		.len = sys_cpu_to_le32(len),
	};

	frame_ok = true;
	frame_crc = 0U;
	frame_write(h.magic, 4); /* not in the CRC */
	frame_crc = 0U;
	frame_write(&h.type, sizeof(h) - 4U);
}

static void frame_end(void)
{
	uint32_t crc = sys_cpu_to_le32(frame_crc);

	frame_write(&crc, sizeof(crc));
}

/*
 * $shot: a redraw of everything goes to the host as well, each display write as a FRAME_RECT
 * in the panel's coordinates, then a FRAME_SHOT with what the host needs to show them as the
 * screen does: the display reads frame memory line PANEL_X0 + x for column x, except that with
 * hardware scrolling on (portrait), line tfa + i of the scroll area shows memory line
 * tfa + (vsp - tfa + i) % vsa.
 */
struct shot_info {
	uint16_t width, height, panel_x0, orient;
	uint16_t scroll_on, tfa, vsa, vsp;
} __packed;

static void shot_rect(unsigned int x, unsigned int y, unsigned int w, unsigned int h,
		      const uint16_t *buf)
{
	uint16_t r[4] = {sys_cpu_to_le16(x), sys_cpu_to_le16(y), sys_cpu_to_le16(w),
			 sys_cpu_to_le16(h)};

	frame_begin(FRAME_RECT, 0U, sizeof(r) + w * h * 2U, 0U, 0U);
	frame_write(r, sizeof(r));
	frame_write(buf, w * h * 2U);
	frame_end();
}

static void shot_end(void)
{
	struct shot_info i = {
		.width = sys_cpu_to_le16(W),
		.height = sys_cpu_to_le16(H),
		.panel_x0 = sys_cpu_to_le16(PANEL_X0),
		.orient = sys_cpu_to_le16(st.help ? ORIENT_LANDSCAPE : st.orient),
		.scroll_on = sys_cpu_to_le16(scroll_on),
		.tfa = sys_cpu_to_le16(scroll_on_tfa),
		.vsa = sys_cpu_to_le16(SIDE_COLS),
		.vsp = sys_cpu_to_le16(scroll_on_tfa + scroll_off),
	};

	frame_begin(FRAME_SHOT, 0U, sizeof(i), 0U, 0U);
	frame_write(&i, sizeof(i));
	frame_end();
}

/* A burst's raw I/Q as a frame. */
static void send_iq(const struct esp_sdr_rx_burst *b)
{
	int16_t chunk[2 * 64];

	frame_begin(FRAME_IQ, (uint16_t)b->count, (uint32_t)(b->count * 4U), 1U, 0U);
	for (size_t j = 0; j < b->count; j += 64U) {
		size_t n = MIN(b->count - j, 64U);

		for (size_t i = 0; i < n; i++) {
			chunk[2 * i] = (int16_t)sys_cpu_to_le16((uint16_t)esp_sdr_rx_i(b->words[j + i]));
			chunk[2 * i + 1] =
				(int16_t)sys_cpu_to_le16((uint16_t)esp_sdr_rx_q(b->words[j + i]));
		}
		frame_write(chunk, n * 4U);
	}
	frame_end();
	if (st.iq_left > 0) {
		st.iq_left--;
	}
}

/* The row's spectrum before any patching, in power (times norm for dBFS), as a frame. */
static void send_bins(const float *p, float norm, unsigned int bursts, unsigned int segments)
{
	static int16_t out[NFFT];

	for (unsigned int k = 0; k < NFFT; k++) {
		float v = p[(k + NFFT / 2U) % NFFT] * norm;
		float cdb = v > 0.0f ? 1000.0f * log10f(v) : -32768.0f;

		out[k] = (int16_t)sys_cpu_to_le16((uint16_t)(int16_t)CLAMP(lroundf(cdb), -32768L, 32767L));
	}
	frame_begin(FRAME_BINS, NFFT, sizeof(out), bursts, segments);
	frame_write(out, sizeof(out));
	frame_end();
	if (st.bins_left > 0) {
		st.bins_left--;
	}
}

/* ---- spectrum ---- */

/* One burst's Welch average into psd; @return the segment count, or a negative error. */
static void fft_init(void)
{
	unsigned int bits = LOG2(NFFT);

	esp_sdr_fft_twiddles(tw_cos, tw_sin, NFFT);
	for (unsigned int i = 0; i < NFFT; i++) {
		unsigned int r = 0;

		for (unsigned int b = 0; b < bits; b++) {
			r |= ((i >> b) & 1U) << (bits - 1U - b);
		}
		rev[i] = (uint16_t)r;
	}
	for (unsigned int i = 0; i < NFFT / 2U; i++) {
		float a = 6.28318531f * (float)(rev[i] >> 1) / (float)NFFT;

		dsp_w[2U * i] = cosf(a);
		dsp_w[2U * i + 1U] = sinf(a);
	}
}

/* Two tones, either side of DC: the kernel must agree with the C FFT, mirror included. */
static void fft_check(void)
{
	float *re = psd, *im = peak; /* free at boot */
	float err = 0.0f, top = 0.0f;
	uint32_t t0, t_dsp, t_c;

	for (unsigned int k = 0; k < NFFT; k++) {
		float a = 6.28318531f * 100.0f * (float)k / (float)NFFT;
		float b = -6.28318531f * 37.3f * (float)k / (float)NFFT;

		re[k] = cosf(a) + 0.01f * cosf(b);
		im[k] = sinf(a) + 0.01f * sinf(b);
		cbuf[2U * k] = re[k];
		cbuf[2U * k + 1U] = im[k];
	}
	t0 = k_cycle_get_32();
	dsps_fft2r_fc32_aes3_(cbuf, NFFT, dsp_w);
	t_dsp = k_cycle_get_32() - t0;
	t0 = k_cycle_get_32();
	esp_sdr_fft(re, im, NFFT, tw_cos, tw_sin, NFFT);
	t_c = k_cycle_get_32() - t0;
	for (unsigned int k = 0; k < NFFT; k++) {
		float pc = re[rev[k]] * re[rev[k]] + im[rev[k]] * im[rev[k]];
		float pd = cbuf[2U * k] * cbuf[2U * k] + cbuf[2U * k + 1U] * cbuf[2U * k + 1U];

		err = MAX(err, fabsf(pc - pd));
		top = MAX(top, pc);
	}
	use_dsp = err <= 1e-4f * top;
	LOG_INF("fft: esp-dsp %u us, C %u us, error %.1e: %s", k_cyc_to_us_floor32(t_dsp),
		k_cyc_to_us_floor32(t_c), (double)(err / top), use_dsp ? "esp-dsp" : "C");
}

/*
 * Each dump word also carries the gain it was taken at, bits 26 to 20 (bit 27 flags the odd
 * sample, the modem's packet detector most likely, and 31 to 28 its state machine: neither
 * follows clipping). At a fixed gain it matched in every burst checked, but a burst taken
 * at another gain (or, with the AGC, one where it changed) would smear the row: taken again,
 * up to twice, then kept so the screen never stalls. A sample at full scale (+-511) means the
 * ADC clipped, which splatters across the whole span, a bright line through the row: such a
 * burst is left out of the row's peak hold for another, a few ms later (taken again at once
 * it mostly fell in the same strong packet: a third were kept), up to CLIP_EXTRA a row
 * (peak_bursts()). OVL in the header: yellow when one was left out, red when one was kept.
 */
#define CLIP_EXTRA 2
#define BURST_TRIES 3
#define OVL_MS      1000U
static uint32_t gain_retakes, clip_retakes, clip_kept, ovl_ms, ovl_kept_ms;
static bool last_clipped; /* the last capture_psd_n()'s burst hit full scale */

/* OVL yellow: clipping seen, the burst taken again; red: a burst that clipped was kept. */
static uint16_t ovl_color(void)
{
	return ovl_kept_ms != 0U && k_uptime_get_32() - ovl_kept_ms < OVL_MS ? RGB565(255, 60, 60)
									    : RGB565(255, 220, 0);
}

static bool overloaded(void)
{
	return ovl_ms != 0U && k_uptime_get_32() - ovl_ms < OVL_MS;
}

static inline unsigned int word_gain(uint32_t w)
{
	return (w >> 20) & 0x7fU;
}

static bool burst_gain_ok(const struct esp_sdr_rx_burst *b)
{
	unsigned int g0 = word_gain(b->words[0]), g1 = word_gain(b->words[b->count - 1U]);

	return st.gain == ESP_SDR_RX_GAIN_AUTO ? g0 == g1
						: g0 == (unsigned int)st.gain && g1 == g0;
}

static bool burst_clipped(const struct esp_sdr_rx_burst *b)
{
	for (size_t j = 0; j < b->count; j++) {
		if (abs(esp_sdr_rx_i(b->words[j])) >= 511 || abs(esp_sdr_rx_q(b->words[j])) >= 511) {
			return true;
		}
	}
	return false;
}

static int capture_psd_n(size_t count)
{
	struct esp_sdr_rx_burst b;
	unsigned int segments;
	float dc_i = 0.0f, dc_q = 0.0f;
	int ret;

	for (int t = 0;; t++) {
		ret = esp_sdr_rx_capture(st.rate, count, &b);
		if (ret != 0) {
			return ret;
		}
		if (b.count == 0U || burst_gain_ok(&b) || t == BURST_TRIES - 1) {
			break;
		}
		gain_retakes++;
	}
	last_clipped = b.count > 0U && burst_clipped(&b);
	if (last_clipped) {
		ovl_ms = k_uptime_get_32();
	}
	if (st.iq_left != 0) {
		send_iq(&b);
	}
	for (size_t j = 0; j < b.count; j++) {
		dc_i += (float)esp_sdr_rx_i(b.words[j]);
		dc_q += (float)esp_sdr_rx_q(b.words[j]);
	}
	dc_i /= (float)b.count;
	dc_q /= (float)b.count;

	segments = b.count / NFFT;
	memset(psd, 0, sizeof(psd));
	for (unsigned int s = 0; s < segments; s++) {
		const uint32_t *w = &b.words[s * NFFT];

		if (use_dsp) {
			for (unsigned int k = 0; k < NFFT; k++) {
				cbuf[2U * k] = ((float)esp_sdr_rx_i(w[k]) - dc_i) * window[k];
				cbuf[2U * k + 1U] = ((float)esp_sdr_rx_q(w[k]) - dc_q) * window[k];
			}
			dsps_fft2r_fc32_aes3_(cbuf, NFFT, dsp_w);
			for (unsigned int k = 0; k < NFFT; k++) {
				psd[rev[k]] += cbuf[2U * k] * cbuf[2U * k] +
					       cbuf[2U * k + 1U] * cbuf[2U * k + 1U];
			}
			continue;
		}
		float *re = cbuf, *im = &cbuf[NFFT]; /* the C FFT: cbuf's halves */

		for (unsigned int k = 0; k < NFFT; k++) {
			re[k] = ((float)esp_sdr_rx_i(w[k]) - dc_i) * window[k];
			im[k] = ((float)esp_sdr_rx_q(w[k]) - dc_q) * window[k];
		}
		esp_sdr_fft(re, im, NFFT, tw_cos, tw_sin, NFFT);
		for (unsigned int k = 0; k < NFFT; k++) {
			psd[k] += re[k] * re[k] + im[k] * im[k];
		}
	}
	return (int)segments;
}

/* @p bursts bursts' per-bin maximum into peak[], those that clipped left out (clip_drop). */
static int peak_bursts(unsigned int bursts, size_t samples)
{
	unsigned int got = 0, extra = 0;
	int segments = 0;

	while (got < bursts) {
		int ret = capture_psd_n(samples);

		if (ret < 0) {
			return ret;
		}
		segments = ret;
		if (last_clipped && clip_drop) {
			if (extra < CLIP_EXTRA) {
				extra++;
				clip_retakes++;
				continue;
			}
			ovl_kept_ms = k_uptime_get_32();
			clip_kept++;
		} else if (last_clipped) {
			ovl_kept_ms = k_uptime_get_32();
		}
		if (got == 0U) {
			memcpy(peak, psd, sizeof(peak));
		} else {
			for (unsigned int k = 0; k < NFFT; k++) {
				peak[k] = MAX(peak[k], psd[k]);
			}
		}
		got++;
	}
	return segments;
}

/*
 * The receiver's own shape about the LO (f): the analog filters' roll-off (-2.7 dB at
 * +-20 MHz at 80 MS/s), a broad bump around the LO, a few faint lines at fixed offsets. It is
 * the same at every tuning from 1800 to 2800 MHz (within 0.2 to 0.4 dB), cold or warm, and
 * nearly so at every gain, so it is measured once (tools/flatfield.py, the median over 48
 * tunings) and divided out of each capture: one table a rate and gain set (flat_tables.h).
 * The LO's own line varies from capture to capture, 6 to 27 dB: lo_bridge()'s.
 */
static void flat_apply(void)
{
	int r = st.rate == ESP_SDR_RATE_80MSPS ? 0 : st.rate == ESP_SDR_RATE_40MSPS ? 1 : 2;
	int g = st.gain == ESP_SDR_RX_GAIN_AUTO ? 1
		: st.gain <= FLAT_GAIN_LOW	? 0
		: st.gain <= FLAT_GAIN_MID	? 1
						: 2;
	const float *t = flat_tab[r][g];

	for (unsigned int k = 0; k < NFFT; k++) {
		psd[k] *= t[k];
	}
}

/* 10 log10(x) within about 0.02 dB for normal x > 0: the exponent, a parabola over the mantissa. */
static inline float fast_db(float x)
{
	union {
		float f;
		uint32_t i;
	} u = {.f = x};
	float e = (float)((int)((u.i >> 23) & 0xffU) - 128);

	u.i = (u.i & 0x7fffffU) | 0x3f800000U;
	return 3.0103f * (e + (-0.34484843f * u.f + 2.02466578f) * u.f - 0.67487759f);
}

/* f, $flat */
static void flat_set(bool on)
{
	st.flat = on;
}

/*
 * The LO leakage and DC residue, with the skirt around them (+3 dB out to about 110 kHz at
 * every LO): bridged over 120 kHz either side, from the mean of three bins beyond each end.
 */
static void lo_bridge(void)
{
	int w = (int)ceilf(120.0f * (float)NFFT / (float)fs_khz());
	float l = 0.0f, r = 0.0f;

	for (int j = 1; j <= 3; j++) {
		l += psd[(unsigned int)((int)NFFT - w - j)];
		r += psd[(unsigned int)(w + j)];
	}
	l /= 3.0f;
	r /= 3.0f;
	for (int k = -w; k <= w; k++) {
		psd[(unsigned int)((k + (int)NFFT) % (int)NFFT)] =
			l + (r - l) * (float)(k + w + 1) / (float)(2 * w + 2);
	}
}

static void commit_row(float sum, unsigned int valid);

/*
 * The width filter (x, X in the header): the receiver's own lines are all narrow (its clock
 * harmonics, spurs that move with the LO in ways no formula covers, more of them under
 * 1842 MHz), so, blind: a run of bins standing more than width_db above the mean (in dB) of
 * a ring one to three widths out is a line when that core is no wider than width_khz; it and
 * its skirts (above WIDTH_SKIRT_DB, up to width_khz either side) are set to that mean. Steady
 * or not, they go, but so do real narrow signals: NB-IoT and GSM carriers (180, 200 kHz) and
 * narrower. Wi-Fi, Bluetooth, 4G stand far wider. peak[] holds the dB levels' running sum.
 */
#define WIDTH_SKIRT_DB 1.5f
static int width_khz = 600; /* $width: the spur cluster under 1842 MHz wanders over 1 MHz */
static float width_db = 4.0f;
static float width_ring(int k, int near, int far)
{
	float sum = 0.0f;

	for (int side = -1; side <= 1; side += 2) {
		int a = k + side * near, b = k + side * far;
		int lo = MIN(a, b), hi = MAX(a, b);

		if (lo >= 0 && hi < (int)NFFT) {
			sum += peak[hi] - (lo > 0 ? peak[lo - 1] : 0.0f);
		} else {
			for (int j = lo; j <= hi; j++) {
				unsigned int m = (unsigned int)((j % (int)NFFT + (int)NFFT) % (int)NFFT);

				sum += peak[m] - (m > 0U ? peak[m - 1U] : 0.0f);
			}
		}
	}
	return sum / (float)(2 * (far - near + 1));
}

static void width_filter(void)
{
	int w = MAX(3, (int)lroundf((float)width_khz * (float)NFFT / (float)fs_khz()));
	float total = 0.0f;

	for (unsigned int k = 0; k < NFFT; k++) {
		total += fast_db(psd[k] + 1e-20f);
		peak[k] = total; /* bins 0 to k */
	}
	for (int k = 0; k < (int)NFFT; k++) {
		float base = width_ring(k, w, 3 * w);

		if (fast_db(psd[k] + 1e-20f) - base <= width_db) {
			continue;
		}
		int a = k, b = k;

		/* The core, above width_db: narrow or not. */
		while (b < (int)NFFT - 1 && fast_db(psd[b + 1] + 1e-20f) - base > width_db &&
		       b - a < w) {
			b++;
		}
		if (b - a + 1 > w) {
			while (b < (int)NFFT - 1 && fast_db(psd[b + 1] + 1e-20f) - base > width_db) {
				b++;
			}
			k = b; /* wide: a real signal's */
			continue;
		}
		/* Its skirts, up to w bins either side. */
		for (int n = 0; n < w && a > 0 && fast_db(psd[a - 1] + 1e-20f) - base > WIDTH_SKIRT_DB;
		     n++) {
			a--;
		}
		for (int n = 0; n < w && b < (int)NFFT - 1 &&
				fast_db(psd[b + 1] + 1e-20f) - base > WIDTH_SKIRT_DB;
		     n++) {
			b++;
		}
		float fill = powf(10.0f, 0.1f * base);

		for (int j = a; j <= b; j++) {
			psd[j] = fill;
		}
		k = b;
	}
}

/*
 * The spans of 80 MHz and wider sweep (80 in 3 steps; a single capture's filters pass only
 * about +-18 MHz, the rest dim): the view in steps of at most 32 MHz, each captured at
 * 80 MS/s with the LO at its centre, of which only the middle (inside the filters' +-18 MHz,
 * clear of the edges) is used, its shape flattened like any capture's (flat_apply(); without
 * it the shape repeats at every step). A step is a retune (about 1 ms with the short settle)
 * and SWEEP_SAMPLES samples (4 FFTs averaged), the same capture as the other spans; peak hold
 * shares the P bursts of a row out over the steps (at least one each), so a row takes about
 * as long as at 16 MHz and the scroll speed falls smoothly with the width. No frames.
 */
#define SWEEP_STEP_KHZ 32000
#define SWEEP_SAMPLES  SAMPLES
static int capture_sweep(void)
{
	int span = span_khz(), left = st.view_khz - span / 2;
	int n = DIV_ROUND_UP(span, SWEEP_STEP_KHZ), step = span / n;
	/* P bursts a row as elsewhere, shared out over the steps (at least one each). */
	unsigned int bursts = DIV_ROUND_UP((unsigned int)st.peak, (unsigned int)n);
	float bin_khz = (float)fs_khz() / (float)NFFT, sum = 0.0f;
	unsigned int valid = 0;

	for (unsigned int c = 0; c < W; c++) {
		row_db[c] = FLOOR_DB;
	}
	for (int i = 0; i < n; i++) {
		int a = left + i * step, centre = (a + step / 2 + 500) / 1000 * 1000;
		int segments = 0;
		int ret = esp_sdr_set_freq((uint32_t)(centre / 1000));

		if (ret != 0 && ret != -ERANGE) {
			continue;
		}
		segments = peak_bursts(bursts, SWEEP_SAMPLES);
		if (segments < 0) {
			return segments;
		}
		memcpy(psd, peak, sizeof(psd));
		if (st.flat) {
			flat_apply();
		}
		lo_bridge();
		if (st.width_filter) {
			width_filter();
		}

		float norm = 1.0f / (FULLSCALE * FULLSCALE * window_gain * (float)segments);
		int xa = (int)((int64_t)(a - left) * W / span);
		int xb = (int)((int64_t)(a + step - left) * W / span);

		for (int x = MAX(xa, 0); x < MIN(xb, (int)W); x++) {
			/* The pixel's frequencies, in bins about this step's LO. */
			float fa = (float)left + (float)x * (float)span / (float)W - (float)centre;
			int ma = (int)floorf(fa / bin_khz) + (int)(NFFT / 2U);
			int mb = MAX((int)floorf((fa + (float)span / (float)W) / bin_khz) +
					     (int)(NFFT / 2U),
				     ma + 1);
			float p = 0.0f;

			ma = MAX(ma, 0);
			mb = MIN(mb, (int)NFFT);
			for (int m = ma; m < mb; m++) {
				p = MAX(p, psd[((unsigned int)m + NFFT / 2U) % NFFT]);
			}
			p *= norm;
			row_db[x] = p > 0.0f ? 10.0f * log10f(p) : FLOOR_DB;
			sum += row_db[x];
			valid++;
		}
		st.lo_mhz = centre / 1000;
		st.lo_ofs_khz = 0;
	}
	commit_row(sum, valid);
	return 0;
}

static int capture_row(void)
{
	/* Zoomed in, each of the zoom rows per redraw takes its share of the peak bursts. */
	unsigned int bursts = MAX((unsigned int)st.peak / (unsigned int)st.zoom, 1U);
	int segments = peak_bursts(bursts, SAMPLES);

	if (segments < 0) {
		return segments;
	}
	memcpy(psd, peak, sizeof(psd));

	float norm = 1.0f / (FULLSCALE * FULLSCALE * window_gain * (float)segments);
	float sum = 0.0f;
	unsigned int valid = 0;

	if (st.bins_left != 0 && !st.bins_post) {
		send_bins(psd, norm, bursts, (unsigned int)segments);
	}

	if (st.flat) {
		flat_apply();
	}
	if (st.width_filter) {
		width_filter();
	}

	lo_bridge();
	if (st.bins_left != 0 && st.bins_post) {
		send_bins(psd, norm, bursts, (unsigned int)segments);
	}

	/* The automatic floor follows the pixels inside the analog filters: all of them now. */
	int in_a = pass_a, in_b = pass_b;

	/* Pixel edges in shifted bins (NFFT/2 is the LO). */
	float bin_khz = (float)fs_khz() / (float)NFFT;
	float px_bins = (float)span_khz() / (float)W / bin_khz;
	float left = (float)(st.view_khz - span_khz() / 2 - lo_khz()) / bin_khz +
		     (float)(NFFT / 2U);

	for (unsigned int c = 0; c < W; c++) {
		int ma = (int)floorf(left + (float)c * px_bins);
		int mb = MAX((int)floorf(left + (float)(c + 1U) * px_bins), ma + 1);
		float p = 0.0f;

		if (ma < 0 || mb > (int)NFFT) {
			row_db[c] = FLOOR_DB;
			continue;
		}
		for (int m = ma; m < mb; m++) {
			p = MAX(p, psd[((unsigned int)m + NFFT / 2U) % NFFT]);
		}
		p *= norm;
		row_db[c] = p > 0.0f ? 10.0f * log10f(p) : FLOOR_DB;
		if ((int)c >= in_a && (int)c < in_b) {
			sum += row_db[c];
			valid++;
		}
	}

	commit_row(sum, valid);
	return 0;
}

/* row_db[] done: the automatic floor, the history, the trace and the console dump. */
static void commit_row(float sum, unsigned int valid)
{
	if (st.auto_floor && valid > 0) {
		/* Track the mean level; the floor sits a little below it. */
		st.floor_db += 0.05f * (sum / (float)valid - 5.0f - st.floor_db);
	}

	head = (head + HIST_ROWS - 1U) % HIST_ROWS;
	for (unsigned int c = 0; c < W; c++) {
		hist[head][c] = (uint8_t)level(row_db[c]);
		trace_db[c] = st.trace_reset ? row_db[c]
					     : trace_db[c] + 0.3f * (row_db[c] - trace_db[c]);
	}
	st.trace_reset = false;
	for (unsigned int c = 0; c < W; c++) {
		dump_db[c] = dump_fresh ? row_db[c] : MAX(dump_db[c], row_db[c]);
	}
	dump_fresh = false;
}

/*
 * One console line: the view, then the 240 pixels' maximum in dB since the last line, for
 * looking at the spectrum from a host (keys can be typed on the console too).
 */
static void dump_spectrum(void)
{
	static char line[64 + W * 5];
	int len = snprintf(line, sizeof(line),
			   "spec lo %d view %d span %d rate %u zoom %d peak %d floor %d:", st.lo_mhz,
			   st.view_khz, span_khz(), (unsigned int)fs_khz(), st.zoom, st.peak,
			   (int)st.floor_db);

	for (unsigned int c = 0; c < W && len < (int)sizeof(line) - 8; c++) {
		len += snprintf(&line[len], sizeof(line) - (size_t)len, " %d",
				(int)lroundf(MAX(dump_db[c], -999.0f)));
	}
	line[len++] = '\n';
	/* Straight to the UART: one log message per number would overflow the log buffer. */
	con_write(line, (size_t)len);
	dump_fresh = true;
}

/* ---- console commands ---- */

/* The main loop's longest pass since the last status, in ms: hesitations on the screen. */
static uint32_t loop_max_ms;

static void print_status(void)
{
	con_printf("stat lo %d lo_khz %d view %d span %d rate %d zoom %d gain %d peak %d hide %d floor %d "
	       "range %d nfft %u samples %u text %d screen %d bins %d iq %d region %s orient %d rotate %d acc %d %d %d loop_max %u flat %d retakes %u clips %u kept %u ovl %d\n",
	       st.lo_mhz, lo_khz(), st.view_khz, span_khz(), fs_khz(), st.zoom, st.gain, st.peak,
	       st.width_filter, (int)lroundf(st.floor_db), (int)lroundf(st.range_db), NFFT,
	       SAMPLES, st.text, st.screen, st.bins_left, st.iq_left, st.region_us ? "us" : "eu", st.orient, st.auto_rotate,
	       (int)lroundf(acc[0] * 10.0f), (int)lroundf(acc[1] * 10.0f), (int)lroundf(acc[2] * 10.0f),
	       (unsigned int)loop_max_ms, st.flat, (unsigned int)gain_retakes, (unsigned int)clip_retakes, (unsigned int)clip_kept,
	       overloaded());
	loop_max_ms = 0U;

}

static bool parse_flag(const char *a, bool *out)
{
	if (strcmp(a, "on") == 0 || strcmp(a, "1") == 0) {
		*out = true;
	} else if (strcmp(a, "off") == 0 || strcmp(a, "0") == 0) {
		*out = false;
	} else {
		return false;
	}
	return true;
}

static bool parse_int(const char *a, long *out)
{
	char *end;

	*out = strtol(a, &end, 10);
	return *a != '\0' && *end == '\0';
}

/* A count: none for 1, "on" for all (-1), "off" for none. */
static bool parse_count(const char *a, int *out)
{
	long v;

	if (*a == '\0') {
		*out = 1;
	} else if (strcmp(a, "on") == 0) {
		*out = -1;
	} else if (strcmp(a, "off") == 0) {
		*out = 0;
	} else if (parse_int(a, &v) && v >= 0) {
		*out = (int)MIN(v, (long)INT_MAX);
	} else {
		return false;
	}
	return true;
}

/* One "$name arg" line from the console; every command answers "ok ..." or "err ...". */
static void handle_cmd(char *line)
{
	int old_view = st.view_khz, old_span = span_khz(), old_zoom = st.zoom;
	char *arg = strchr(line, ' ');
	bool flag;
	long v;

	if (arg != NULL) {
		*arg++ = '\0';
		while (*arg == ' ') {
			arg++;
		}
	} else {
		arg = "";
	}

	if (*line == '\0') {
		return;
	} else if (strcmp(line, "help") == 0) {
		con_printf("ok help status, bins [n|on|off], post on|off, iq [n|off], text on|off, "
			   "screen on|off, rotate auto|land|left|right, "
		       "tune <kHz>, lo <MHz>, span <MHz>, gain <n|auto>, peak <1-32|on|off>, hide on|off, width <kHz> [dB], clip drop|keep, shot, flat on|off, region eu|us\n");
	} else if (strcmp(line, "status") == 0) {
		print_status();
	} else if (strcmp(line, "bins") == 0 && parse_count(arg, &st.bins_left)) {
		con_printf("ok bins %d\n", st.bins_left);
	} else if (strcmp(line, "iq") == 0 && parse_count(arg, &st.iq_left) && st.iq_left >= 0) {
		con_printf("ok iq %d\n", st.iq_left);
	} else if (strcmp(line, "post") == 0 && parse_flag(arg, &flag)) {
		st.bins_post = flag;
		con_printf("ok post %d\n", st.bins_post);
	} else if (strcmp(line, "text") == 0 && parse_flag(arg, &flag)) {
		st.text = flag;
		con_printf("ok text %d\n", st.text);
	} else if (strcmp(line, "tune") == 0 && parse_int(arg, &v)) {
		st.view_khz = (int)CLAMP(v, FREQ_MIN_MHZ * 1000L, FREQ_MAX_MHZ * 1000L);
		view_changed(old_view, old_span, old_zoom);
		con_printf("ok tune %d lo %d\n", st.view_khz, st.lo_mhz);
	} else if (strcmp(line, "lo") == 0 && parse_int(arg, &v) && v >= FREQ_MIN_MHZ &&
		   v <= FREQ_MAX_MHZ) {
		/* The LO alone; the view moves only if it would leave the capture. */
		st.lo_mhz = (int)v;
		st.lo_ofs_khz = 0;
		st.dirty_radio = true;
		if (st.view_khz - span_khz() / 2 < lo_khz() - fs_khz() / 2 ||
		    st.view_khz + span_khz() / 2 > lo_khz() + fs_khz() / 2) {
			st.view_khz = st.lo_mhz * 1000;
			hist_remap(old_view, old_span, old_zoom);
		}
		con_printf("ok lo %d view %d\n", st.lo_mhz, st.view_khz);
	} else if (strcmp(line, "span") == 0 && parse_int(arg, &v)) {
		int i;

		for (i = 0; i < (int)ARRAY_SIZE(spans); i++) {
			if (spans[i].span_mhz == v) {
				break;
			}
		}
		if (i == (int)ARRAY_SIZE(spans)) {
			con_printf("err span 80, 40, 16, 8, 4 or 2\n");
			return;
		}
		set_span(i);
		view_changed(old_view, old_span, old_zoom);
		con_printf("ok span %d rate %d\n", span_khz(), fs_khz());
	} else if (strcmp(line, "gain") == 0 && strcmp(arg, "auto") == 0) {
		st.gain = ESP_SDR_RX_GAIN_AUTO;
		st.dirty_radio = true;
		con_printf("ok gain auto\n");
	} else if (strcmp(line, "gain") == 0 && parse_int(arg, &v) && v >= 0 &&
		   v <= esp_sdr_rx_gain_max()) {
		st.gain = (int)v;
		st.dirty_radio = true;
		con_printf("ok gain %d\n", st.gain);
	} else if (strcmp(line, "screen") == 0 && parse_flag(arg, &flag)) {
		st.screen = flag;
		st.help_drawn = false;
		con_printf("ok screen %d\n", st.screen);
	} else if (strcmp(line, "rotate") == 0 &&
		   (strcmp(arg, "auto") == 0 || strcmp(arg, "land") == 0 ||
		    strcmp(arg, "left") == 0 || strcmp(arg, "right") == 0)) {
		/* left and right: that edge of the Cardputer at the bottom. */
		st.auto_rotate = strcmp(arg, "auto") == 0;
		if (!st.auto_rotate) {
			st.orient = arg[0] == 'l' && arg[1] == 'e'   ? ORIENT_LEFT_DOWN
				    : arg[0] == 'r'                 ? ORIENT_RIGHT_DOWN
								    : ORIENT_LANDSCAPE;
		}
		con_printf("ok rotate %s orient %d\n", arg, st.orient);
	} else if (strcmp(line, "peak") == 0 && parse_int(arg, &v) && v >= 1 && v <= PEAK_MAX &&
		   IS_POWER_OF_TWO(v)) {
		st.peak = (int)v; /* bursts per row */
		con_printf("ok peak %d\n", st.peak);
	} else if (strcmp(line, "peak") == 0 && parse_flag(arg, &flag)) {
		st.peak = flag ? PEAK_BURSTS : 1;
		con_printf("ok peak %d\n", st.peak);
	} else if (strcmp(line, "bw") == 0 && parse_int(arg, &v) && v >= 1 && v <= 80) {
		st.bw_mhz = (int)v;
		st.dirty_radio = true;
		con_printf("ok bw %d\n", st.bw_mhz);
	} else if (strcmp(line, "settle") == 0 && parse_int(arg, &v) && v >= 0 && v <= 3000) {
		/* The wait after each retune, 1 ms by default (see main()). */
		esp_sdr_set_retune_settle((uint32_t)v);
		con_printf("ok settle %d\n", (int)v);
	} else if (strcmp(line, "region") == 0 && (strcmp(arg, "eu") == 0 || strcmp(arg, "us") == 0)) {
		st.region_us = arg[0] == 'u';
		st.side_dirty = true;
		con_printf("ok region %s\n", arg);
	} else if (strcmp(line, "flat") == 0 && parse_flag(arg, &flag)) {
		flat_set(flag);
		con_printf("ok flat %d\n", st.flat);
	} else if (strcmp(line, "width") == 0 && strtol(arg, NULL, 10) >= 50 &&
		   strtol(arg, NULL, 10) <= 3000) {
		char *end;
		float db;

		width_khz = (int)strtol(arg, &end, 10);
		db = strtof(end, &end);
		db = db > 0.0f ? db : width_db;
		width_db = CLAMP(db, 1.0f, 20.0f);
		con_printf("ok width %d %d.%d\n", width_khz, (int)width_db,
			   (int)(10.0f * width_db) % 10);
	} else if (strcmp(line, "clip") == 0 && (strcmp(arg, "drop") == 0 || strcmp(arg, "keep") == 0)) {
		clip_drop = arg[0] == 'd';
		con_printf("ok clip %s\n", arg);
	} else if (strcmp(line, "shot") == 0) {
		/* The screen as drawn, at the next redraw (shot_end()). */
		st.shot_req = st.screen;
		con_printf(st.screen ? "ok shot\n" : "err shot screen paused\n");
	} else if (strcmp(line, "hide") == 0 && parse_flag(arg, &flag)) {
		st.width_filter = flag;
		con_printf("ok hide %d\n", st.width_filter);
	} else {
		con_printf("err %s %s (try $help)\n", line, arg);
	}
}

int main(void)
{
	uint32_t last_dump_ms;
	struct display_capabilities caps;
	uint32_t rows = 0, last_ms;
	int ret;

	printk("waterfall: ESP-SDR on M5Stack Cardputer-Adv\n");

	for (unsigned int k = 0; k < NFFT; k++) {
		window[k] = esp_sdr_window(ESP_SDR_WINDOW_HANN, k, NFFT);
		window_gain += (float)NFFT * window[k] * window[k];
	}
	fft_init();
	fft_check();
	build_palette();
	for (unsigned int c = 0; c < W; c++) {
		trace_db[c] = st.floor_db;
	}

	if (!device_is_ready(display)) {
		LOG_ERR("display not ready");
		return 0;
	}
	display_get_capabilities(display, &caps);
	LOG_INF("display %ux%u, format 0x%x", caps.x_resolution, caps.y_resolution,
		caps.current_pixel_format);
	draw_header();
	draw_scale();
	draw_trace();
	draw_waterfall();
	display_blanking_off(display);
	panel_refresh(0);

	kbd_init();
	imu_init();

	ret = esp_sdr_init();
	if (ret != 0) {
		LOG_ERR("esp_sdr_init: %d", ret);
		return 0;
	}
	/*
	 * Against the library's 3 ms: captures right after a 40 MHz hop matched settled ones
	 * (passband within 0.15 dB, clock harmonics 0.5 dB) with no wait at all after a reset, but
	 * after some power-ons the first 500 us or so make spurs up to +32 dB, smeared over several bins
	 * (1802.8 MHz in the 80 MHz sweep on 1840, 2240.3 in the 160 on 2300); gone from 1 ms.
	 */
	esp_sdr_set_retune_settle(1000U);
	printk("waterfall: radio ready, gain table 0..%d\n", esp_sdr_rx_gain_max());

	last_ms = k_uptime_get_32();
	last_dump_ms = last_ms;
	uint32_t last_draw_ms = last_ms;
	uint32_t loop_ms = last_ms;

	for (;;) {
		static char cmd[64];
		static int cmd_len = -1; /* -1: not in a $ command */
		bool input = false;
		char k;
		unsigned char ck;

		while ((k = kbd_poll()) != 0) {
			handle_key(rotate_key(k));
			input = true;
		}
		while (uart_poll_in(console, &ck) == 0) {
			input = true;
			if (cmd_len < 0) {
				if (ck == '$') {
					cmd_len = 0;
				} else {
					handle_key((char)ck);
				}
			} else if (ck == '\r' || ck == '\n') {
				cmd[cmd_len] = '\0';
				cmd_len = -1;
				handle_cmd(cmd);
			} else if (cmd_len < (int)sizeof(cmd) - 1) {
				cmd[cmd_len++] = (char)ck;
			}
		}
		if (st.dirty_radio) {
			apply_radio();
		}
		/* Zoomed in, zoom captures per redraw: time resolution grows with frequency's. */
		ret = 0;
		for (int n = 0; n < st.zoom && ret == 0; n++) {
			ret = sweeping() ? capture_sweep() : capture_row();
			rows++;
			if (ret == 0 && st.orient != ORIENT_LANDSCAPE && st.screen && !st.help && scroll_on &&
			    !st.side_dirty) {
				side_push();
			}
		}
		if (ret != 0) {
			LOG_WRN("capture: %d", ret);
			k_msleep(100);
			continue;
		}
		uint32_t now = k_uptime_get_32();

		loop_max_ms = MAX(loop_max_ms, now - loop_ms);
		loop_ms = now;
		bool draw = input || st.orient == ORIENT_LANDSCAPE || now - last_draw_ms >= FRAME_MS;

		if (st.shot_req && st.screen) {
			/* Everything redrawn, each write also to the host, then the layout. */
			st.shot_req = false;
			st.shot = true;
			if (st.help) {
				scroll_stop();
				draw_help();
			} else if (st.orient != ORIENT_LANDSCAPE) {
				side_full();
			} else {
				scroll_stop();
				draw_header();
				draw_scale();
				draw_trace();
				draw_waterfall();
			}
			st.shot = false;
			shot_end();
			draw = false;
		}
		if (draw) {
			last_draw_ms = now;
		}
		if (!draw) {
			/* Drawn with the next frame. */
		} else if (!st.screen) {
			/* Paused for the host (the history still fills): say so once. */
			if (!st.help_drawn) {
				scroll_stop();
				draw_header();
				st.help_drawn = true;
			}
			st.side_dirty = true;
		} else if (st.help) {
			if (!st.help_drawn) {
				scroll_stop();
				draw_help();
			}
			st.side_dirty = true;
		} else if (st.orient != ORIENT_LANDSCAPE) {
			/* The columns went out as they came; the trace follows at the frame rate. */
			if (st.side_dirty || !scroll_on) {
				side_full();
			} else {
				side_header();
				side_trace();
			}
		} else {
			scroll_stop();
			draw_header();
			draw_scale();
			draw_trace();
			draw_waterfall();
		}

		static uint32_t last_imu_ms;

		if (now - last_imu_ms >= 200U) {
			imu_poll();
			orient_update();
			last_imu_ms = now;
		}
		if (now - last_dump_ms >= 2000U && st.text) {
			dump_spectrum();
			last_dump_ms = now;
		}

		if (now - last_ms >= 5000U && st.text) {
			con_printf("waterfall: LO %d MHz, view %d kHz x%d, %u rows/s, floor %d dB, "
			       "gain %d\n",
			       st.lo_mhz, st.view_khz, st.zoom,
			       (unsigned int)(rows * 1000U / (now - last_ms)), (int)st.floor_db,
			       st.gain);
			rows = 0;
			last_ms = now;
		}
		/* Let lower priority threads (the Wi-Fi driver, the logger) run. */
		k_msleep(1);
	}
	return 0;
}
