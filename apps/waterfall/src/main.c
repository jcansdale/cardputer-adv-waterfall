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
 *   , /   scroll left / right        ; .   zoom in / out (x1 to x8)
 *   j k   frequency -10 / +10 MHz    h     back to 2440 MHz, x1
 *   r     rate 16 / 40 / 80 MS/s     w     Wi-Fi channel overlay on/off
 *   p     peak hold over PEAK_BURSTS bursts on/off
 *   - =   colour floor -2 / +2 dB    [ ]   colour range -5 / +5 dB
 *   a     automatic floor on/off     n m   gain -1 / +1 (fixed)   g  gain auto
 *   x     hide the board's own clock harmonics on/off
 */

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <soc/rtc_cntl_reg.h>

#include <esp_sdr/esp_sdr.h>
#include <esp_sdr/esp_sdr_dsp.h>

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

#define NFFT      1024U
#define SAMPLES   4096U
#define ZOOM_MAX  8
/* Wi-Fi is bursty: with peak hold each row is the per-bin maximum over this many bursts. */
#define PEAK_BURSTS 8
#define FULLSCALE 512.0f
#define FLOOR_DB  (-150.0f)

#define FREQ_MIN_MHZ     1800
#define FREQ_MAX_MHZ     2800
#define FREQ_DEFAULT_MHZ 2440

/* The panel takes RGB565 most significant byte first. */
#define RGB565(r, g, b)                                                                            \
	sys_cpu_to_be16((uint16_t)((((r) & 0xf8) << 8) | (((g) & 0xfc) << 3) | ((b) >> 3)))

static const struct device *const display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

static float re[NFFT], im[NFFT], window[NFFT], psd[NFFT], peak[NFFT];
static float tw_cos[NFFT / 2], tw_sin[NFFT / 2];
static float window_gain;

static float row_db[W], trace_db[W];
/* The per-pixel maximum since the last console dump. */
static float dump_db[W];
static bool dump_fresh = true;
static uint8_t hist[WF_H][W];
static unsigned int head;
static uint16_t palette[256];
static uint16_t fb[W * CHUNK_ROWS];

static struct {
	int lo_mhz;   /* the radio's LO */
	int view_khz; /* the centre of the screen */
	int zoom;     /* the screen spans the sample rate / zoom */
	bool trace_reset;
	bool channels; /* the 2.4 GHz Wi-Fi channel overlay */
	bool peak_hold;
	bool hide_spurs; /* patch over the board's clock harmonics */
	enum esp_sdr_rate rate;
	int gain; /* ESP_SDR_RX_GAIN_AUTO or a table index */
	bool help;       /* the key help covers the screen */
	bool help_drawn;
	bool auto_floor;
	float floor_db, range_db;
	bool tune_warn;
	bool dirty_radio;
} st = {
	.lo_mhz = FREQ_DEFAULT_MHZ,
	.view_khz = FREQ_DEFAULT_MHZ * 1000,
	.zoom = 1,
	.peak_hold = true,
	.rate = ESP_SDR_RATE_16MSPS,
	/* Fixed mid gain: the AGC re-levels each burst and flattens bursty Wi-Fi into the noise. */
	.gain = 42,
	.auto_floor = true,
	.floor_db = -100.0f,
	.range_db = 40.0f,
	.dirty_radio = true,
};

/*
 * The board hears its own clocks: steady carriers exactly on multiples of the 40 MHz crystal
 * and of the 48 MHz USB clock (measured at 2352, 2360, 2400, 2440, 2448 and 2496 MHz).
 */
static bool is_spur_khz(int khz)
{
	return khz % 40000 == 0 || khz % 48000 == 0;
}

/* Calls fn(khz) for each harmonic from a to b kHz. */
#define FOR_EACH_SPUR(a, b, khz)                                                              \
	for (int khz = ((a) + 7999) / 8000 * 8000; khz <= (b); khz += 8000)                    \
		if (is_spur_khz(khz))

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

static void draw_text(uint16_t *buf, unsigned int x, unsigned int y, const char *s,
		      uint16_t color)
{
	for (; *s != '\0' && x + 5 <= W; s++, x += 6) {
		int ch = toupper((unsigned char)*s);

		if (ch < 0x20 || ch > 0x5d) {
			ch = '?';
		}
		for (unsigned int col = 0; col < 5; col++) {
			uint8_t bits = font5x7[ch - 0x20][col];

			for (unsigned int row = 0; row < 7; row++) {
				if (bits & BIT(row)) {
					buf[(y + row) * W + x + col] = color;
				}
			}
		}
	}
}

static int fs_khz(void)
{
	return (int)(esp_sdr_rx_rate_hz(st.rate) / 1000U);
}

static int span_khz(void)
{
	return fs_khz() / st.zoom;
}

static uint8_t *hist_row(unsigned int age)
{
	return hist[(head + age) % WF_H];
}

/*
 * Re-lay the history for a new view so old rows stay where their frequencies are, and
 * stretch it in time with the zoom (anchored at the newest row), as rows are drawn zoom
 * pixels tall. In place: zooming in reads rows at or above the one written, so go down
 * from the bottom; zooming out reads rows at or below, so go up from the top.
 */
static void hist_remap(int old_view_khz, int old_span_khz, int old_zoom)
{
	static uint8_t tmp[W];
	int new_span = span_khz();
	int d = (st.view_khz - new_span / 2) - (old_view_khz - old_span_khz / 2);

	if (st.zoom > old_zoom) {
		for (int r = WF_H - 1; r > 0; r--) {
			memcpy(hist_row(r), hist_row(r * old_zoom / st.zoom), W);
		}
	} else if (st.zoom < old_zoom) {
		for (unsigned int r = 1; r < WF_H; r++) {
			unsigned int src = r * old_zoom / st.zoom;

			if (src < WF_H) {
				memcpy(hist_row(r), hist_row(src), W);
			} else {
				memset(hist_row(r), 0, W);
			}
		}
	}

	for (unsigned int r = 0; r < WF_H; r++) {
		for (unsigned int c = 0; c < W; c++) {
			float x = ((float)d + ((float)c + 0.5f) * (float)new_span / (float)W) *
				  (float)W / (float)old_span_khz;

			tmp[c] = (x >= 0.0f && x < (float)W) ? hist[r][(unsigned int)x] : 0U;
		}
		memcpy(hist[r], tmp, W);
	}
	st.trace_reset = true;
}

/* Retune when the view leaves the span the radio captures. */
static void follow_view(void)
{
	int half = fs_khz() / 2, span = span_khz(), lo = st.lo_mhz * 1000;

	if (st.view_khz - span / 2 < lo - half || st.view_khz + span / 2 > lo + half) {
		st.lo_mhz = CLAMP((st.view_khz + 500) / 1000, FREQ_MIN_MHZ, FREQ_MAX_MHZ);
		st.dirty_radio = true;
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
	snprintf(line, sizeof(line), "%4d.%03d%s %2uM%c%c X%d G:%s %c%d/%d", st.view_khz / 1000,
		 st.view_khz % 1000, st.tune_warn ? "!" : " ", (unsigned int)(fs_khz() / 1000),
		 st.peak_hold ? 'P' : ' ', st.hide_spurs ? 'X' : ' ', st.zoom, gain, st.auto_floor ? 'A' : 'L', (int)st.floor_db, (int)st.range_db);
	for (unsigned int k = 0; k < W * HDR_H; k++) {
		fb[k] = bg;
	}
	draw_text(fb, 2, 1, line, fg);
	blit(0, HDR_H);
}

/*
 * The frequency scale: ticks at least 24 pixels apart (taller on whole MHz) and, over the
 * ticks of a coarser step that leaves room for the text, their frequency in MHz.
 */
static void draw_scale(void)
{
	static const int steps[] = {10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000};
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
	/* The board's own clock harmonics (magenta). */
	FOR_EACH_SPUR(left, left + span - 1, khz) {
		unsigned int x = (unsigned int)((int64_t)(khz - left) * W / span);

		for (unsigned int y = SCALE_H - 3U; y < SCALE_H && x < W; y++) {
			fb[y * W + x] = RGB565(220, 0, 220);
		}
	}
	/* The view centre (white) and the tuning centre, the LO artefact (red). */
	fb[(SCALE_H - 1) * W + W / 2] = RGB565(255, 255, 255);
	int lo_x = (int)(((int64_t)st.lo_mhz * 1000 - left) * W / span);

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
/* The 40 MHz PHY mode's analog filters pass about +-18 MHz (doc/hardware-quirks.md). */
#define PASS_KHZ 18000
#define DIM(c)   (((c) >> 1) & 0x7befU) /* RGB565 at half brightness */
static int pass_a, pass_b; /* pixels pass_a to pass_b - 1 are inside the analog filters */
#define BLE_COLOR RGB565(0, 220, 255)
static uint8_t ch_at[W];
static uint8_t ble_at[W];
static bool ch_band[W];

static const struct {
	uint8_t ch;
	int khz;
} ble_adv[] = {{37, 2402000}, {38, 2426000}, {39, 2480000}};

static int chan_khz(int ch)
{
	return ch == 14 ? 2484000 : 2407000 + 5000 * ch;
}

static void build_overlay(void)
{
	int span = span_khz(), left = st.view_khz - span / 2;

	pass_a = 0;
	pass_b = W;
	if (st.rate != ESP_SDR_RATE_16MSPS) {
		int lo = st.lo_mhz * 1000;

		pass_a = CLAMP((int)(((int64_t)lo - PASS_KHZ - left) * W / span), 0, (int)W);
		pass_b = CLAMP((int)(((int64_t)lo + PASS_KHZ - left) * W / span) + 1, 0, (int)W);
	}
	memset(ch_at, 0, sizeof(ch_at));
	memset(ble_at, 0, sizeof(ble_at));
	memset(ch_band, 0, sizeof(ch_band));
	for (size_t i = 0; i < ARRAY_SIZE(ble_adv); i++) {
		int x = (int)(((int64_t)ble_adv[i].khz - left) * W / span);

		if (x >= 0 && x < (int)W) {
			ble_at[x] = ble_adv[i].ch;
		}
	}
	for (int ch = 1; ch <= 14; ch++) {
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
	uint16_t bg = RGB565(0, 0, 0), grid = RGB565(40, 40, 40);
	uint16_t band = RGB565(0, 24, 48), mark = RGB565(110, 110, 110);

	build_overlay();
	for (unsigned int y0 = 0; y0 < PSD_H; y0 += CHUNK_ROWS) {
		unsigned int rows = MIN(CHUNK_ROWS, PSD_H - y0);

		for (unsigned int x = 0; x < W; x++) {
			unsigned int lv = level(trace_db[x]);
			unsigned int h = lv * PSD_H / 256U;

			for (unsigned int r = 0; r < rows; r++) {
				unsigned int y = y0 + r; /* 0 at the top */
				unsigned int from_bottom = PSD_H - 1U - y;
				uint16_t c = (x == W / 2 || y == PSD_H / 2) ? grid
					     : ch_band[x]                     ? band
									      : bg;

				if (ch_at[x] != 0 &&
				    ((y >= LABEL_H - 2U && y < LABEL_H) ||
				     (st.channels && y >= 2U * LABEL_H && (y & 1U) == 0))) {
					c = mark;
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
			int next = 0; /* the first free pixel for a label */

			for (unsigned int x = 0; x < W; x++) {
				char num[4];
				int lx;

				if (ch_at[x] == 0) {
					continue;
				}
				snprintf(num, sizeof(num), "%u", ch_at[x]);
				lx = (int)x - (6 * (int)strlen(num) - 1) / 2; /* centred on the channel */
				if (lx < next || lx < 0 || lx + 6 * (int)strlen(num) > (int)W) {
					continue;
				}
				draw_text(fb, (unsigned int)lx, 1, num, RGB565(255, 220, 0));
				next = lx + 6 * (int)strlen(num) + 1;
			}
		}
		if (y0 == LABEL_H) {
			for (unsigned int x = 0; x < W; x++) {
				char name[8];
				int lx = (int)x - 8; /* "B37" is 17 pixels, centred */

				if (ble_at[x] == 0) {
					continue;
				}
				snprintf(name, sizeof(name), "B%u", ble_at[x]);
				lx = CLAMP(lx, 0, (int)W - 17);
				draw_text(fb, (unsigned int)lx, 1, name, BLE_COLOR);
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
		";  .   ZOOM IN/OUT  (UP/DOWN)",
		"J  K   -10/+10 MHZ    H  HOME",
		"R      RATE 16/40/80 MS/S",
		"N  M   GAIN -/+       G  AGC",
		"-  =   COLOUR FLOOR   A  AUTO",
		"[  ]   COLOUR RANGE",
		"P      PEAK HOLD      W  CH. LINES",
		"X      HIDE OWN CLOCK HARMONICS",
		"RED: TUNING CENTRE  MAGENTA: CLOCKS",
		"YELLOW 1-14: WI-FI CHANNELS",
		"CYAN B37-39: BLUETOOTH LE ADVERTISING",
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
			const uint8_t *src = hist[(head + y0 + r) % WF_H];

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
		st.zoom = MIN(st.zoom * 2, ZOOM_MAX);
		break;
	case '.':
		st.zoom = MAX(st.zoom / 2, 1);
		break;
	case 'j':
		st.view_khz -= 10000;
		break;
	case 'k':
		st.view_khz += 10000;
		break;
	case 'h':
		st.view_khz = FREQ_DEFAULT_MHZ * 1000;
		st.zoom = 1;
		break;
	case 'r':
		st.rate = st.rate == ESP_SDR_RATE_16MSPS   ? ESP_SDR_RATE_40MSPS
			  : st.rate == ESP_SDR_RATE_40MSPS ? ESP_SDR_RATE_80MSPS
							   : ESP_SDR_RATE_16MSPS;
		st.dirty_radio = true; /* the analog filters follow the rate */
		break;
	case 'p':
		st.peak_hold = !st.peak_hold;
		return;
	case '-':
		st.auto_floor = false;
		st.floor_db -= 2.0f;
		return;
	case '=':
		st.auto_floor = false;
		st.floor_db += 2.0f;
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
		st.hide_spurs = !st.hide_spurs;
		break;
	case 'w':
		st.channels = !st.channels;
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
	st.view_khz = CLAMP(st.view_khz, FREQ_MIN_MHZ * 1000, FREQ_MAX_MHZ * 1000);
	follow_view();
	if (st.view_khz != old_view || span_khz() != old_span) {
		hist_remap(old_view, old_span, old_zoom);
	}
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
	ret = esp_sdr_rx_set_bandwidth(wide ? 0U : CLAMP(16U, bw_min, bw_max));
	if (ret != 0) {
		LOG_WRN("rx bandwidth: %d", ret);
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

/* ---- spectrum ---- */

/* One burst's Welch average into psd; @return the segment count, or a negative error. */
static int capture_psd(void)
{
	struct esp_sdr_rx_burst b;
	unsigned int segments;
	float dc_i = 0.0f, dc_q = 0.0f;
	int ret = esp_sdr_rx_capture(st.rate, SAMPLES, &b);

	if (ret != 0) {
		return ret;
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

static int capture_row(void)
{
	unsigned int bursts = st.peak_hold ? PEAK_BURSTS : 1U;
	int segments = 0;

	for (unsigned int n = 0; n < bursts; n++) {
		int ret = capture_psd();

		if (ret < 0) {
			return ret;
		}
		segments = ret;
		if (n == 0U) {
			memcpy(peak, psd, sizeof(peak));
		} else {
			for (unsigned int k = 0; k < NFFT; k++) {
				peak[k] = MAX(peak[k], psd[k]);
			}
		}
	}
	memcpy(psd, peak, sizeof(psd));

	float norm = 1.0f / (FULLSCALE * FULLSCALE * window_gain * (float)segments);
	float sum = 0.0f;
	unsigned int valid = 0;

	/* The LO leakage and DC residue: the neighbours' mean instead. */
	psd[0] = psd[1] = psd[NFFT - 1U] = 0.5f * (psd[2] + psd[NFFT - 2U]);

	/* Likewise the clock harmonics, over the Hann window's main lobe. */
	if (st.hide_spurs) {
		int lo = st.lo_mhz * 1000, half = fs_khz() / 2;

		FOR_EACH_SPUR(lo - half, lo + half, khz) {
			int kc = (int)lroundf((float)(khz - lo) * (float)NFFT / (float)fs_khz());

			if (kc >= -2 && kc <= 2) {
				continue; /* the LO patch has it */
			}
			float fill = 0.5f * (psd[(unsigned int)(kc - 3 + (int)NFFT) % NFFT] +
					     psd[(unsigned int)(kc + 3 + (int)NFFT) % NFFT]);

			for (int k = kc - 2; k <= kc + 2; k++) {
				psd[(unsigned int)(k + (int)NFFT) % NFFT] = fill;
			}
		}
	}

	/* Pixel edges in shifted bins (NFFT/2 is the LO). */
	float bin_khz = (float)fs_khz() / (float)NFFT;
	float px_bins = (float)span_khz() / (float)W / bin_khz;
	float left = (float)(st.view_khz - span_khz() / 2 - st.lo_mhz * 1000) / bin_khz +
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
		sum += row_db[c];
		valid++;
	}

	if (st.auto_floor && valid > 0) {
		/* Track the mean level; the floor sits a little below it. */
		st.floor_db += 0.05f * (sum / (float)valid - 5.0f - st.floor_db);
	}

	head = (head + WF_H - 1U) % WF_H;
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
	/* Zoomed in, each row is zoom pixels tall: time stretches with frequency. */
	for (int n = 1; n < st.zoom; n++) {
		head = (head + WF_H - 1U) % WF_H;
		memcpy(hist[head], hist_row(1), W);
	}
	return 0;
}

/*
 * One console line: the view, then the 240 pixels' maximum in dB since the last line, for
 * looking at the spectrum from a host (keys can be typed on the console too).
 */
static void dump_spectrum(const struct device *console)
{
	static char line[64 + W * 5];
	int len = snprintf(line, sizeof(line),
			   "spec lo %d view %d span %d rate %u zoom %d peak %d floor %d:", st.lo_mhz,
			   st.view_khz, span_khz(), (unsigned int)fs_khz(), st.zoom, st.peak_hold,
			   (int)st.floor_db);

	for (unsigned int c = 0; c < W && len < (int)sizeof(line) - 8; c++) {
		len += snprintf(&line[len], sizeof(line) - (size_t)len, " %d",
				(int)lroundf(MAX(dump_db[c], -999.0f)));
	}
	line[len++] = '\n';
	/* Straight to the UART: one log message per number would overflow the log buffer. */
	for (int i = 0; i < len; i++) {
		uart_poll_out(console, (unsigned char)line[i]);
	}
	dump_fresh = true;
}

int main(void)
{
	const struct device *const console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t last_dump_ms;
	struct display_capabilities caps;
	uint32_t rows = 0, last_ms;
	int ret;

	printk("waterfall: ESP-SDR on M5Stack Cardputer-Adv\n");

	for (unsigned int k = 0; k < NFFT; k++) {
		window[k] = esp_sdr_window(ESP_SDR_WINDOW_HANN, k, NFFT);
		window_gain += (float)NFFT * window[k] * window[k];
	}
	esp_sdr_fft_twiddles(tw_cos, tw_sin, NFFT);
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

	kbd_init();

	ret = esp_sdr_init();
	if (ret != 0) {
		LOG_ERR("esp_sdr_init: %d", ret);
		return 0;
	}
	printk("waterfall: radio ready, gain table 0..%d\n", esp_sdr_rx_gain_max());

	last_ms = k_uptime_get_32();
	last_dump_ms = last_ms;
	for (;;) {
		char k;
		unsigned char ck;

		while ((k = kbd_poll()) != 0) {
			handle_key(k);
		}
		while (uart_poll_in(console, &ck) == 0) {
			handle_key((char)ck);
		}
		if (st.dirty_radio) {
			apply_radio();
		}
		ret = capture_row();
		if (ret != 0) {
			LOG_WRN("capture: %d", ret);
			k_msleep(100);
			continue;
		}
		if (!st.help) {
			draw_header();
			draw_scale();
			draw_trace();
			draw_waterfall();
		} else if (!st.help_drawn) {
			draw_help();
		}
		rows++;

		uint32_t now = k_uptime_get_32();

		if (now - last_dump_ms >= 2000U) {
			dump_spectrum(console);
			last_dump_ms = now;
		}

		if (now - last_ms >= 5000U) {
			printk("waterfall: LO %d MHz, view %d kHz x%d, %u rows/s, floor %d dB, "
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
