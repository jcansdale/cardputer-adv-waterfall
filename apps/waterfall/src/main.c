/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Spectrum and waterfall on the M5Stack Cardputer-Adv's 240x135 display, receive only.
 *
 * Each row is a Welch average of Hann-windowed 256-point FFTs over one 4096-sample burst
 * (16 or 40 MS/s), the middle 240 bins of 256 shown. No PSRAM: the history is one byte per
 * pixel (palette index), converted to RGB565 a few rows at a time on the way to the display.
 *
 * Keys (Cardputer arrows are ; . , /):
 *   , /   frequency -1 / +1 MHz      ; .   frequency +10 / -10 MHz
 *   h     back to 2440 MHz           r     rate 16 / 40 MS/s
 *   - =   colour floor -2 / +2 dB    [ ]   colour range -5 / +5 dB
 *   a     automatic floor on/off     n m   gain -1 / +1 (fixed)   g  gain auto
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
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <esp_sdr/esp_sdr.h>
#include <esp_sdr/esp_sdr_dsp.h>

LOG_MODULE_REGISTER(waterfall, LOG_LEVEL_INF);

#define W          240
#define H          135
#define HDR_H      10
#define PSD_Y      HDR_H
#define PSD_H      30
#define WF_Y       (PSD_Y + PSD_H)
#define WF_H       (H - WF_Y)
#define CHUNK_ROWS 8

#define NFFT      256U
#define SAMPLES   4096U
#define BIN0      ((NFFT - W) / 2U)
#define FULLSCALE 512.0f
#define FLOOR_DB  (-150.0f)

#define FREQ_MIN_MHZ     1800
#define FREQ_MAX_MHZ     2800
#define FREQ_DEFAULT_MHZ 2440

/* The panel takes RGB565 most significant byte first. */
#define RGB565(r, g, b)                                                                            \
	sys_cpu_to_be16((uint16_t)((((r) & 0xf8) << 8) | (((g) & 0xfc) << 3) | ((b) >> 3)))

static const struct device *const display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));

static float re[NFFT], im[NFFT], window[NFFT], psd[NFFT];
static float tw_cos[NFFT / 2], tw_sin[NFFT / 2];
static float window_gain;

static float row_db[W], trace_db[W];
static uint8_t hist[WF_H][W];
static unsigned int head;
static uint16_t palette[256];
static uint16_t fb[W * CHUNK_ROWS];

static struct {
	int freq_mhz;
	enum esp_sdr_rate rate;
	int gain; /* ESP_SDR_RX_GAIN_AUTO or a table index */
	bool auto_floor;
	float floor_db, range_db;
	bool tune_warn;
	bool dirty_radio;
} st = {
	.freq_mhz = FREQ_DEFAULT_MHZ,
	.rate = ESP_SDR_RATE_16MSPS,
	.gain = ESP_SDR_RX_GAIN_AUTO,
	.auto_floor = true,
	.floor_db = -100.0f,
	.range_db = 40.0f,
	.dirty_radio = true,
};

/* 5x7 font, ASCII 0x20 to 0x5a, columns LSB on top. */
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
	{0x61, 0x51, 0x49, 0x45, 0x43},
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

		if (ch < 0x20 || ch > 0x5a) {
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
	snprintf(line, sizeof(line), "%4d MHZ%s %2uM G:%s %c%d/%d", st.freq_mhz,
		 st.tune_warn ? "!" : " ", (unsigned int)(esp_sdr_rx_rate_hz(st.rate) / 1000000U),
		 gain,
		 st.auto_floor ? 'A' : 'L', (int)st.floor_db, (int)st.range_db);
	for (unsigned int k = 0; k < W * HDR_H; k++) {
		fb[k] = bg;
	}
	draw_text(fb, 2, 1, line, fg);
	/* Centre marker. */
	fb[(HDR_H - 1) * W + W / 2] = RGB565(255, 0, 0);
	blit(0, HDR_H);
}

static unsigned int level(float db)
{
	float x = (db - st.floor_db) / st.range_db;

	return (unsigned int)CLAMP(x * 255.0f, 0.0f, 255.0f);
}

static void draw_trace(void)
{
	uint16_t bg = RGB565(0, 0, 0), grid = RGB565(40, 40, 40);

	for (unsigned int y0 = 0; y0 < PSD_H; y0 += CHUNK_ROWS) {
		unsigned int rows = MIN(CHUNK_ROWS, PSD_H - y0);

		for (unsigned int x = 0; x < W; x++) {
			unsigned int lv = level(trace_db[x]);
			unsigned int h = lv * PSD_H / 256U;

			for (unsigned int r = 0; r < rows; r++) {
				unsigned int y = y0 + r; /* 0 at the top */
				unsigned int from_bottom = PSD_H - 1U - y;
				uint16_t c = (x == W / 2 || y == PSD_H / 2) ? grid : bg;

				if (from_bottom < h) {
					c = from_bottom + 1U == h ? RGB565(255, 255, 255)
								  : palette[lv / 2U];
				}
				fb[r * W + x] = c;
			}
		}
		blit(PSD_Y + y0, rows);
	}
}

static void draw_waterfall(void)
{
	for (unsigned int y0 = 0; y0 < WF_H; y0 += CHUNK_ROWS) {
		unsigned int rows = MIN(CHUNK_ROWS, WF_H - y0);

		for (unsigned int r = 0; r < rows; r++) {
			const uint8_t *src = hist[(head + y0 + r) % WF_H];

			for (unsigned int x = 0; x < W; x++) {
				fb[r * W + x] = palette[src[x]];
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

	switch (k) {
	case ',':
		st.freq_mhz -= 1;
		break;
	case '/':
		st.freq_mhz += 1;
		break;
	case ';':
		st.freq_mhz += 10;
		break;
	case '.':
		st.freq_mhz -= 10;
		break;
	case 'h':
		st.freq_mhz = FREQ_DEFAULT_MHZ;
		break;
	case 'r':
		st.rate = st.rate == ESP_SDR_RATE_16MSPS ? ESP_SDR_RATE_40MSPS
							 : ESP_SDR_RATE_16MSPS;
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
	st.freq_mhz = CLAMP(st.freq_mhz, FREQ_MIN_MHZ, FREQ_MAX_MHZ);
	st.dirty_radio = true;
}

static void apply_radio(void)
{
	int ret = esp_sdr_set_freq(st.freq_mhz);

	st.tune_warn = ret == -ERANGE;
	if (ret != 0 && ret != -ERANGE) {
		LOG_WRN("set_freq %d: %d", st.freq_mhz, ret);
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

static int capture_row(void)
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

	float norm = 1.0f / (FULLSCALE * FULLSCALE * window_gain * (float)segments);
	float sum = 0.0f;

	for (unsigned int c = 0; c < W; c++) {
		unsigned int m = BIN0 + c;               /* shifted: m = NFFT/2 is DC */
		unsigned int k = (m + NFFT / 2U) % NFFT; /* natural order */
		float p = psd[k];

		/* The LO leakage and DC residue: the neighbours' mean instead. */
		if (m == NFFT / 2U) {
			p = 0.5f * (psd[1] + psd[NFFT - 1U]);
		}
		p *= norm;
		row_db[c] = p > 0.0f ? 10.0f * log10f(p) : FLOOR_DB;
		sum += row_db[c];
	}

	if (st.auto_floor) {
		/* Track the mean level; the floor sits a little below it. */
		st.floor_db += 0.05f * (sum / (float)W - 5.0f - st.floor_db);
	}

	head = (head + WF_H - 1U) % WF_H;
	for (unsigned int c = 0; c < W; c++) {
		hist[head][c] = (uint8_t)level(row_db[c]);
		trace_db[c] += 0.3f * (row_db[c] - trace_db[c]);
	}
	return 0;
}

int main(void)
{
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
	for (;;) {
		char k;

		while ((k = kbd_poll()) != 0) {
			handle_key(k);
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
		draw_header();
		draw_trace();
		draw_waterfall();
		rows++;

		uint32_t now = k_uptime_get_32();

		if (now - last_ms >= 5000U) {
			printk("waterfall: %d MHz, %u rows/s, floor %d dB, gain %d\n", st.freq_mhz,
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
