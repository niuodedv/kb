/*
 * LCD 屏幕 UI 主模块（ST7789V，320x172，SPI 只写）
 *
 * 运行模型：lcd 专用线程，周期巡检 + 仅在状态变化时局部重绘对应控件
 *（不建整帧影子缓冲；按“控件=若干行”行条带 + display_write 落屏）。
 *
 * 布局见 lcd 显示.txt【4.2】：
 *   第一行：左=三模图标（当前白底红字，其余黑底白字描白边）
 *           右=电量色条 + 电量百分比（色条靠最右，百分比数字在条左侧右对齐）
 *   第二行：居中大字时间 yyyy-MM-dd HH:mm:ss（kb_time 软件时钟）
 *   第三行：亮度滑动条（白底，左端按亮度填绿，从左到右递增）
 * 交互：方向（导航）模式（NumLock 关）时旋钮调亮度（ble 音量此时让出）；
 *       数字模式（NumLock 开）时旋钮维持 BLE 音量。
 *
 * 颜色输出：每个 RGB565 像素占 2 字节，缓冲里【低字节在前】发出
 *（实测本屏 ST7789V 以“第 2 字节为高字节”解析像素；此前高字节在前
 *  导致绿色 0x07E0 被显示成红色）。display 驱动/mipi-dbi 对像素数据直通，
 * 不在此处做任何字节交换。
 */

#include "lcd.h"

#include "backlight.h"
#include "lcd_font.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

#include "hid/hid_keymap.h"
#include "knob/knob.h"
#include "mode/mode.h"
#include "power/bat_adc.h"
#include "power/battery.h"
#include "time/kb_time.h"

LOG_MODULE_REGISTER(kb_lcd, LOG_LEVEL_INF);

/* ---------------- 屏参 / 布局常量（lcd 显示.txt 4.2） ---------------- */
#define LCD_W 320
#define LCD_H 172

/* 行 1：三模图标 */
#define MODE_Y0         4
#define MODE_BLOCK_W    54
#define MODE_BLOCK_H    24
#define MODE_GAP        8
#define MODE_X0         8
#define MODE_TXT_SCALE  2u
#define MODE_TXT_GAP    2u   /* 字间距(px)，缩放后字符宽 = 5*scale */

/* 行 1：电量数字 + 电量条（整体靠右；条在最右，数字在条左侧右对齐） */
#define GAUGE_W         44
#define GAUGE_H         16
#define GAUGE_X         (LCD_W - 8 - GAUGE_W)      /* 右缘距屏右 8px */
#define GAUGE_Y         (MODE_Y0 + 4)              /* 8 */

/* 电量百分比数字：靠右排在色条左侧（色条带黑框后更醒目） */
#define BAT_PCT_GAP       4u   /* 数字与色条间距 */
#define BAT_PCT_SCALE     1u   /* 7px 高，与色条垂直居中对齐 */
#define BAT_PCT_CHAR_GAP  1u   /* 数字间字间距 */

/* 充电指示 “CHG”：固定槽位，位于百分比数字左侧。
 * 百分比数字右对齐到色条，最宽为 “100%”；CHG 槽按该最宽位置再左移一格预留，
 * 保证任意位数下都不与数字重叠，且槽位固定便于放电时整槽清黑。 */
#define CHG_TXT_GAP      4u    /* CHG 与（最宽）百分比数字的间距 */
#define CHG_PCT_MAX_W    (4 * ((int)KB_FONT_W * BAT_PCT_SCALE + BAT_PCT_CHAR_GAP) \
			  - BAT_PCT_CHAR_GAP)                                   /* “100%” 宽 */
#define CHG_TXT_W        (3 * ((int)KB_FONT_W * BAT_PCT_SCALE + BAT_PCT_CHAR_GAP) \
			  - BAT_PCT_CHAR_GAP)                                   /* “CHG” 宽 */
#define CHG_TXT_X        (GAUGE_X - (int)BAT_PCT_GAP - CHG_PCT_MAX_W \
			  - (int)CHG_TXT_GAP - CHG_TXT_W)

/* 行 2：时间（19 字符 x 16px = 304，字形 5x7 放大 3 倍，高 21） */
#define TIME_SCALE      3u
#define TIME_GAP        1u
#define TIME_CHAR_W     (KB_FONT_W * TIME_SCALE + TIME_GAP)   /* 16 */
#define TIME_W          (19 * TIME_CHAR_W)                     /* 304 */
#define TIME_X          ((LCD_W - TIME_W) / 2)                 /* 8 */
#define TIME_Y          75
#define TIME_H          (KB_FONT_H * TIME_SCALE)               /* 21 */

/* 行 3：亮度滑动条 */
#define SLIDER_W        288
#define SLIDER_H        14
#define SLIDER_X        ((LCD_W - SLIDER_W) / 2)               /* 16 */
#define SLIDER_Y        140

#define LCD_POLL_MS     200
#define BAT_POLL_MS     1000

/* ---------------- 颜色（RGB565；缓冲内低字节在前，见文件头说明） ---------------- */
struct kb_rgb {
	uint8_t hi;
	uint8_t lo;
};

static const struct kb_rgb CLR_BLACK = { 0x00, 0x00 };
static const struct kb_rgb CLR_WHITE = { 0xff, 0xff };
static const struct kb_rgb CLR_RED   = { 0xf8, 0x00 };
static const struct kb_rgb CLR_GREEN = { 0x07, 0xe0 };
static const struct kb_rgb CLR_YELLOW = { 0xff, 0xe0 };

/* ---------------- 内部状态 ---------------- */
static const struct device *lcd_dev;
static uint8_t row_buf[LCD_W * 2];   /* 单行条带缓冲（行内逐像素打包） */

/* 上次绘制快照：用于“变了才重绘” */
static enum kb_mode last_mode;
static char last_time[32];
static uint8_t last_bright;          /* 0xFF 表示尚未绘制过 */
static uint8_t last_bat_pct;         /* 0xFF 表示尚未绘制过 */
static bool last_charging;           /* 上次绘制的充电状态 */

static int64_t last_bat_check_ms;

/* ---------------- 基础像素原语 ---------------- */

static inline void px_set(uint8_t *buf, int x, const struct kb_rgb *c)
{
	/* 屏端按“第 2 字节=高字节”解析：先发低字节再发高字节 */
	buf[2 * x] = c->lo;
	buf[2 * x + 1] = c->hi;
}

static void row_fill(uint8_t *buf, int w, const struct kb_rgb *c)
{
	for (int x = 0; x < w; x++) {
		px_set(buf, x, c);
	}
}

/* ---------------- 文字（行条带模式） ---------------- */

/* 把一个字形画进「第 row 行」（row 是所在控件的局部行号） */
static void glyph_row(uint8_t *buf, int w, char ch, int gx, int top, int row,
		      int scale, const struct kb_rgb *c)
{
	const uint8_t *g = kb_lcd_font_glyph(ch);
	int rr;

	if (!g) {
		return;
	}
	rr = row - top;
	if (rr < 0 || rr >= (int)(KB_FONT_H * scale)) {
		return;
	}

	{
		uint8_t bits = g[rr / scale];

		for (int col = 0; col < KB_FONT_W; col++) {
			int x;

			if (!(bits & (1u << col))) {
				continue;
			}
			for (int s = 0; s < scale; s++) {
				x = gx + col * scale + s;
				if (x >= 0 && x < w) {
					px_set(buf, x, c);
				}
			}
		}
	}
}

/* 一行内画完整字符串（row=控件局部行号） */
static void text_row(uint8_t *buf, int w, const char *s, int x0, int top,
		     int row, int scale, int gap, const struct kb_rgb *c)
{
	int adv = KB_FONT_W * scale + gap;

	for (int i = 0; s[i] != '\0'; i++) {
		glyph_row(buf, w, s[i], x0 + i * adv, top, row, scale, c);
	}
}

/* ---------------- 屏写入 ---------------- */

static int flush_row(int x, int y, const uint8_t *buf, uint16_t w)
{
	struct display_buffer_descriptor desc = {
		.width = w,
		.height = 1,
		.pitch = w,
		.buf_size = (uint32_t)w * 2u,
	};

	if (!lcd_dev) {
		return -ENODEV;
	}
	return display_write(lcd_dev, x, y, &desc, buf);
}

/* ---------------- 控件：三模图标 ---------------- */

static const char *const mode_labels[3] = { "USB", "2.4G", "BLE" };
static const enum kb_mode mode_of_block[3] = { KB_MODE_USB, KB_MODE_24G,
						KB_MODE_BLE };

static void draw_mode_block_row(uint8_t *buf, int r, int blk, enum kb_mode cur)
{
	const char *label = mode_labels[blk];
	bool active = (cur != KB_MODE_UNKNOWN && cur == mode_of_block[blk]);
	const struct kb_rgb *fg = active ? &CLR_RED : &CLR_WHITE;
	size_t n = strlen(label);
	int top;
	int x0;
	int lw;

	/* 背景：当前档白底，其余黑底 */
	row_fill(buf, MODE_BLOCK_W, active ? &CLR_WHITE : &CLR_BLACK);

	/* 非当前档：1px 白描边 */
	if (!active) {
		if (r == 0 || r == MODE_BLOCK_H - 1) {
			row_fill(buf, MODE_BLOCK_W, &CLR_WHITE);
		} else {
			px_set(buf, 0, &CLR_WHITE);
			px_set(buf, MODE_BLOCK_W - 1, &CLR_WHITE);
		}
	}

	/* 文字垂直/水平居中 */
	top = (MODE_BLOCK_H - (int)(KB_FONT_H * MODE_TXT_SCALE)) / 2;
	lw = (int)n * KB_FONT_W * MODE_TXT_SCALE + (int)(n - 1) * MODE_TXT_GAP;
	x0 = (MODE_BLOCK_W - lw) / 2;
	text_row(buf, MODE_BLOCK_W, label, x0, top, r, MODE_TXT_SCALE,
		 MODE_TXT_GAP, fg);
}

static void draw_mode_blocks(void)
{
	enum kb_mode cur = kb_mode_get();

	for (int blk = 0; blk < 3; blk++) {
		int bx = MODE_X0 + blk * (MODE_BLOCK_W + MODE_GAP);

		for (int r = 0; r < MODE_BLOCK_H; r++) {
			draw_mode_block_row(row_buf, r, blk, cur);
			flush_row(bx, MODE_Y0 + r, row_buf, MODE_BLOCK_W);
		}
	}
}

/* ---------------- 控件：电量色条 ---------------- */

static const struct kb_rgb *pct_color(uint8_t pct)
{
	if (pct >= 60) {
		return &CLR_GREEN;
	}
	if (pct >= 20) {
		return &CLR_YELLOW;
	}
	return &CLR_RED;
}

static void draw_gauge_row(uint8_t *buf, int r, uint8_t pct, bool valid)
{
	int fill_w;
	int inner_w = GAUGE_W - 2;

	/* 1px 黑框让色条在白底上更醒目 */
	if (r == 0 || r == GAUGE_H - 1) {
		row_fill(buf, GAUGE_W, &CLR_BLACK);
		return;
	}

	row_fill(buf, GAUGE_W, &CLR_WHITE);
	px_set(buf, 0, &CLR_BLACK);
	px_set(buf, GAUGE_W - 1, &CLR_BLACK);

	if (!valid || pct == 0) {
		return;
	}

	fill_w = (inner_w * pct) / 100;
	if (fill_w > inner_w) {
		fill_w = inner_w;
	}
	for (int x = 1; x < GAUGE_W - 1; x++) {
		if ((x - 1) < fill_w) {
			px_set(buf, x, pct_color(pct));
		}
	}
}

static void draw_gauge(uint8_t pct, bool valid)
{
	for (int r = 0; r < GAUGE_H; r++) {
		draw_gauge_row(row_buf, r, pct, valid);
		flush_row(GAUGE_X, GAUGE_Y + r, row_buf, GAUGE_W);
	}
}

/* 电量百分比数字：右对齐排在色条左侧（未采样到有效电量时显示 "--%"） */
static void draw_pct_text(uint8_t pct, bool valid)
{
	char s[8];
	int n;
	int w;
	int x;
	int top;

	if (valid) {
		snprintf(s, sizeof(s), "%u%%", (unsigned)pct);
	} else {
		strncpy(s, "--%", sizeof(s) - 1);
		s[sizeof(s) - 1] = '\0';
	}

	n = (int)strlen(s);
	w = n * (int)(KB_FONT_W * BAT_PCT_SCALE + BAT_PCT_CHAR_GAP)
	    - BAT_PCT_CHAR_GAP;
	x = GAUGE_X - (int)BAT_PCT_GAP - w;
	top = GAUGE_Y + (GAUGE_H - (int)KB_FONT_H) / 2;

	/* 注意：字形“行内偏移”要传 0——落屏 y 已在 flush 里用 top+r 承担，
	 * 若再传 top，glyph_row 的 rr=row-top 恒为负，数字一个字都画不出来。 */
	for (int r = 0; r < KB_FONT_H; r++) {
		row_fill(row_buf, w, &CLR_BLACK);
		text_row(row_buf, w, s, 0, 0, r, BAT_PCT_SCALE,
			 BAT_PCT_CHAR_GAP, &CLR_WHITE);
		flush_row(x, top + r, row_buf, w);
	}
}

/* 充电指示：百分比数字左侧固定槽位显示 “CHG”（充电中）；放电时整槽清黑 */
static void draw_chg_state(bool charging)
{
	int top = GAUGE_Y + ((int)GAUGE_H - (int)KB_FONT_H) / 2;

	for (int r = 0; r < KB_FONT_H; r++) {
		row_fill(row_buf, CHG_TXT_W, &CLR_BLACK);
		if (charging) {
			text_row(row_buf, CHG_TXT_W, "CHG", 0, 0, r,
				 BAT_PCT_SCALE, BAT_PCT_CHAR_GAP, &CLR_WHITE);
		}
		flush_row(CHG_TXT_X, top + r, row_buf, CHG_TXT_W);
	}
}

/* 电池总控件：充电指示 + 百分比数字 + 电量条
 * （数据是否有效：尚无首拍电压时显示 "--%"） */
static void draw_battery_area(uint8_t pct, bool valid, bool charging)
{
	draw_chg_state(charging);
	draw_pct_text(pct, valid);
	draw_gauge(pct, valid);
}

/* ---------------- 控件：第二行时间 ---------------- */

static void draw_time_row(uint8_t *buf, int r, const char *s)
{
	row_fill(buf, TIME_W, &CLR_BLACK);
	text_row(buf, TIME_W, s, 0, 0, r, TIME_SCALE, TIME_GAP, &CLR_WHITE);
}

static void draw_time(const char *s)
{
	for (int r = 0; r < TIME_H; r++) {
		draw_time_row(row_buf, r, s);
		flush_row(TIME_X, TIME_Y + r, row_buf, TIME_W);
	}
}

/* ---------------- 控件：亮度滑动条 ---------------- */

static void draw_slider_row(uint8_t *buf, int r, uint8_t bright)
{
	int fill_w;

	(void)r;

	fill_w = (SLIDER_W * bright) / 100;
	if (fill_w > SLIDER_W) {
		fill_w = SLIDER_W;
	}
	for (int x = 0; x < SLIDER_W; x++) {
		px_set(buf, x, x < fill_w ? &CLR_GREEN : &CLR_WHITE);
	}
}

static void draw_slider(uint8_t bright)
{
	for (int r = 0; r < SLIDER_H; r++) {
		draw_slider_row(row_buf, r, bright);
		flush_row(SLIDER_X, SLIDER_Y + r, row_buf, SLIDER_W);
	}
}

/* ---------------- 全屏清黑（一次性） ---------------- */

static void clear_screen_black(void)
{
	row_fill(row_buf, LCD_W, &CLR_BLACK);
	for (int y = 0; y < LCD_H; y++) {
		flush_row(0, y, row_buf, LCD_W);
	}
}

/* ---------------- 电池/时间的读取与重绘触发 ---------------- */

static bool bat_data_valid(uint16_t mv, uint8_t pct)
{
	/* 还没有首拍电压/百分比 => 视为未知（避免开机瞬间假“低电量红”） */
	return mv != 0 || pct != 0;
}

static void refresh_battery(void)
{
	struct kb_battery_status bat;
	uint16_t mv = kb_bat_adc_get_mv();
	uint8_t pct = kb_bat_adc_get_percent();
	bool valid = bat_data_valid(mv, pct);
	bool charging = false;

	/* 充电中状态来自 IP5306（battery.c 聚合，含 I2C 读取）；失败按放电处理 */
	if (kb_battery_get_status(&bat) == 0) {
		charging = bat.charging;
	}

	if ((pct == last_bat_pct) && (charging == last_charging)) {
		return;
	}
	last_bat_pct = pct;
	last_charging = charging;

	draw_battery_area(pct, valid, charging);
	LOG_DBG("电量刷新: %u%%%s%s", pct, valid ? "" : " (未采样)",
		charging ? " 充电中" : "");
}

static void refresh_time(void)
{
	struct kb_time t;
	char str[32];

	if (kb_time_get(&t) != 0) {
		return;
	}

	snprintf(str, sizeof(str), "%04u-%02u-%02u %02u:%02u:%02u",
		 (unsigned)t.year, (unsigned)t.month, (unsigned)t.day,
		 (unsigned)t.hour, (unsigned)t.minute, (unsigned)t.second);

	if (strcmp(str, last_time) == 0) {
		return;
	}
	strncpy(last_time, str, sizeof(last_time) - 1);
	last_time[sizeof(last_time) - 1] = '\0';

	draw_time(str);
}

static void refresh_top_left(void)
{
	enum kb_mode cur = kb_mode_get();

	if (cur == last_mode) {
		return;
	}
	last_mode = cur;
	draw_mode_blocks();
}

static void refresh_slider(void)
{
	uint8_t bright = kb_backlight_get_percent();

	if (bright == last_bright) {
		return;
	}
	last_bright = bright;
	draw_slider(bright);
}

/* 全量首绘 */
static void redraw_all(void)
{
	last_mode = KB_MODE_UNKNOWN;
	memset(last_time, 0, sizeof(last_time));
	last_bright = 0xff;
	last_bat_pct = 0xff;

	draw_mode_blocks();
	refresh_battery();
	refresh_time();
	refresh_slider();
}

/* ---------------- LCD 专用刷新线程 ---------------- */

static void lcd_thread_entry(void *p1, void *p2, void *p3)
{
	(void)p1;
	(void)p2;
	(void)p3;

	k_sleep(K_MSEC(150)); /* 等电池/模式/时间模块都完成初始化 */
	redraw_all();
	last_bat_check_ms = k_uptime_get();

	for (;;) {
		k_sleep(K_MSEC(LCD_POLL_MS));

		refresh_top_left();

		if (k_uptime_get() - last_bat_check_ms >= BAT_POLL_MS) {
			last_bat_check_ms = k_uptime_get();
			refresh_battery();
		}

		refresh_time();
		refresh_slider();
	}
}

K_THREAD_STACK_DEFINE(lcd_stack, 1024);
static struct k_thread lcd_thread_data;

/* ---------------- 旋钮：方向（导航）模式下调亮度 ---------------- */

static void lcd_knob_cb(enum kb_knob_dir dir, int16_t step, int32_t total,
			int32_t angle, void *user_data)
{
	int pct;
	int delta;

	(void)step;
	(void)total;
	(void)angle;
	(void)user_data;

	/* 数字模式（NumLock 开）=> 旋钮维持原有用途（如 BLE 系统音量） */
	if (hid_numlock_get()) {
		return;
	}

	/* 背光已按实测改为反极性 PWM，软件 pct 与实际亮度一致：顺拧=加亮。
	 * 若顺拧仍变暗，再用 RTT“旋钮 方向=CW/CCW”日志核对物理方向。 */
	delta = (dir == KB_KNOB_CW) ? 5 : -5;
	pct = (int)kb_backlight_get_percent() + delta;
	if (pct < 0) {
		pct = 0;
	}
	if (pct > 100) {
		pct = 100;
	}
	kb_backlight_set_percent((uint8_t)pct);
}

/* ---------------- 初始化 ---------------- */

int kb_lcd_init(void)
{
	int err;

	lcd_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(lcd_dev)) {
		LOG_ERR("显示设备不可用: %s", lcd_dev->name);
		return -ENODEV;
	}

	err = display_blanking_off(lcd_dev);
	if (err != 0) {
		LOG_WRN("打开显示失败(驱动可能默认已点亮): %d", err);
	}

	err = kb_backlight_init();
	if (err != 0) {
		LOG_WRN("背光初始化失败: %d", err);
	}

	/* 先清黑，避免 GRAM 上电乱码残留 */
	clear_screen_black();

	err = kb_knob_register_cb(lcd_knob_cb, NULL);
	if (err != 0) {
		LOG_ERR("订阅旋钮失败: %d", err);
		return err;
	}

	k_thread_create(&lcd_thread_data, lcd_stack,
			K_THREAD_STACK_SIZEOF(lcd_stack),
			lcd_thread_entry, NULL, NULL, NULL,
			7, 0, K_NO_WAIT);

	LOG_INF("LCD 初始化完成");
	return 0;
}

/* ---------------- shell：lcd bl / lcd refresh ---------------- */

static int cmd_lcd_bl(const struct shell *sh, size_t argc, char **argv)
{
	unsigned long val;
	int err = 0;

	(void)argc;

	if (argv[1] == NULL) {
		shell_error(sh, "用法: lcd bl <0-100>");
		return -EINVAL;
	}
	val = shell_strtoul(argv[1], 10, &err);
	if (err != 0) {
		shell_error(sh, "参数不是数字");
		return -EINVAL;
	}
	if (val > 100) {
		shell_error(sh, "亮度需在 0~100 之间");
		return -EINVAL;
	}

	kb_backlight_set_percent((uint8_t)val);
	shell_print(sh, "背光亮度: %u%%", (unsigned)val);
	return 0;
}

static int cmd_lcd_refresh(const struct shell *sh, size_t argc, char **argv)
{
	(void)argc;
	(void)argv;

	redraw_all();
	shell_print(sh, "已强制重绘");
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_lcd,
	SHELL_CMD(bl, NULL, "设置背光亮度: lcd bl <0-100>", cmd_lcd_bl),
	SHELL_CMD(refresh, NULL, "强制全量重绘: lcd refresh", cmd_lcd_refresh),
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(lcd, &sub_lcd, "LCD 调试", NULL);
