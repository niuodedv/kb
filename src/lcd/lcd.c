/*
 * LCD 屏幕 UI 主模块（ST7789V，320x172，SPI 只写）
 *
 * 运行模型：lcd 专用线程，周期巡检 + 仅在状态变化时局部重绘对应控件
 *（不建整帧影子缓冲；按“控件=若干行”行条带 + display_write 落屏）。
 *
 * 布局见 lcd 显示.txt【4.2】：
 *   第一行：左=三模图标（当前白底红字，其余黑底白字描白边）
 *           右=电池图标（白壳+正极凸头，内芯分色填充）+ 放大 2 倍的
 *             电量百分比数字（未采样显示 "--%"）；充电时 “CHG” 绿色小字
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

/* 行 1：右上角电量组 = [CHG 槽][放大百分比数字][电池图标(外壳+正极凸头)]，
 * 与左侧三模图标块同高(24px)并垂直居中对齐；图标整体靠最右(屏右留 8px)。 */
#define GRP_Y0         (MODE_Y0)
#define GRP_H          (MODE_BLOCK_H)
#define BAT_RIGHT      (LCD_W - 8)     /* 组右缘(不含) */

/* 电池图标几何：白壳(2px 线宽，顶/底行两端各留 2px 成圆角观感)内嵌
 * 2px 黑色间隙的内容区，内容区按电量比例填色；右侧实心凸头贴壳居中。 */
#define SHELL_W        26
#define SHELL_H        16
#define SHELL_FRAME    2u
#define SHELL_GAP      2u
#define TAB_W          3u
#define TAB_H          8u
#define INNER_W        (SHELL_W - 2 * (SHELL_FRAME + SHELL_GAP))  /* 18 */
#define INNER_H        (SHELL_H - 2 * (SHELL_FRAME + SHELL_GAP))  /* 8  */
#define INNER_OFF      (SHELL_FRAME + SHELL_GAP)                  /* 4  */
#define SHELL_X        (BAT_RIGHT - TAB_W - SHELL_W)
#define SHELL_Y        (GRP_Y0 + (GRP_H - SHELL_H) / 2)
#define GAUGE_BUF_W    (SHELL_W + TAB_W)                          /* 行缓冲宽 */

/* 百分比大数字：放大 2 倍，右对齐固定槽（槽宽按最宽 “100%”），
 * 槽位固定 => 数字位数变化时电池图标不会左右抖动。 */
#define BAT_PCT_GAP       6u   /* 数字槽与电池壳间距 */
#define BAT_PCT_SCALE     2u
#define BAT_PCT_CHAR_GAP  1u
#define BAT_PCT_SLOT_W    (4 * ((int)KB_FONT_W * BAT_PCT_SCALE + BAT_PCT_CHAR_GAP) \
			   - BAT_PCT_CHAR_GAP)                       /* “100%” 宽 */
#define BAT_PCT_X_R       (SHELL_X - BAT_PCT_GAP)
#define BAT_PCT_X         (BAT_PCT_X_R - BAT_PCT_SLOT_W)
#define BAT_PCT_Y         (GRP_Y0 + (GRP_H - (int)(KB_FONT_H * BAT_PCT_SCALE)) / 2)

/* 充电指示 “CHG”：固定槽位，位于百分比数字槽左侧（充电时绿色，平时整槽清黑）。 */
#define CHG_TXT_GAP      4u
#define CHG_TXT_W        (3 * ((int)KB_FONT_W + BAT_PCT_CHAR_GAP) - BAT_PCT_CHAR_GAP)
#define CHG_TXT_X        (BAT_PCT_X - (int)CHG_TXT_GAP - CHG_TXT_W)
#define CHG_TXT_Y        (GRP_Y0 + (GRP_H - (int)KB_FONT_H) / 2)

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

/* 低功耗挂起控制：kb_lcd_set_idle() 置标志 + 给信号量唤醒线程，
 * 线程按当前 idle 标志决定挂起还是恢复巡检（见 lcd_thread_entry）。 */
static struct k_sem lcd_ctrl;
static bool lcd_idle;

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

/* 电池图标单行：白壳(顶/底 2px、左右 2px 竖框，顶/底两端切角圆角) +
 * 右侧正极凸头 + 内容区按比例填充（放电随电量 绿/黄/红，充电整条绿）。
 * r 为图标局部行号(0..SHELL_H-1)，buf 宽 GAUGE_BUF_W(=壳+凸头)。 */
static void draw_gauge_row(uint8_t *buf, int r, uint8_t pct, bool valid,
			   bool charging)
{
	int i;
	int fill_w;
	int tab_top = (int)(SHELL_H - TAB_H) / 2;

	row_fill(buf, GAUGE_BUF_W, &CLR_BLACK);

	/* 白壳：2px 边框，四角留 SHELL_FRAME 切角成圆角观感 */
	if (r < (int)SHELL_FRAME || r >= (int)SHELL_H - (int)SHELL_FRAME) {
		for (i = (int)SHELL_FRAME; i < (int)SHELL_W - (int)SHELL_FRAME;
		     i++) {
			px_set(buf, i, &CLR_WHITE);
		}
	} else if (r < (int)SHELL_H) {
		for (i = 0; i < (int)SHELL_FRAME; i++) {
			px_set(buf, i, &CLR_WHITE);
			px_set(buf, (int)SHELL_W - 1 - i, &CLR_WHITE);
		}
	}

	/* 内容区（内嵌 2px 黑色间隙）按比例填电量色 */
	if (valid && pct != 0 && r >= (int)INNER_OFF &&
	    r < (int)INNER_OFF + (int)INNER_H) {
		const struct kb_rgb *col = charging ? &CLR_GREEN : pct_color(pct);

		fill_w = ((int)INNER_W * pct) / 100;
		if (fill_w > (int)INNER_W) {
			fill_w = (int)INNER_W;
		}
		for (i = (int)INNER_OFF; i < (int)INNER_OFF + fill_w; i++) {
			px_set(buf, i, col);
		}
	}

	/* 正极凸头：贴壳右侧、垂直居中实心白 */
	if (r >= tab_top && r < tab_top + (int)TAB_H) {
		for (i = (int)SHELL_W; i < (int)GAUGE_BUF_W; i++) {
			px_set(buf, i, &CLR_WHITE);
		}
	}
}

static void draw_gauge(uint8_t pct, bool valid, bool charging)
{
	for (int r = 0; r < SHELL_H; r++) {
		draw_gauge_row(row_buf, r, pct, valid, charging);
		flush_row(SHELL_X, SHELL_Y + r, row_buf, GAUGE_BUF_W);
	}
}

/* 电量百分比数字：放大 2 倍，右对齐固定槽排到电池图标左侧
 * （未采样到有效电量时显示 "--%"） */
static void draw_pct_text(uint8_t pct, bool valid)
{
	char s[8];
	int n;
	int w;
	int x0;
	int rows = (int)(KB_FONT_H * BAT_PCT_SCALE);
	int r;

	if (valid) {
		snprintf(s, sizeof(s), "%u%%", (unsigned)pct);
	} else {
		strncpy(s, "--%", sizeof(s) - 1);
		s[sizeof(s) - 1] = '\0';
	}

	n = (int)strlen(s);
	w = n * (int)(KB_FONT_W * BAT_PCT_SCALE + BAT_PCT_CHAR_GAP)
	    - BAT_PCT_CHAR_GAP;
	x0 = BAT_PCT_SLOT_W - w;   /* 槽内右对齐 */

	/* 注意：字形“行内偏移”要传 0——落屏 y 已在 flush 里用 top+r 承担，
	 * 若再传 top，glyph_row 的 rr=row-top 恒为负，数字一个字都画不出来。 */
	for (r = 0; r < rows; r++) {
		row_fill(row_buf, BAT_PCT_SLOT_W, &CLR_BLACK);
		text_row(row_buf, BAT_PCT_SLOT_W, s, x0, 0, r, BAT_PCT_SCALE,
			 BAT_PCT_CHAR_GAP, &CLR_WHITE);
		flush_row(BAT_PCT_X, BAT_PCT_Y + r, row_buf, BAT_PCT_SLOT_W);
	}
}

/* 充电指示：数字槽左侧固定槽显示 “CHG”（充电中，绿色）；放电整槽清黑 */
static void draw_chg_state(bool charging)
{
	int r;

	for (r = 0; r < (int)KB_FONT_H; r++) {
		row_fill(row_buf, CHG_TXT_W, &CLR_BLACK);
		if (charging) {
			text_row(row_buf, CHG_TXT_W, "CHG", 0, 0, r, 1,
				 BAT_PCT_CHAR_GAP, &CLR_GREEN);
		}
		flush_row(CHG_TXT_X, CHG_TXT_Y + r, row_buf, CHG_TXT_W);
	}
}

/* 电池总控件：充电指示 + 百分比数字 + 电池图标
 * （数据是否有效：尚无首拍电压时显示 "--%"） */
static void draw_battery_area(uint8_t pct, bool valid, bool charging)
{
	draw_chg_state(charging);
	draw_pct_text(pct, valid);
	draw_gauge(pct, valid, charging);
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
		if (lcd_idle) {
			/* 低功耗挂起：先把滑条按“当前(已调低)亮度”补一帧，
			 * 再停等 kb_lcd_set_idle(false) 唤醒（期间不刷 SPI/不读 I2C）。 */
			refresh_slider();
			LOG_DBG("LCD 刷新挂起（低功耗），画面冻结");
			k_sem_take(&lcd_ctrl, K_FOREVER);

			if (!lcd_idle) {
				/* 被唤醒：全量重绘补齐走秒/电量/滑条，再回常规节奏 */
				redraw_all();
				last_bat_check_ms = k_uptime_get();
				LOG_DBG("LCD 刷新恢复，全量重绘");
			}
			continue;
		}

		k_sem_take(&lcd_ctrl, K_MSEC(LCD_POLL_MS));

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

/* ---------------- 低功耗挂起/恢复 ---------------- */

int kb_lcd_set_idle(bool idle)
{
	if (lcd_idle == idle) {
		return 0;
	}

	/* 标志 + 信号量：线程要么被立刻唤醒处理（退出挂起会全量重绘），
	 * 要么在下一个循环看到标志转入挂起等待。线程内执行，无阻塞。 */
	lcd_idle = idle;
	k_sem_give(&lcd_ctrl);

	return 0;
}

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

	k_sem_init(&lcd_ctrl, 0, 1);   /* 信号量初值 0，最大 1（挂起/唤醒通知） */

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
