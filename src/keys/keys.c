/*
 * 按键模块实现
 *
 * 订阅两个 input 设备：
 *   1) kbd-matrix（gpio-kbd-matrix 驱动）-> 得到 (行, 列, 按下/松开)
 *   2) keymap（input-keymap 驱动）      -> 得到解析后的标准键值 INPUT_KEY_xxx
 *
 * 之所以两个都订阅：bring-up 阶段需要同时核对「物理位置」与「解析出的键值」，
 * 用来确认键帽与 (行,列) 的对应关系（尤其 ROW5，见 矩阵按键.txt 6.2）。
 */

#include "keys.h"

#include <errno.h>

#include "led/led.h"

#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(keys, LOG_LEVEL_INF);

#define MATRIX_NODE DT_NODELABEL(kbd_matrix)
#define KEYMAP_NODE DT_NODELABEL(kbd_keymap)

static const struct device *const matrix_dev = DEVICE_DT_GET(MATRIX_NODE);
static const struct device *const keymap_dev = DEVICE_DT_GET(KEYMAP_NODE);

struct key_desc {
	uint8_t row;
	uint8_t col;
	const char *name;
};

/*
 * (行, 列) -> 键名
 *
 * 说明：ROW5 的三个键（COL0/COL1/COL3）与原理图内嵌键帽图例无法完全对应，
 *       名称后带 "?" 的需要用 shell 命令实测确认后修正（改这里即可，不用改设备树）。
 */
static const struct key_desc key_map[] = {
	{0, 3, "EC11 旋钮按键(Mute)"},
	{1, 0, "NumLock"},
	{1, 1, "/"},
	{1, 2, "*"},
	{1, 3, "-"},
	{2, 0, "7 / Home"},
	{2, 1, "8 / Up"},
	{2, 2, "9 / PgUp"},
	{3, 0, "4 / Left"},
	{3, 1, "5"},
	{3, 2, "6 / Right"},
	{3, 3, "+"},
	{4, 0, "1 / End"},
	{4, 1, "2 / Down"},
	{4, 2, "3 / PgDn"},
	{5, 0, "0 / Ins?"},
	{5, 1, ". / Del?"},
	{5, 3, "Enter?"},
};

const char *keys_key_name(uint8_t row, uint8_t col)
{
	for (size_t i = 0; i < ARRAY_SIZE(key_map); i++) {
		if ((key_map[i].row == row) && (key_map[i].col == col)) {
			return key_map[i].name;
		}
	}

	return NULL;
}

/*
 * 矩阵设备的事件以三个一组上报：ABS_X=列、ABS_Y=行、BTN_TOUCH=按下，
 * 最后一条带 sync 标志，收到 sync 才算一组完整事件。
 */
static uint8_t cur_row;
static uint8_t cur_col;
static bool cur_pressed;

static void matrix_cb(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	switch (evt->code) {
	case INPUT_ABS_X:
		cur_col = (uint8_t)evt->value;
		break;
	case INPUT_ABS_Y:
		cur_row = (uint8_t)evt->value;
		break;
	case INPUT_BTN_TOUCH:
		cur_pressed = (evt->value != 0);
		break;
	default:
		break;
	}

	if (!evt->sync) {
		return;
	}

	const char *name = keys_key_name(cur_row, cur_col);

	if (name == NULL) {
		LOG_WRN("未定义按键: ROW%u COL%u (key_id=%u) %s",
			cur_row, cur_col, KB_KEY_ID(cur_row, cur_col),
			cur_pressed ? "按下" : "松开");
		return;
	}

	LOG_INF("按键 %-22s ROW%u COL%u key_id=%2u -> %s",
		name, cur_row, cur_col, KB_KEY_ID(cur_row, cur_col),
		cur_pressed ? "按下" : "松开");

	/*
	 * 驱动对应的按键 LED：按下点亮，松手渐灭。
	 * EC11 旋钮按键 (0,3) 等无灯键位由 led 模块查表后静默忽略。
	 * 这两个函数只改状态 + 提交工作项，真正的 SPI 刷新在工作队列里做，
	 * 不会阻塞 input 线程。
	 */
	if (cur_pressed) {
		kb_led_press(cur_row, cur_col);
	} else {
		kb_led_release(cur_row, cur_col);
	}
}

/* 键值映射设备的事件：已经是标准键值，每条自带 sync */
static void keymap_cb(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	if (evt->type != INPUT_EV_KEY) {
		return;
	}

	LOG_INF("  键值 INPUT_KEY_%-3u -> %s", evt->code,
		evt->value ? "按下" : "松开");
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_NODELABEL(kbd_matrix)), matrix_cb, NULL);
INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_NODELABEL(kbd_keymap)), keymap_cb, NULL);

int keys_init(void)
{
	if (!device_is_ready(matrix_dev)) {
		LOG_ERR("矩阵设备未就绪: %s", matrix_dev->name);
		return -ENODEV;
	}

	if (!device_is_ready(keymap_dev)) {
		LOG_ERR("键值映射设备未就绪: %s", keymap_dev->name);
		return -ENODEV;
	}

	LOG_INF("矩阵就绪: %s (%d 行 x %d 列)", matrix_dev->name,
		DT_PROP_LEN(MATRIX_NODE, row_gpios),
		DT_PROP_LEN(MATRIX_NODE, col_gpios));
	LOG_INF("键值映射就绪: %s", keymap_dev->name);

	return 0;
}
