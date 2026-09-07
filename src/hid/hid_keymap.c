/*
 * 公共 HID 翻译层实现（BLE / USB 共用，接口说明见 hid_keymap.h）
 *
 * 线程模型：状态修改只发生在 input 线程（transport 回调）与
 * 断连/模式切换的清零路径，且同一时刻只有一个档位在活跃处理，
 * 与原 ble_hid.c 的使用方式一致，不额外加锁。
 */

#include "hid_keymap.h"

#include <errno.h>
#include <string.h>

#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/class/hid.h>

#include "led/led.h"

LOG_MODULE_REGISTER(hid_keymap, LOG_LEVEL_INF);

/* Keypad 的句点用法：zephyr/usb/class/hid.h 的枚举里没有，按 USB HID 标准补齐 */
#define HID_KEY_KP_DOT 0x63

#define KEY_PRESS_MAX    6

/*
 * NumLock 双功能层（矩阵按键.txt 6.3 / BLE通信.txt 5.1）
 *
 * 设备树的 keymap 只放了「数字层」（INPUT_KEY_KP*）。NumLock 关闭时要发导航键，
 * 这一层的切换放在本表做；其余键（/ * - + Enter NumLock 5）两层一样，
 * 直接走 Zephyr 官方的 input_to_hid_code() 翻译。
 *
 * ⚠️ INPUT_KEY_KPDOT 恰好不在官方映射表里（会返回 -1），必须在本表中显式给出。
 */
struct dual_key {
	uint16_t input_code;
	uint8_t hid_num;   /* NumLock 开：数字层 */
	uint8_t hid_nav;   /* NumLock 关：导航层 */
};

static const struct dual_key dual_keys[] = {
	{INPUT_KEY_KP7,   HID_KEY_KP_7,   HID_KEY_HOME},
	{INPUT_KEY_KP8,   HID_KEY_KP_8,   HID_KEY_UP},
	{INPUT_KEY_KP9,   HID_KEY_KP_9,   HID_KEY_PAGEUP},
	{INPUT_KEY_KP4,   HID_KEY_KP_4,   HID_KEY_LEFT},
	{INPUT_KEY_KP6,   HID_KEY_KP_6,   HID_KEY_RIGHT},
	{INPUT_KEY_KP1,   HID_KEY_KP_1,   HID_KEY_END},
	{INPUT_KEY_KP2,   HID_KEY_KP_2,   HID_KEY_DOWN},
	{INPUT_KEY_KP3,   HID_KEY_KP_3,   HID_KEY_PAGEDOWN},
	{INPUT_KEY_KP0,   HID_KEY_KP_0,   HID_KEY_INSERT},
	{INPUT_KEY_KPDOT, HID_KEY_KP_DOT, HID_KEY_DELETE},
};

static struct hid_kb_state kbd_state;
static bool numlock_on = true;

/* ==================== HID 报告描述符 ====================
 * Report ID 1 = Keyboard Keys（输入，8 字节：修饰键 + 保留 + 6 键值）
 * Report ID 2 = Keyboard LEDs（输出，1 字节：用于同步主机 NumLock）
 * Report ID 3 = Consumer Control（输入，2 字节：16 位 Consumer 用法）
 * BLE 的 rep_map 与 USB 的 hid_device_register 共用这一份。
 */
const uint8_t hid_report_map[] = {
	/* ---- Collection: Keyboard ---- */
	0x05, 0x01,       /* Usage Page (Generic Desktop) */
	0x09, 0x06,       /* Usage (Keyboard) */
	0xA1, 0x01,       /* Collection (Application) */

	/* ---- Report ID 1: Keyboard Keys (input) ---- */
	0x85, HID_REPORT_ID_KEYS,
	0x05, 0x07,       /* Usage Page (Keyboard/Keypad) */
	0x19, 0xE0,       /* Usage Minimum (0xE0) */
	0x29, 0xE7,       /* Usage Maximum (0xE7) */
	0x15, 0x00,       /* Logical Minimum (0) */
	0x25, 0x01,       /* Logical Maximum (1) */
	0x75, 0x01,       /* Report Size (1) */
	0x95, 0x08,       /* Report Count (8) */
	0x81, 0x02,       /* Input (Data, Variable, Absolute) - 修饰键 */

	0x95, 0x01,       /* Report Count (1) */
	0x75, 0x08,       /* Report Size (8) */
	0x81, 0x01,       /* Input (Constant) - 保留字节 */

	0x95, 0x06,       /* Report Count (6) */
	0x75, 0x08,       /* Report Size (8) */
	0x15, 0x00,       /* Logical Minimum (0) */
	0x25, 0x65,       /* Logical Maximum (0x65) */
	0x05, 0x07,       /* Usage Page (Keyboard/Keypad) */
	0x19, 0x00,       /* Usage Minimum (0) */
	0x29, 0x65,       /* Usage Maximum (0x65) */
	0x81, 0x00,       /* Input (Data, Array) - 6 个键值，即 6KRO */

	/* ---- Report ID 2: Keyboard LEDs (output) ---- */
	0x85, HID_REPORT_ID_LEDS,
	0x95, 0x05,       /* Report Count (5) */
	0x75, 0x01,       /* Report Size (1) */
	0x05, 0x08,       /* Usage Page (LEDs) */
	0x19, 0x01,       /* Usage Minimum (1) - Num Lock */
	0x29, 0x05,       /* Usage Maximum (5) */
	0x91, 0x02,       /* Output (Data, Variable, Absolute) */
	0x95, 0x01,       /* Report Count (1) */
	0x75, 0x03,       /* Report Size (3) */
	0x91, 0x01,       /* Output (Constant) - 补齐到 1 字节 */

	0xC0,             /* End Collection (Keyboard) */

	/* ---- Collection: Consumer Control（系统音量）---- */
	0x05, 0x0C,       /* Usage Page (Consumer) */
	0x09, 0x01,       /* Usage (Consumer Control) */
	0xA1, 0x01,       /* Collection (Application) */
	0x85, HID_REPORT_ID_CONSUMER,
	0x15, 0x00,       /* Logical Minimum (0) */
	0x26, 0xFF, 0x03, /* Logical Maximum (0x03FF) */
	0x19, 0x00,       /* Usage Minimum (0) */
	0x2A, 0xFF, 0x03, /* Usage Maximum (0x03FF) */
	0x75, 0x10,       /* Report Size (16) */
	0x95, 0x01,       /* Report Count (1) */
	0x81, 0x00,       /* Input (Data, Array, Absolute) - 16 位用法值 */
	0xC0,             /* End Collection (Consumer) */
};

const size_t hid_report_map_size = sizeof(hid_report_map);

/* ==================== 翻译 ==================== */

uint8_t hid_keymap_modifier(uint16_t input_code)
{
	return input_to_hid_modifier(input_code);
}

int hid_keymap_resolve(uint16_t input_code, uint8_t *usage)
{
	for (size_t i = 0; i < ARRAY_SIZE(dual_keys); i++) {
		if (dual_keys[i].input_code == input_code) {
			*usage = numlock_on ? dual_keys[i].hid_num
					    : dual_keys[i].hid_nav;
			return 0;
		}
	}

	int16_t code = input_to_hid_code(input_code);

	if (code < 0) {
		return -ENOENT;
	}

	*usage = (uint8_t)code;
	return 0;
}

/* ==================== 按键状态 ==================== */

const struct hid_kb_state *hid_kb_state_get(void)
{
	return &kbd_state;
}

void hid_kb_state_reset(void)
{
	memset(&kbd_state, 0, sizeof(kbd_state));
}

void hid_kb_state_key(uint8_t usage, uint8_t mod_mask, bool pressed)
{
	if (mod_mask != 0) {
		if (pressed) {
			kbd_state.modifier |= mod_mask;
		} else {
			kbd_state.modifier &= ~mod_mask;
		}
		return;
	}

	if (pressed) {
		for (int i = 0; i < KEY_PRESS_MAX; i++) {
			if (kbd_state.keys[i] == 0) {
				kbd_state.keys[i] = usage;
				return;
			}
		}

		LOG_WRN("键值槽已满（6KRO），按键 0x%02X 被丢弃", usage);
	} else {
		for (int i = 0; i < KEY_PRESS_MAX; i++) {
			if (kbd_state.keys[i] == usage) {
				kbd_state.keys[i] = 0;
				return;
			}
		}
	}
}

/* ==================== 主机 NumLock 同步 ==================== */

bool hid_numlock_get(void)
{
	return numlock_on;
}

void hid_numlock_set(bool on)
{
	if (on == numlock_on) {
		return;
	}

	numlock_on = on;

	/*
	 * NumLock 灯指示：方向（导航）模式下 NumLock 键 LED 常亮，
	 * 数字模式下取消常亮。
	 */
	if (numlock_on) {
		kb_led_force_off(1, 0);
	} else {
		kb_led_force_on(1, 0);
	}

	LOG_INF("主机 NumLock: %s（%s层生效）%s", numlock_on ? "开" : "关",
		numlock_on ? "数字" : "导航",
		numlock_on ? "" : "，NumLock 灯常亮");
}
