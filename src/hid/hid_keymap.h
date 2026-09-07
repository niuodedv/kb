/*
 * 公共 HID 翻译层（BLE / USB 两个 transport 共用）
 *
 * 从 ble_hid.c 抽出的「与传输方式无关」的 HID 逻辑：
 *   - NumLock 双功能层翻译（dual_keys[] / hid_keymap_resolve）
 *   - 按键状态机（修饰键位图 + 6KRO 键值槽）
 *   - 主机 NumLock LED Output Report 同步（含 (1,0) 指示灯常亮）
 *   - HID 报告描述符（BLE rep_map 与 USB HID 描述符共用同一份）
 *
 * ⚠️ 本层只产出「载荷」（键盘 8 字节 / Consumer 2 字节）。
 *    报文头由各 transport 自己负责：
 *      BLE  不带 Report ID（由 GATT 特征区分）
 *      USB  中断传输报文首字节【必须】带 Report ID（键盘 9 字节 / Consumer 3 字节）
 *
 * 详见 usb 通信.txt / BLE通信.txt
 */

#ifndef HID_KEYMAP_H_
#define HID_KEYMAP_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Consumer 用法（Usage Page 0x0C），BLE / USB 共用 ---- */
#define HID_CONSUMER_USAGE_VOL_UP   0x00E9
#define HID_CONSUMER_USAGE_VOL_DOWN 0x00EA
#define HID_CONSUMER_USAGE_MUTE     0x00E2

/* ---- LED Output Report 的位定义（Usage Page 0x08）---- */
#define HID_LED_BIT_NUM_LOCK BIT(0)

/* ---- Report ID（USB 报文头用；BLE 报文不含）---- */
#define HID_REPORT_ID_KEYS     1
#define HID_REPORT_ID_LEDS     2
#define HID_REPORT_ID_CONSUMER 3

/* 键盘按键状态：修饰键位图 + 最多 6 个键值槽（6KRO） */
struct hid_kb_state {
	uint8_t modifier;
	uint8_t keys[6];
};

/** HID 报告描述符（BLE HIDS rep_map 与 USB hid_device_register 共用） */
extern const uint8_t hid_report_map[];
extern const size_t hid_report_map_size;

/**
 * @brief INPUT_KEY_* -> HID 修饰键位图（0 表示非修饰键）
 */
uint8_t hid_keymap_modifier(uint16_t input_code);

/**
 * @brief INPUT_KEY_* -> HID 键盘用法，带 NumLock 层切换
 *
 * @retval 0 成功；-ENOENT 表示该键值未映射到 HID
 */
int hid_keymap_resolve(uint16_t input_code, uint8_t *usage);

/** @brief 当前按键状态（只读；transport 据此组包发送） */
const struct hid_kb_state *hid_kb_state_get(void);

/** @brief 清空全部按键状态（模式切换 / 断连前的「全键释放」） */
void hid_kb_state_reset(void);

/**
 * @brief 更新按键状态
 *
 * @param usage    HID 键盘用法（hid_keymap_resolve 的输出）
 * @param mod_mask 修饰键位图（hid_keymap_modifier 的输出，非 0 时只操作位图）
 * @param pressed  true 按下 / false 松开
 */
void hid_kb_state_key(uint8_t usage, uint8_t mod_mask, bool pressed);

/** @brief 当前 NumLock 状态（由主机 LED Output Report 同步，默认开） */
bool hid_numlock_get(void);

/**
 * @brief 同步主机 NumLock 状态
 *
 * 内部处理双功能层切换与 NumLock 键 (1,0) LED 指示：
 * 方向（导航）模式下常亮，数字模式下熄灭。
 */
void hid_numlock_set(bool on);

#ifdef __cplusplus
}
#endif

#endif /* HID_KEYMAP_H_ */
