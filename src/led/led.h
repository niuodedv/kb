/*
 * 按键 LED（WS2812）模块对外接口
 *
 * 行为：按键按下 -> 对应 LED 立即亮；按键抬起 -> 该 LED 渐灭。
 * 详见 按键led.txt
 */

#ifndef LED_H_
#define LED_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 灯链上的 LED 数量
 *
 * 物理键位 18 个 = 17 个机械轴 + 1 个 EC11 旋钮按键，
 * 其中 (0,3) 的 EC11 旋钮按键没有 LED，故灯链为 17 颗。
 */
#define KB_LED_COUNT 17

/** @brief 该键位没有 LED（6 个空位 + EC11 旋钮按键） */
#define KB_LED_NONE  0xFF

/**
 * @brief BLE 配对/连接状态灯效（占用 (1,1) "/" 键位的 LED）
 *
 * 该键位的按键灯反馈被状态指示征用（按下不再点亮）。
 */
enum kb_led_status {
	/** 熄灭（非 BLE 档或未要求显示） */
	KB_LED_STATUS_OFF = 0,
	/** 已连接：绿灯常亮 */
	KB_LED_STATUS_CONNECTED,
	/** 广播中且已有绑定（等待已配对电脑回连）：蓝灯慢闪 */
	KB_LED_STATUS_ADV_BONDED,
	/** 配对模式（无绑定、可被新主机配对）：蓝灯快闪 */
	KB_LED_STATUS_PAIRING,
	/** 配对成功：绿灯闪三下后自动回到「已连接」常亮 */
	KB_LED_STATUS_PAIR_OK,
};

/** @brief 灯效 */
enum kb_led_effect {
	/** 默认：按下点亮、松手渐灭 */
	KB_LED_EFFECT_DEFAULT = 0,
	/** 常灭（灯带供电被切断） */
	KB_LED_EFFECT_OFF,
	/** 预留：呼吸灯（当前未实现，设置会返回 -ENOTSUP） */
	KB_LED_EFFECT_BREATH,
};

/** @brief 可配置项快照 */
struct kb_led_config {
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t brightness;
	uint8_t effect;
	uint16_t fade_ms;
};

/**
 * @brief 初始化 LED 模块
 *
 * 检查灯带与供电设备是否就绪、初始化互斥锁与延时工作项。
 * 具体点亮/渐灭由按键事件驱动，见 kb_led_press/release。
 *
 * @retval 0 成功
 * @retval -ENODEV 灯带或供电设备未就绪
 * @retval -EINVAL 设备树里的灯链长度与 KB_LED_COUNT 不一致
 */
int kb_led_init(void);

/**
 * @brief 按键按下：点亮该键对应的 LED
 *
 * 非阻塞，可在 input 回调里直接调用。无灯键位（EC11 等）静默忽略。
 *
 * @param row 矩阵行号（0 起）
 * @param col 矩阵列号（0 起）
 * @retval 0 成功（含无灯键位被忽略的情况）
 */
int kb_led_press(uint8_t row, uint8_t col);

/**
 * @brief 按键抬起：该 LED 开始渐灭
 *
 * 非阻塞，可在 input 回调里直接调用。无灯键位静默忽略。
 */
int kb_led_release(uint8_t row, uint8_t col);

/**
 * @brief 立即熄灭所有 LED（模式切换 / 休眠前调用）
 */
void kb_led_all_off(void);

/**
 * @brief 强制某键 LED 常亮（忽略按下/松手，用于 NumLock 等状态指示）
 *
 * 非阻塞，可在任意线程调用。常亮期间该键的 press/release 不再控制其亮度，
 * 直到调用 kb_led_force_off。无灯键位（KB_LED_NONE）静默忽略。
 *
 * @param row 矩阵行号（0 起）
 * @param col 矩阵列号（0 起）
 */
int kb_led_force_on(uint8_t row, uint8_t col);

/**
 * @brief 取消某键 LED 强制常亮，恢复为熄灭/默认按键行为
 */
int kb_led_force_off(uint8_t row, uint8_t col);

/**
 * @brief 设置 BLE 配对/连接状态灯效（见 enum kb_led_status）
 *
 * 非阻塞，可在任意线程调用。相同状态重复设置会被忽略（不重置闪烁相位）。
 * 状态灯占用 (1,1) 键位：该灯有独立颜色（绿/蓝），不受 kb_led_set_color 影响。
 *
 * @retval 0 成功
 * @retval -EINVAL 非法状态值
 */
int kb_led_status_set(enum kb_led_status status);

/** @brief 设置全局颜色（默认 0,255,0 纯绿） */
int kb_led_set_color(uint8_t r, uint8_t g, uint8_t b);

/** @brief 设置全局亮度 0~255，默认 255 */
int kb_led_set_brightness(uint8_t level);

/** @brief 设置灯效，见 enum kb_led_effect */
int kb_led_set_effect(uint8_t effect);

/** @brief 设置松手渐灭时长（毫秒），默认 250 */
int kb_led_set_fade_ms(uint16_t ms);

/** @brief 读取当前配置快照 */
int kb_led_get_config(struct kb_led_config *out);

/**
 * @brief 预留给上位机的统一命令入口
 *
 * 后续 BLE GATT 写回调 / USB HID Vendor Report 回调，收到数据后
 * 【只调这一个函数】即可，LED 模块不感知数据来自蓝牙还是 USB。
 * 帧格式见 按键led.txt 第十一节：
 *     [0]=0xA5 帧头  [1]=CMD  [2]=LEN  [3..]=PAYLOAD  [末]=XOR
 *
 * @param frame     输入帧
 * @param len       输入帧长度
 * @param resp      响应缓冲区，调用方保证 >= 16 字节
 * @param resp_len  输出：响应帧长度
 * @retval 0 成功（命令执行结果体现在 resp 是 ACK 还是 NACK）
 * @retval -EINVAL 参数为 NULL
 */
int kb_led_host_cmd(const uint8_t *frame, uint16_t len,
		    uint8_t *resp, uint16_t *resp_len);

#ifdef __cplusplus
}
#endif

#endif /* LED_H_ */
