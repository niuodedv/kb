/*
 * LCD 背光模块对外接口（PWM 调亮度，0~100%）
 *
 * 硬件：nRF PWM0 OUT0 -> P1.11 (SCREEN_BLK)，默认输出高有效。
 * 详见 lcd 显示.txt【4.5】
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化背光 PWM 并设为默认亮度
 *
 * @retval 0 成功
 */
int kb_backlight_init(void);

/**
 * @brief 设置背光亮度
 *
 * @param pct 0..100（0 关背光）
 * @retval 0 成功
 */
int kb_backlight_set_percent(uint8_t pct);

/** @brief 读取当前亮度（0..100） */
uint8_t kb_backlight_get_percent(void);

#ifdef __cplusplus
}
#endif
