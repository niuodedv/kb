/*
 * LCD 屏幕 UI 模块对外接口
 *
 * 硬件/接线/实现设计详见 lcd 显示.txt：
 *   - 320x172 ST7789V，走 Zephyr mipi-dbi-spi + display API
 *   - 第一行：三模图标 + 电量色条(靠右) + 电量百分比数字
 *   - 第二行：大字时钟 yyyy-MM-dd HH:mm:ss（软件时钟，kb_time）
 *   - 第三行：亮度滑动条
 * 交互：方向（导航）模式（NumLock 关）下旋转旋钮调背光亮度；
 *       数字模式（NumLock 开）下旋钮维持原有用途（BLE 音量）。
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 LCD：打开显示、清黑屏、起专用刷新线程
 *
 * 内部还会初始化背光 PWM 并订阅旋钮（方向模式下旋钮调背光亮度）。
 *
 * @retval 0 成功
 */
int kb_lcd_init(void);

#ifdef __cplusplus
}
#endif
