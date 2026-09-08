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

#include <stdbool.h>
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

/**
 * @brief 挂起/恢复 LCD 周期刷新（低功耗档 1/2 用）
 *
 * - idle=true ：停止 200ms 巡检与 1s 电池 I2C 读取，画面保持最后一帧
 *   （ST7789 有 GRAM 自刷新），刷新线程转入挂起等待。
 * - idle=false：立即唤醒线程做一次全量重绘，再恢复正常巡检节奏。
 *
 * @param idle true=挂起刷新 / false=恢复刷新
 * @retval 0 成功
 *
 * @note 本接口只控制“刷新”，不碰背光亮度（由调用方 kb_backlight_set_percent）
 */
int kb_lcd_set_idle(bool idle);

#ifdef __cplusplus
}
#endif
