/*
 * IP5306-I2C 电源管理芯片驱动（英集芯定制 I2C 版本）
 *
 * 职责（详见 电源管理.txt）：
 *   (A) 每 10s 向 IP5306 KEY(WAKEUP=P0.22) 发一个短按脉冲，阻止其轻载待机，
 *       => SYS_POWER(5V) 不掉电，nRF 不意外断电；
 *   (B) I2C 读取充放电 / 充满状态，供电池状态接口使用。
 */
#pragma once

#include <stdbool.h>
#include <errno.h>


#ifdef __cplusplus
extern "C" {
#endif

/**
 * 初始化 IP5306：获取 I2C 设备 / WAKEUP GPIO，并启动 10s 保活定时器。
 * 必须在 kb_battery_get_status() 之前调用。
 * @return 0 成功，负值为错误码
 */
int ip5306_init(void);

/** 读充电中标志（true=充电 / false=放电），来自 REG_READ0.bit3 */
int ip5306_get_charge(bool *charging);

/** 读充满标志（true=已充满），来自 REG_READ1.bit3 */
int ip5306_is_full(bool *full);

/** 启动 10s 保活脉冲（阻止 IP5306 轻载待机） */
void ip5306_keepalive_start(void);

/** 停止保活脉冲（准备 System Off 整机断电前调用） */
void ip5306_keepalive_stop(void);

/**
 * 设置 BOOST 常开（SYS_CTL0.bit1）。
 * 仅作备选/调试：常开后 IP5306 永不轻载关机，待机电流升至 ~3mA，
 * 且 System Off 时无法真正断电，不建议日常使用。
 */
int ip5306_set_boost_always_on(bool on);

#ifdef __cplusplus
}
#endif
