/*
 * 电池电压采样模块对外接口（BAT_ADC P0.31 / AIN7 + BAT_ADC_EN P0.09）
 *
 * 电路与流程详见 电池电压测量.txt：
 *   VBAT → R6(100k) → 节点(C29 100nF) → R8(100k) → Q1(NMOS) → GND
 *   BAT_ADC_EN 高电平导通，分压 1/2，V_BAT = 采样值 × 2。
 *
 * 线程模型：专用低优先级线程周期采样（默认 60s），不在系统工作队列
 * 上做 30ms 延时（LED 动画也跑在系统 wq，不能被拖累）。
 */

#ifndef BAT_ADC_H_
#define BAT_ADC_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化电池电压采样
 *
 * 配置 ADC 通道（AIN7）与 EN GPIO（输出低，MOS 默认关断）。
 * 周期采样线程延迟数秒后自启动（等 main 完成外设初始化）。
 *
 * @retval 0 成功
 * @retval -ENODEV ADC/GPIO 设备未就绪
 */
int kb_bat_adc_init(void);

/** @brief 最近一次测得的电池电压（mV），未采样前为 0 */
uint16_t kb_bat_adc_get_mv(void);

/** @brief 最近一次测得的电量百分比（线性映射 3.3~4.2V，待标定） */
uint8_t kb_bat_adc_get_percent(void);

#ifdef __cplusplus
}
#endif

#endif /* BAT_ADC_H_ */
