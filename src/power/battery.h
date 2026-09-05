/*
 * 电池状态聚合接口（IP5306 I2C + 预留外部 ADC 百分比）
 * 详见 电源管理.txt 4.2：charging/full 来自 IP5306，percent 来自 BAT_ADC（待标定）。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct kb_battery_status {
	bool charging;   /* true=充电中, false=放电 */
	bool full;       /* true=已充满 */
	uint8_t percent; /* 0~100，来自外部 ADC；本阶段预留接口，恒为 0 */
};

/**
 * 聚合电池状态：充放电 / 充满来自 IP5306 I2C，百分比预留（BAT_ADC 待标定）。
 * @return 0 成功，负值为错误码
 */
int kb_battery_get_status(struct kb_battery_status *st);

#ifdef __cplusplus
}
#endif
