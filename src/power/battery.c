/*
 * 电池状态聚合（IP5306 I2C + 预留外部 ADC 百分比）
 * 详见 电源管理.txt 4.2：charging/full 来自 IP5306，percent 来自 BAT_ADC（待标定）。
 */
#include "battery.h"
#include "ip5306.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

int kb_battery_get_status(struct kb_battery_status *st)
{
	bool charging = false;
	bool full = false;

	if (!st) {
		return -EINVAL;
	}

	/* 充放电 / 充满来自 IP5306 I2C；读失败则保守填 false */
	if (ip5306_get_charge(&charging) != 0) {
		LOG_WRN("读取 IP5306 充电状态失败");
	}
	if (ip5306_is_full(&full) != 0) {
		LOG_WRN("读取 IP5306 充满状态失败");
	}

	st->charging = charging;
	st->full = full;
	/* TODO: BAT_ADC(P0.31) 经 BAT_ADC_EN 使能后采样 => 电压换算百分比。
	 *       BAT_ADC_EN 引脚（项目记事本标"待确认"）与标定曲线确定后再补。 */
	st->percent = 0U;

	return 0;
}

static int cmd_battery(const struct shell *sh, size_t argc, char **argv)
{
	struct kb_battery_status st;

	kb_battery_get_status(&st);
	shell_print(sh, "battery: charging=%d full=%d percent=%u%%",
		    st.charging, st.full, st.percent);
	return 0;
}

SHELL_CMD_REGISTER(battery, NULL, "IP5306 battery status (charging/full)", cmd_battery);
