/*
 * LCD 背光模块实现（PWM 调亮度 0~100%）
 *
 * 硬件：nRF PWM0 OUT0 = P1.11 (SCREEN_BLK)。
 * 频率取 10kHz（超过可闻范围），由 PWM 驱动按内部 16MHz 时钟换算占空比。
 * 实测：屏背光为“低电平点亮”（或等效反相驱动），故必须用 PWM_POLARITY_INVERTED，
 * 使 pct 越大 → 低电平占空越大 → 越亮（曾用 normal 极性导致亮度与百分比正好相反）。
 * 详见 lcd 显示.txt【4.5】
 */

#include "backlight.h"

#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

LOG_MODULE_REGISTER(kb_backlight, LOG_LEVEL_INF);

#define BACKLIGHT_PWM_DEV  DT_NODELABEL(pwm0)
#define BACKLIGHT_CHANNEL  0u
#define BACKLIGHT_PERIOD_NS 100000u   /* 10 kHz */
#define BACKLIGHT_DEFAULT_PCT 60u
#define BACKLIGHT_MAX_PCT 100u

static const struct device *bl_dev;
static uint8_t bl_pct = BACKLIGHT_DEFAULT_PCT;

int kb_backlight_init(void)
{
	bl_dev = DEVICE_DT_GET(BACKLIGHT_PWM_DEV);
	if (!device_is_ready(bl_dev)) {
		LOG_ERR("PWM 背光设备不可用");
		return -ENODEV;
	}

	bl_pct = BACKLIGHT_DEFAULT_PCT;
	return kb_backlight_set_percent(bl_pct);
}

int kb_backlight_set_percent(uint8_t pct)
{
	uint32_t pulse;
	int err;

	if (pct > BACKLIGHT_MAX_PCT) {
		pct = BACKLIGHT_MAX_PCT;
	}

	if (!bl_dev || !device_is_ready(bl_dev)) {
		return -ENODEV;
	}

	/* pwm_set() 按纳秒换算成周期/脉宽（内部时钟 16MHz），驱动自动选分频。
	 * 反极性：脉宽即“低电平”点亮时长，pct 越大越亮。 */
	pulse = (BACKLIGHT_PERIOD_NS * pct) / BACKLIGHT_MAX_PCT;
	err = pwm_set(bl_dev, BACKLIGHT_CHANNEL, BACKLIGHT_PERIOD_NS,
		      pulse, PWM_POLARITY_INVERTED);
	if (err != 0) {
		LOG_ERR("设置背光 PWM 失败 err=%d", err);
		return err;
	}

	bl_pct = pct;
	return 0;
}

uint8_t kb_backlight_get_percent(void)
{
	return bl_pct;
}

/* ---------------- shell：bl <0-100> / bl get ---------------- */

static int cmd_backlight(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1) {
		shell_print(sh, "背光亮度: %u%%", (unsigned)kb_backlight_get_percent());
		return 0;
	}
	if (argc == 2) {
		unsigned long val;
		int err = 0;

		val = shell_strtoul(argv[1], 10, &err);
		if (err != 0) {
			shell_error(sh, "参数不是数字");
			return -EINVAL;
		}

		if (val > BACKLIGHT_MAX_PCT) {
			shell_error(sh, "亮度需在 0~100 之间");
			return -EINVAL;
		}
		kb_backlight_set_percent((uint8_t)val);
		return 0;
	}

	shell_error(sh, "用法: bl [0-100]");
	return -EINVAL;
}

SHELL_CMD_REGISTER(bl, NULL, "背光亮度: bl [0-100]", cmd_backlight);
