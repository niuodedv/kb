#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "keys/keys.h"
#include "led/led.h"
#include "mode/mode.h"
#include "knob/knob.h"
#include "power/ip5306.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

int main(void)
{
	int err;

	LOG_INF("=== 蓝牙小键盘启动（矩阵按键 + 按键 LED + 模式检测） ===");

	err = keys_init();
	if (err) {
		LOG_ERR("按键模块初始化失败: %d", err);
		return err;
	}

	err = kb_led_init();
	if (err) {
		LOG_ERR("按键 LED 模块初始化失败: %d", err);
		return err;
	}

	err = kb_mode_init();
	if (err) {
		LOG_ERR("模式检测模块初始化失败: %d", err);
		return err;
	}

	err = kb_knob_init();
	if (err) {
		LOG_ERR("旋钮模块初始化失败: %d", err);
		return err;
	}

	err = ip5306_init();
	if (err) {
		LOG_ERR("电源管理(IP5306)初始化失败: %d", err);
		return err;
	}

	LOG_INF("当前模式: %s", kb_mode_name(kb_mode_get()));
	LOG_INF("等待按键事件...");

	return 0;
}
