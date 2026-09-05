#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "keys/keys.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

int main(void)
{
	int err;

	LOG_INF("=== 蓝牙小键盘启动（阶段 1：矩阵按键） ===");

	err = keys_init();
	if (err) {
		LOG_ERR("按键模块初始化失败: %d", err);
		return err;
	}

	LOG_INF("等待按键事件...");

	return 0;
}
