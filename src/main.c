#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "keys/keys.h"
#include "led/led.h"
#include "mode/mode.h"
#include "knob/knob.h"
#include "ble/ble_hid.h"
#include "usb/usb_hid.h"
#include "power/ip5306.h"
#include "power/bat_adc.h"
#include "time/kb_time.h"
#include "lcd/lcd.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

int main(void)
{
	int err;

	LOG_INF("=== 蓝牙小键盘启动（矩阵按键 + LED + 模式检测 + 旋钮 + BLE HID） ===");

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

	/*
	 * BLE HID：注册 HIDS 服务并启动蓝牙协议栈。
	 * 必须在 kb_mode_init() 之后：初始化时要读一次当前模式，
	 * 上电就在 BLE 档的话直接开始广播。
	 */
	err = kb_ble_hid_init();
	if (err) {
		LOG_ERR("BLE HID 模块初始化失败: %d", err);
		return err;
	}

	/*
	 * USB HID：USB 档插入后由主机枚举，按键仅 USB 档发出。
	 * 依赖 kb_mode_init()（读模式门控）与公共 HID 层，无严格顺序要求。
	 */
	err = kb_usb_hid_init();
	if (err) {
		LOG_ERR("USB HID 模块初始化失败: %d", err);
		return err;
	}

	err = ip5306_init();
	if (err) {
		LOG_ERR("电源管理(IP5306)初始化失败: %d", err);
		return err;
	}

	/* 电池电压采样：EN=P0.09、ADC=AIN7，专用线程周期测量（见 电池电压测量.txt） */
	err = kb_bat_adc_init();
	if (err) {
		LOG_ERR("电池电压采样模块初始化失败: %d", err);
		return err;
	}

	/* 软件时钟 + LCD 屏幕（详见 lcd 显示.txt）：
	 * LCD 在最后初始化，保证模式/电源/旋钮等先就绪（其专用线程会再等 150ms）。 */
	err = kb_time_init();
	if (err) {
		LOG_ERR("软件时钟初始化失败: %d", err);
		return err;
	}

	err = kb_lcd_init();
	if (err) {
		LOG_ERR("LCD 屏幕初始化失败: %d", err);
		return err;
	}

	LOG_INF("当前模式: %s", kb_mode_name(kb_mode_get()));
	LOG_INF("等待按键事件...");

	return 0;
}
