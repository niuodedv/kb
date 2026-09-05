/*
 * IP5306-I2C 电源管理芯片驱动（英集芯定制 I2C 版本）
 *
 * 职责（详见 电源管理.txt）：
 *   (A) 每 10s 向 IP5306 KEY(WAKEUP=P0.22) 发一个 ~80ms 脉冲，阻止其轻载待机
 *       => SYS_POWER(5V) 不掉电，nRF 不意外断电；
 *   (B) I2C 读取充放电 / 充满状态，供电池状态接口。
 *
 * 注意：
 *   - IP5306 寄存器必须 read-modify-write，禁止整字节覆盖（破坏 Reserved 位）。
 *   - KEY 有效电平见设备树 wakeup-gpios 的 ACTIVE 标志（本板默认低有效=按下拉低），
 *     待 P08 实测确认；错误极性可能误触发"1s 内双击 -> 关闭 boost -> 整机断电"。
 */

#include "ip5306.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ip5306, LOG_LEVEL_INF);

#define DT_IP5306 DT_NODELABEL(ip5306)

/* 寄存器（手册 V1.21） */
#define REG_SYS_CTL0 0x00
#define REG_SYS_CTL2 0x02
#define REG_READ0    0x70
#define REG_READ1    0x71

/* SYS_CTL0 位 */
#define SYS_CTL0_BOOST_EN   BIT(5)
#define SYS_CTL0_CHG_EN     BIT(4)
#define SYS_CTL0_BOOST_LOCK BIT(1)  /* 常开，永不轻载关机 */
#define SYS_CTL0_KEY_OFF    BIT(0)

/* REG_READ0 / REG_READ1 位 */
#define READ0_CHARGE_EN BIT(3)  /* 1=充电中 / 0=放电 */
#define READ1_FULL      BIT(3)  /* 1=已充满 */

/* 保活参数（需求 A） */
#define KEEPALIVE_PERIOD_MS 10000U
#define KEEPALIVE_PULSE_MS  80U    /* 30ms < 脉宽 < 2s，落在短按窗口 */

static const struct i2c_dt_spec bus = I2C_DT_SPEC_GET(DT_IP5306);
static const struct gpio_dt_spec wake = GPIO_DT_SPEC_GET(DT_IP5306, wakeup_gpios);

static struct k_work_delayable ka_work;
static bool ka_running;

/* ---- I2C 封装（read-modify-write 保护 Reserved 位）---- */
static int reg_read(uint8_t reg, uint8_t *val)
{
	return i2c_reg_read_byte_dt(&bus, reg, val);
}

static int reg_write(uint8_t reg, uint8_t val)
{
	return i2c_reg_write_byte_dt(&bus, reg, val);
}

static int reg_modify(uint8_t reg, uint8_t mask, uint8_t bits)
{
	uint8_t v;
	int err;

	err = reg_read(reg, &v);
	if (err) {
		return err;
	}
	v = (uint8_t)((v & (uint8_t)~mask) | (bits & mask));
	return reg_write(reg, v);
}

/* ---- 保活脉冲工作项 ---- */
static void ka_work_handler(struct k_work *work)
{
	/* 断言 KEY 有效电平（由设备树 ACTIVE 标志决定），维持一个短按脉宽 */
	gpio_pin_set_dt(&wake, 1);
	k_msleep(KEEPALIVE_PULSE_MS);
	gpio_pin_set_dt(&wake, 0);

	if (ka_running) {
		k_work_schedule(k_work_delayable_from_work(work),
				K_MSEC(KEEPALIVE_PERIOD_MS));
	}
}

int ip5306_init(void)
{
	int err;

	if (!device_is_ready(bus.bus)) {
		LOG_ERR("IP5306 I2C 总线未就绪");
		return -ENODEV;
	}
	if (!device_is_ready(wake.port)) {
		LOG_ERR("IP5306 WAKEUP GPIO 未就绪");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&wake, GPIO_OUTPUT_INACTIVE);
	if (err) {
		LOG_ERR("IP5306 WAKEUP 配置失败: %d", err);
		return err;
	}

	k_work_init_delayable(&ka_work, ka_work_handler);
	ip5306_keepalive_start();

	LOG_INF("IP5306 初始化完成：保活已启动（周期 %u ms，脉宽 %u ms）",
		KEEPALIVE_PERIOD_MS, KEEPALIVE_PULSE_MS);
	return 0;
}

int ip5306_get_charge(bool *charging)
{
	uint8_t v;
	int err;

	if (!charging) {
		return -EINVAL;
	}
	err = reg_read(REG_READ0, &v);
	if (err) {
		return err;
	}
	*charging = (v & READ0_CHARGE_EN) != 0U;
	return 0;
}

int ip5306_is_full(bool *full)
{
	uint8_t v;
	int err;

	if (!full) {
		return -EINVAL;
	}
	err = reg_read(REG_READ1, &v);
	if (err) {
		return err;
	}
	*full = (v & READ1_FULL) != 0U;
	return 0;
}

void ip5306_keepalive_start(void)
{
	ka_running = true;
	/* 立即发一次脉冲，之后每 10s 一次 */
	k_work_schedule(&ka_work, K_NO_WAIT);
}

void ip5306_keepalive_stop(void)
{
	ka_running = false;
	k_work_cancel_delayable(&ka_work);
	gpio_pin_set_dt(&wake, 0);
}

int ip5306_set_boost_always_on(bool on)
{
	return reg_modify(REG_SYS_CTL0, SYS_CTL0_BOOST_LOCK,
			  on ? SYS_CTL0_BOOST_LOCK : 0U);
}
