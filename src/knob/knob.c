/*
 * 旋钮编码器（EC11）模块实现
 *
 * 原理：A(P0.10) / B(P1.06) 两相正交方波，相差 90°。
 *   每次引脚跳变（双边沿中断）读两路电平 -> 合成 2bit 状态 ->
 *   与「上一状态」查 QEM 表得到本次跳变增量（+1 正转 / -1 反转 / 0 非法两比特跳变）。
 *   每累计到 ±2 步（=1 档 detent）产生一次档事件，方向取净符号。
 *
 * 计数粒度：30 档 / 15 脉冲(PPR) => 每档 12°、每步 6°（4×PPR=60 计数/圈）。
 *   角度 = 累计步数 × 6（整数运算，免浮点）。
 *
 * 线程模型：
 *   - GPIO 回调在 ISR 上下文：只做 读两引脚 + 查表 + 原子累加 + 提交工作项，
 *     不调用任何阻塞 API（参照 mode.c / led.c 的「ISR 极短」约定）。
 *   - 真正的日志与订阅者回调放在【系统工作队列】里执行，与 LED 动画互不拖累。
 *
 * 引脚：A=P0.10(gpio0)、B=P1.06(gpio1)，固定硬连线，直接引用端口+引脚。
 *   之所以不另建 dts 节点用 GPIO_DT_SPEC_GET：裸节点缺 binding 时该宏无法生成
 *   pin 宏；固定板卡写死端口+引脚最稳妥（见 旋钮编码器.txt 第二节）。
 *
 * 详见 旋钮编码器.txt
 */

#include "knob.h"

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/shell/shell.h>

LOG_MODULE_REGISTER(knob, LOG_LEVEL_INF);

/* 调试开关：上板定位「旋转无打印」时打开，稳定后删除本段与 KNOB_DIAG 相关代码 */
#define KNOB_DIAG 1

/* ---- A/B 两相 GPIO（固定引脚，板级硬连线）----
 * EC11 A=P0.10(gpio0.10)、B=P1.06(gpio1.6)，均为对公共端 C(接地) 通断的触点，
 * 需上拉：默认 nRF 内部上拉；若原理图已有外部上拉电阻，把 GPIO_PULL_UP 去掉即可。
 */
#define KNOB_A_PORT DT_NODELABEL(gpio0)
#define KNOB_B_PORT DT_NODELABEL(gpio1)
static const struct device *a_port = DEVICE_DT_GET(KNOB_A_PORT);
static const struct device *b_port = DEVICE_DT_GET(KNOB_B_PORT);
#define KNOB_A_PIN 10
#define KNOB_B_PIN 6

/* ---- 正交查表：idx = (old<<2)|new，值 = 本次跳变增量 ---- */
static const int8_t qem_table[16] = {
	 0, -1, +1,  0,   /* 00->00, 00->01, 00->10, 00->11 */
	+1,  0,  0, -1,   /* 01->00, 01->01, 01->10, 01->11 */
	-1,  0,  0, +1,   /* 10->00, 10->01, 10->10, 10->11 */
	 0, +1, -1,  0,   /* 11->00, 11->01, 11->10, 11->11 */
};

/* ---- 参数 ---- */
#define STEPS_PER_DETENT 2      /* 1 档 = 2 步 */
#define DEG_PER_STEP     6      /* 1 步 = 6°（360 / (4*15)）*/
#define CB_SLOTS         4

struct cb_entry {
	kb_knob_cb_t cb;
	void *user_data;
};

static struct cb_entry cbs[CB_SLOTS];

/* ISR 上下文更新的量（total_steps 用原子，其余仅在 ISR 内改写）*/
static atomic_t total_steps = ATOMIC_INIT(0);
static uint8_t old_state;       /* 上一时刻 (A<<1)|B */
static int16_t detent_acc;      /* 当前档内累计步数，达到 ±2 触发一次档事件 */

/* 档事件暂存（供工作项读取）*/
static enum kb_knob_dir ev_dir;
static int16_t ev_step;
static int32_t ev_total;
static int32_t ev_angle;

static struct k_spinlock lock;       /* 保护 old_state / detent_acc */
static struct k_mutex cb_lock;       /* 保护 cbs[]（注册在初始化线程，回调在系统队列）*/

static struct gpio_callback a_cb;
static struct gpio_callback b_cb;
static struct k_work detent_work;

#ifdef KNOB_DIAG
static atomic_t g_transitions = ATOMIC_INIT(0);   /* ISR 累计的引脚跳变次数 */
static struct k_work_delayable diag_work;

/* 每 3 秒打印一次原始 A/B 电平与跳变数，用于判断「引脚到底有没有在动」 */
static void diag_work_handler(struct k_work *work)
{
	static int last_tr;
	int a = gpio_pin_get(a_port, KNOB_A_PIN);
	int b = gpio_pin_get(b_port, KNOB_B_PIN);
	int tr = (int)atomic_get(&g_transitions);

	LOG_INF("旋钮诊断: A=%d B=%d 累计步=%d 角度=%d 旧态=%d 近3s跳变=%d",
		a, b, kb_knob_get_steps(), kb_knob_get_angle(), old_state, tr - last_tr);
	last_tr = tr;

	k_work_schedule(k_work_delayable_from_work(work), K_SECONDS(3));
}
#endif

/* 上板手动查询：在 RTT shell 里输入 knob 即可看到当前 A/B 原始电平 */
static int cmd_knob(const struct shell *sh, size_t argc, char **argv)
{
	int a = gpio_pin_get(a_port, KNOB_A_PIN);
	int b = gpio_pin_get(b_port, KNOB_B_PIN);

	shell_print(sh, "knob: A=%d B=%d 累计步=%d 角度=%d 旧态=%d",
		    a, b, kb_knob_get_steps(), kb_knob_get_angle(), old_state);
	return 0;
}
SHELL_CMD_REGISTER(knob, NULL, "EC11 knob raw status", cmd_knob);

const char *kb_knob_dir_name(enum kb_knob_dir dir)
{
	switch (dir) {
	case KB_KNOB_CW:
		return "CW(正转)";
	case KB_KNOB_CCW:
		return "CCW(反转)";
	default:
		return "NONE";
	}
}

int32_t kb_knob_get_steps(void)
{
	return (int32_t)atomic_get(&total_steps);
}

int32_t kb_knob_get_angle(void)
{
	return (int32_t)atomic_get(&total_steps) * DEG_PER_STEP;
}

void kb_knob_reset(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	atomic_set(&total_steps, 0);
	detent_acc = 0;
	k_spin_unlock(&lock, key);
}

/*
 * 在 ISR 内调用：处理一次引脚跳变。
 * 读两路当前电平（物理电平，内部上拉时空闲=高），合成新状态，查表得增量并累加；
 * 达到一档则记录事件并提交工作项。
 */
static void knob_isr_handler(void)
{
	int a = gpio_pin_get(a_port, KNOB_A_PIN);
	int b = gpio_pin_get(b_port, KNOB_B_PIN);
	uint8_t new_state;
	int8_t delta;
	bool fire = false;

	if ((a < 0) || (b < 0)) {
		return;  /* 读数异常，忽略本次 */
	}

	atomic_inc(&g_transitions);

	new_state = (uint8_t)(((a != 0) << 1) | (b != 0));

	k_spinlock_key_t key = k_spin_lock(&lock);

	delta = qem_table[((uint8_t)(old_state << 2)) | new_state];
	old_state = new_state;

	if (delta != 0) {
		atomic_add(&total_steps, delta);
		detent_acc += delta;

		if ((detent_acc >= STEPS_PER_DETENT) ||
		    (detent_acc <= -STEPS_PER_DETENT)) {
			/* 凑满一档：方向取净符号，step 记 ±2 */
			ev_dir   = (detent_acc > 0) ? KB_KNOB_CW : KB_KNOB_CCW;
			ev_step  = (int16_t)(detent_acc > 0 ? STEPS_PER_DETENT
							 : -STEPS_PER_DETENT);
			ev_total = (int32_t)atomic_get(&total_steps);
			ev_angle = ev_total * DEG_PER_STEP;
			detent_acc = 0;  /* 余数为 0（一档恰好 2 步）*/
			fire = true;
		}
	}

	k_spin_unlock(&lock, key);

	if (fire) {
		k_work_submit(&detent_work);
	}
}

/* A 相端口中断入口 */
static void knob_a_cb(const struct device *port, struct gpio_callback *cb, uint32_t pin)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pin);
	knob_isr_handler();
}

/* B 相端口中断入口 */
static void knob_b_cb(const struct device *port, struct gpio_callback *cb, uint32_t pin)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pin);
	knob_isr_handler();
}

/* 工作项：在系统工作队列里打印日志 + 派发订阅者（不阻塞 ISR）*/
static void detent_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	LOG_INF("旋钮 方向=%s 本次=%d° 累计步=%d 累计角=%d°",
		kb_knob_dir_name(ev_dir),
		(int)ev_step * DEG_PER_STEP, ev_total, ev_angle);

	k_mutex_lock(&cb_lock, K_FOREVER);
	for (int i = 0; i < CB_SLOTS; i++) {
		if (cbs[i].cb != NULL) {
			cbs[i].cb(ev_dir, ev_step, ev_total, ev_angle, cbs[i].user_data);
		}
	}
	k_mutex_unlock(&cb_lock);
}

int kb_knob_register_cb(kb_knob_cb_t cb, void *user_data)
{
	if (cb == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&cb_lock, K_FOREVER);

	for (int i = 0; i < CB_SLOTS; i++) {
		if (cbs[i].cb == cb) {
			k_mutex_unlock(&cb_lock);
			return -EALREADY;
		}
	}

	for (int i = 0; i < CB_SLOTS; i++) {
		if (cbs[i].cb == NULL) {
			cbs[i].cb = cb;
			cbs[i].user_data = user_data;
			k_mutex_unlock(&cb_lock);
			return 0;
		}
	}

	k_mutex_unlock(&cb_lock);
	return -ENOMEM;
}

int kb_knob_init(void)
{
	int err;
	int a, b;

	if (!device_is_ready(a_port)) {
		LOG_ERR("旋钮 A 相 GPIO 未就绪: %s", a_port->name);
		return -ENODEV;
	}
	if (!device_is_ready(b_port)) {
		LOG_ERR("旋钮 B 相 GPIO 未就绪: %s", b_port->name);
		return -ENODEV;
	}

	/* 配置为输入 + 内部上拉 + 双边沿中断 */
	err = gpio_pin_configure(a_port, KNOB_A_PIN, GPIO_INPUT | GPIO_PULL_UP);
	if (err) {
		LOG_ERR("旋钮 A 相配置失败: %d", err);
		return err;
	}
	err = gpio_pin_configure(b_port, KNOB_B_PIN, GPIO_INPUT | GPIO_PULL_UP);
	if (err) {
		LOG_ERR("旋钮 B 相配置失败: %d", err);
		return err;
	}

	err = gpio_pin_interrupt_configure(a_port, KNOB_A_PIN, GPIO_INT_EDGE_BOTH);
	if (err) {
		LOG_ERR("旋钮 A 相中断配置失败: %d", err);
		return err;
	}
	err = gpio_pin_interrupt_configure(b_port, KNOB_B_PIN, GPIO_INT_EDGE_BOTH);
	if (err) {
		LOG_ERR("旋钮 B 相中断配置失败: %d", err);
		return err;
	}

	/* 上电定初态：先读一次 A/B 作为 old_state 基准 */
	a = gpio_pin_get(a_port, KNOB_A_PIN);
	b = gpio_pin_get(b_port, KNOB_B_PIN);
	if ((a < 0) || (b < 0)) {
		LOG_ERR("旋钮初态读取失败: a=%d b=%d", a, b);
		return -EIO;
	}
	old_state = (uint8_t)(((a != 0) << 1) | (b != 0));

	k_mutex_init(&cb_lock);
	k_work_init(&detent_work, detent_work_handler);

#ifdef KNOB_DIAG
	k_work_init_delayable(&diag_work, diag_work_handler);
	k_work_schedule(&diag_work, K_NO_WAIT);
#endif

	gpio_init_callback(&a_cb, knob_a_cb, BIT(KNOB_A_PIN));
	gpio_init_callback(&b_cb, knob_b_cb, BIT(KNOB_B_PIN));
	err = gpio_add_callback(a_port, &a_cb);
	if (err) {
		LOG_ERR("旋钮 A 相加回调失败: %d", err);
		return err;
	}
	err = gpio_add_callback(b_port, &b_cb);
	if (err) {
		LOG_ERR("旋钮 B 相加回调失败: %d", err);
		return err;
	}

	LOG_INF("旋钮编码器就绪: A=%s.%u B=%s.%u（每档 12°，每步 6°）",
		a_port->name, KNOB_A_PIN, b_port->name, KNOB_B_PIN);

	return 0;
}
