/*
 * 模式检测模块实现（三模开关 ADC 判档）
 *
 * 硬件：3V3 -> R1(10k) -> SW2(三档) -> C11(100nF) -> GND，开关输出接 MODE(P0.29)
 *       P0.29 = 模块脚 8 = AI5 = SAADC AIN5
 *
 * 判档（详见 模式切换.txt 3.1）：
 *       <= 500mV          -> USB
 *       1300mV < V <= 1650mV -> 2.4G
 *       >= 3000mV         -> BLE
 * 两个未定义区间（500~1300、1650~3000）是刻意留的死区，
 * 换档时电压会扫过它们，必须判为 UNKNOWN 并清零候选，
 * 绝不能就近归类，否则换档途中会疯狂误切换。
 *
 * 详见 模式切换.txt
 */

#include "mode.h"

#include <errno.h>
#include <string.h>

#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mode, LOG_LEVEL_INF);

/*
 * 采用 Zephyr 官方 adc_dt 路径（samples/drivers/adc/adc_dt），
 * 用 adc_dt_spec 描述通道（来自 kb.dts 的 &adc / channel@5），
 * 走 adc_read_dt / adc_raw_to_millivolts_dt，由驱动按 DT 配置
 * 处理增益 / 参考 / 分辨率与 raw 对齐，
 * 规避手写 adc_read + 手写 adc_raw_to_millivolts 在 nRF 上把 16 位
 * raw 误当 12 位分辨率、导致满量程饱和读数（~115200mV）使三档
 * 永远判 BLE 的 Bug。
 */
#define MODE_ADC_NODE  DT_NODELABEL(adc)
#define MODE_CH_NODE   DT_CHILD(MODE_ADC_NODE, channel_5)

static const struct adc_dt_spec mode_adc = {
	.dev = DEVICE_DT_GET(MODE_ADC_NODE),
	.channel_id = DT_REG_ADDR(MODE_CH_NODE),
	.channel_cfg_dt_node_exists = 1,
	.channel_cfg = ADC_CHANNEL_CFG_DT(MODE_CH_NODE),
	.vref_mv = 0,
	.resolution = DT_PROP(MODE_CH_NODE, zephyr_resolution),
	.oversampling = 0,
};

/* ---- 判档阈值（mV）---- */
#define MV_USB_MAX  500
#define MV_24G_MIN  1300
#define MV_24G_MAX  1650
#define MV_BLE_MIN  3000

/* ---- 时序参数 ---- */
#define INIT_SAMPLE_MS  10
#define INIT_SAMPLE_N   3
#define POLL_PERIOD_MS  100
#define DEBOUNCE_N      3
/* 心跳周期：每 30 秒打一条存活日志（带 raw 原始值），日志时间戳即可看出系统是何时卡死的 */
#define HEARTBEAT_POLLS (30000U / POLL_PERIOD_MS)

#define CB_SLOTS 4

struct cb_entry {
	kb_mode_cb_t cb;
	void *user_data;
};

static struct cb_entry cbs[CB_SLOTS];

static enum kb_mode cur_mode = KB_MODE_UNKNOWN;
static enum kb_mode override_mode = KB_MODE_UNKNOWN;
static enum kb_mode candidate = KB_MODE_UNKNOWN;
static uint8_t candidate_cnt;
static uint32_t poll_count;

static struct k_mutex lock;
static struct k_work_delayable poll_work;

/*
 * 模式检测跑在【独立工作队列】上，不占用系统工作队列。
 *
 * 原因：poll_handler 里的 adc_read() 是阻塞调用（内部等 SAADC 转换完成，
 * 超时是 K_FOREVER）。一旦 SAADC 卡住不返回，系统工作队列会被整个堵死，
 * 而按键 LED 的动画工作项也跑在系统工作队列上 —— 结果是
 * 「背光与模式切换一起失效」，看起来像同一个故障，实际是互相拖累。
 *
 * 独立队列把阻塞隔离在模式检测内部：
 *   SAADC 卡死  -> 只有模式检测停，LED 动画照常，一眼可辨；
 *   LED 刷新卡死 -> 模式检测照常，同样一眼可辨。
 */
#define MODE_WQ_STACK_SIZE 1024
#define MODE_WQ_PRIORITY   5

K_THREAD_STACK_DEFINE(mode_wq_stack, MODE_WQ_STACK_SIZE);
static struct k_work_q mode_workq;

const char *kb_mode_name(enum kb_mode mode)
{
	switch (mode) {
	case KB_MODE_USB:
		return "USB";
	case KB_MODE_24G:
		return "2.4G";
	case KB_MODE_BLE:
		return "BLE";
	default:
		return "UNKNOWN";
	}
}

static enum kb_mode classify(int32_t mv)
{
	if (mv <= MV_USB_MAX) {
		return KB_MODE_USB;
	}

	if ((mv > MV_24G_MIN) && (mv <= MV_24G_MAX)) {
		return KB_MODE_24G;
	}

	if (mv >= MV_BLE_MIN) {
		return KB_MODE_BLE;
	}

	/* 死区：正在换档 */
	return KB_MODE_UNKNOWN;
}

static int sample_mv(int32_t *mv, int32_t *raw)
{
	/* ⚠️ nRF SAADC 每个采样是 int16_t，驱动每样本只写 2 字节。
	 * 之前这里用 int32_t，高 16 位是栈上未初始化的残留值（实测恒 2），
	 * 导致 raw = 0x20000 + 真实值（如 134823=131072+3751），
	 * mv 恒 ~115200mV 恒判 BLE —— 这就是三挡切换失效的根因！ */
	int16_t raw_val;
	int err;
	struct adc_sequence seq = {
		.buffer = &raw_val,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(raw_val),
	};

	/* 用 DT spec 初始化序列，确保 raw 对齐/分辨率与驱动期望一致 */
	adc_sequence_init_dt(&mode_adc, &seq);

	err = adc_read_dt(&mode_adc, &seq);
	if (err) {
		return err;
	}

	if (raw != NULL) {
		*raw = raw_val;
	}

	if (mv != NULL) {
		int32_t mv_val = raw_val;

		/* 由 spec 自动取增益/参考，得到真实毫伏（满量程 ~3600mV） */
		err = adc_raw_to_millivolts_dt(&mode_adc, &mv_val);
		if (err) {
			return err;
		}
		*mv = mv_val;
	}

	return 0;
}

static void set_mode(enum kb_mode new_mode)
{
	enum kb_mode prev;

	k_mutex_lock(&lock, K_FOREVER);

	prev = cur_mode;
	if (new_mode == prev) {
		k_mutex_unlock(&lock);
		return;
	}

	cur_mode = new_mode;

	/* 每次切换都打日志 */
	LOG_INF("模式切换: %s -> %s", kb_mode_name(prev), kb_mode_name(new_mode));

	for (int i = 0; i < CB_SLOTS; i++) {
		if (cbs[i].cb != NULL) {
			cbs[i].cb(prev, new_mode, cbs[i].user_data);
		}
	}

	k_mutex_unlock(&lock);
}

/* 上电快速定档：连续 INIT_SAMPLE_N 次判到同一档才认账 */
static int initial_scan(void)
{
	enum kb_mode cand = KB_MODE_UNKNOWN;
	int32_t mv = 0;
	int err;

	for (int i = 0; i < INIT_SAMPLE_N; i++) {
		enum kb_mode m;

		err = sample_mv(&mv, NULL);
		if (err) {
			LOG_ERR("MODE 采样失败: %d", err);
			return err;
		}

		m = classify(mv);

		if (i == 0) {
			cand = m;
		} else if (m != cand) {
			cand = KB_MODE_UNKNOWN;
			break;
		}

		if (i < (INIT_SAMPLE_N - 1)) {
			k_msleep(INIT_SAMPLE_MS);
		}
	}

	if (cand != KB_MODE_UNKNOWN) {
		LOG_INF("上电定档: MODE=%d mV -> %s", (int)mv, kb_mode_name(cand));
		set_mode(cand);
	} else {
		LOG_WRN("上电未定档：MODE=%d mV，开关可能停在两档之间", (int)mv);
	}

	return 0;
}

static void poll_handler(struct k_work *work)
{
	int32_t mv;
	int32_t raw;
	int err;
	enum kb_mode m;

	ARG_UNUSED(work);

	if (override_mode != KB_MODE_UNKNOWN) {
		/* 被覆盖：不判档，但保持轮询，解除覆盖后能立刻恢复 */
		k_work_reschedule_for_queue(&mode_workq, &poll_work, K_MSEC(POLL_PERIOD_MS));
		return;
	}

	err = sample_mv(&mv, &raw);
	if (err) {
		LOG_ERR("MODE 采样失败: %d", err);
		k_work_reschedule_for_queue(&mode_workq, &poll_work, K_MSEC(POLL_PERIOD_MS));
		return;
	}

	m = classify(mv);

	if ((m == KB_MODE_UNKNOWN) || (m == cur_mode)) {
		/* 死区（正在换档）或维持原档：清候选 */
		candidate = KB_MODE_UNKNOWN;
		candidate_cnt = 0;
	} else {
		if (m != candidate) {
			candidate = m;
			candidate_cnt = 1;
		} else {
			candidate_cnt++;
		}

		if (candidate_cnt >= DEBOUNCE_N) {
			LOG_INF("MODE=%d mV -> %s", (int)mv, kb_mode_name(candidate));
			set_mode(candidate);
			candidate = KB_MODE_UNKNOWN;
			candidate_cnt = 0;
		}
	}

	if ((++poll_count % HEARTBEAT_POLLS) == 0) {
		LOG_INF("心跳: 模式=%s MODE=%d mV raw=%d（日志停止说明系统已卡死或掉电）",
			kb_mode_name(cur_mode), (int)mv, (int)raw);
	}

	k_work_reschedule_for_queue(&mode_workq, &poll_work, K_MSEC(POLL_PERIOD_MS));
}

enum kb_mode kb_mode_get(void)
{
	/* 单字长读取，无需加锁 */
	return cur_mode;
}

int kb_mode_register_cb(kb_mode_cb_t cb, void *user_data)
{
	if (cb == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&lock, K_FOREVER);

	for (int i = 0; i < CB_SLOTS; i++) {
		if (cbs[i].cb == cb) {
			k_mutex_unlock(&lock);
			return -EALREADY;
		}
	}

	for (int i = 0; i < CB_SLOTS; i++) {
		if (cbs[i].cb == NULL) {
			cbs[i].cb = cb;
			cbs[i].user_data = user_data;
			k_mutex_unlock(&lock);
			return 0;
		}
	}

	k_mutex_unlock(&lock);

	return -ENOMEM;
}

int kb_mode_override(enum kb_mode mode)
{
	if ((mode != KB_MODE_UNKNOWN) && (mode != KB_MODE_USB) &&
	    (mode != KB_MODE_24G) && (mode != KB_MODE_BLE)) {
		return -EINVAL;
	}

	k_mutex_lock(&lock, K_FOREVER);
	override_mode = mode;
	k_mutex_unlock(&lock);

	candidate = KB_MODE_UNKNOWN;
	candidate_cnt = 0;

	if (mode == KB_MODE_UNKNOWN) {
		LOG_INF("解除模式覆盖，恢复 ADC 自动检测");
		k_work_reschedule_for_queue(&mode_workq, &poll_work, K_NO_WAIT);
	} else {
		LOG_INF("模式被覆盖为 %s（停止自动检测）", kb_mode_name(mode));
		set_mode(mode);
	}

	return 0;
}

int kb_mode_read(int32_t *mv, int32_t *raw)
{
	if ((mv == NULL) && (raw == NULL)) {
		return -EINVAL;
	}

	return sample_mv(mv, raw);
}

int kb_mode_init(void)
{
	int err;

	if (!adc_is_ready_dt(&mode_adc)) {
		LOG_ERR("SAADC 设备未就绪");
		return -ENODEV;
	}

	err = k_mutex_init(&lock);
	if (err) {
		LOG_ERR("互斥锁初始化失败: %d", err);
		return err;
	}

	err = adc_channel_setup_dt(&mode_adc);
	if (err) {
		LOG_ERR("ADC 通道配置失败: %d", err);
		return err;
	}

	k_work_queue_start(&mode_workq, mode_wq_stack,
			   K_THREAD_STACK_SIZEOF(mode_wq_stack),
			   MODE_WQ_PRIORITY, NULL);
	k_thread_name_set(&mode_workq.thread, "mode_wq");

	k_work_init_delayable(&poll_work, poll_handler);

	LOG_INF("模式检测就绪: %s 通道%u（内部参考 %umV，增益 1/6，%u 位，满分度 ~%umV）",
		mode_adc.dev->name, (unsigned int)mode_adc.channel_id,
		(unsigned int)adc_ref_internal(mode_adc.dev),
		(unsigned int)mode_adc.resolution,
		(unsigned int)((uint32_t)adc_ref_internal(mode_adc.dev) * 6U));

	err = initial_scan();
	if (err) {
		return err;
	}

	k_work_reschedule_for_queue(&mode_workq, &poll_work, K_MSEC(POLL_PERIOD_MS));

	return 0;
}

/* ================= shell 调试命令 =================
 * 主要用于 模式切换.txt 11.2 的 D02/D03 标定：
 * 三档各量一次 MODE 电压，与万用表对比，确认标称值落在判档区间中心附近。
 */

#if defined(CONFIG_SHELL)

#include <zephyr/shell/shell.h>

static int cmd_mode_info(const struct shell *sh, size_t argc, char **argv)
{
	int32_t mv = 0;
	int32_t raw = 0;
	int err;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	err = kb_mode_read(&mv, &raw);
	if (err) {
		shell_error(sh, "采样失败: %d", err);
		return 0;
	}

	shell_print(sh, "当前模式: %s", kb_mode_name(kb_mode_get()));
	shell_print(sh, "MODE = %d mV (raw %d)", (int)mv, (int)raw);
	shell_print(sh, "判档: <=%d=USB, %d~%d=2.4G, >=%d=BLE（其余为死区）",
		    MV_USB_MAX, MV_24G_MIN, MV_24G_MAX, MV_BLE_MIN);

	return 0;
}

static int cmd_mode_override(const struct shell *sh, size_t argc, char **argv)
{
	enum kb_mode m;
	int err;

	if (strcmp(argv[1], "auto") == 0) {
		m = KB_MODE_UNKNOWN;
	} else if (strcmp(argv[1], "usb") == 0) {
		m = KB_MODE_USB;
	} else if (strcmp(argv[1], "24g") == 0) {
		m = KB_MODE_24G;
	} else if (strcmp(argv[1], "ble") == 0) {
		m = KB_MODE_BLE;
	} else {
		shell_error(sh, "用法: mode override <usb|24g|ble|auto>");
		return 0;
	}

	err = kb_mode_override(m);
	if (err) {
		shell_error(sh, "设置失败: %d", err);
		return 0;
	}

	shell_print(sh, "当前模式: %s", kb_mode_name(kb_mode_get()));

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_mode,
	SHELL_CMD_ARG(info, NULL,
		      "显示当前模式与 MODE 电压",
		      cmd_mode_info, 1, 0),
	SHELL_CMD_ARG(override, NULL,
		      "覆盖模式: mode override <usb|24g|ble|auto>",
		      cmd_mode_override, 2, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(mode, &sub_mode, "模式检测（三模开关）", NULL);

#endif /* CONFIG_SHELL */
