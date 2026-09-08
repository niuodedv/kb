/*
 * 电池电压采样模块实现（BAT_ADC P0.31/AIN7 + BAT_ADC_EN P0.09）
 *
 * 采样流程（详见 电池电压测量.txt【5】）：
 *   EN 拉 high → 30ms（RC 5ms×5τ 建立稳定）→ 采 8 次平均（丢第 1 次）
 *   → EN 拉 low → 换算 V_BAT = 平均值 × 2（R6=R8=100k 分压 1/2）
 *   → 线性映射 3.3~4.2V → 0~100%（bring-up 粗线性，D03 按实测曲线标定）
 *
 * 线程模型：
 *   - 专用低优先级线程周期执行，EN 之后的 30ms 延时用 k_msleep
 *     （在自己线程里睡，不拖累系统 wq 上的 LED 动画）
 *   - SAADC 与 mode 模块（AIN5）共享外设：Zephyr adc_read 内部有
 *     adc_context 串行锁，并发调用自动排队，无需额外互斥
 */

#include "bat_adc.h"

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(bat_adc, LOG_LEVEL_INF);

/* ---- 采样通道（来自 kb.dts 的 &adc / channel@7，手动构造 adc_dt_spec，
 *      绕开本 SDK ADC_DT_SPEC_GET_BY_IDX 宏展开失败的问题，与 mode.c 一致） */
#define BAT_ADC_NODE DT_NODELABEL(adc)
#define BAT_CH_NODE  DT_CHILD(BAT_ADC_NODE, channel_7)

static const struct adc_dt_spec bat_adc = {
	.dev = DEVICE_DT_GET(BAT_ADC_NODE),
	.channel_id = DT_REG_ADDR(BAT_CH_NODE),
	.channel_cfg_dt_node_exists = 1,
	.channel_cfg = ADC_CHANNEL_CFG_DT(BAT_CH_NODE),
	.vref_mv = 0,
	.resolution = DT_PROP(BAT_CH_NODE, zephyr_resolution),
	.oversampling = 0,
};

/* ---- EN GPIO（自定义 binding kb,bat-adc-en 的 en-gpios）---- */
static const struct gpio_dt_spec bat_en =
	GPIO_DT_SPEC_GET(DT_NODELABEL(bat_en), en_gpios);

/* ---- 可调参数 ---- */
#define VBAT_RATIO     2        /* R6=R8=100k，分压 1/2 */
#define VBAT_MIN_MV    3300     /* 线性映射下限（0%） */
#define VBAT_MAX_MV    4200     /* 线性映射上限（100%） */
#define SAMPLE_COUNT   8        /* 每轮采样次数（丢弃第 1 次） */
#define SETTLE_MS      30       /* EN 导通后的节点建立延时（5τ≈25ms） */
/* 观察期 5 秒一条；平均漏电 = 21µA × 35ms/5s ≈ 0.15µA，可忽略。
 * 稳定后可放宽到 60s（原值）。 */
#define PERIOD_MS      (5U * 1000U)
#define THREAD_DELAY_MS 3000    /* 线程延迟启动，等 main 完成初始化 */

static uint16_t bat_mv;      /* 最近一次电池电压（mV） */
static uint8_t bat_percent;  /* 最近一次电量百分比 */

/* 采样暂停/恢复控制（低功耗档1：屏幕不显示电量时停止 5s 周期唤醒） */
static struct k_sem ctrl;
static volatile bool sampling;

static int sample_once(int32_t *mv)
{
	/* ⚠️ nRF SAADC 每样本 int16_t（mode.c 同款教训：int32 会混入栈残留） */
	int16_t raw_val;
	struct adc_sequence seq = {
		.buffer = &raw_val,
		.buffer_size = sizeof(raw_val),
	};

	adc_sequence_init_dt(&bat_adc, &seq);

	int err = adc_read_dt(&bat_adc, &seq);

	if (err) {
		return err;
	}

	if (raw_val < 0) {
		raw_val = 0;
	}

	int32_t mv_val = raw_val;

	err = adc_raw_to_millivolts_dt(&bat_adc, &mv_val);
	if (err) {
		return err;
	}

	*mv = mv_val;
	return 0;
}

static void measure(void)
{
	int32_t sum = 0;
	int err;

	err = gpio_pin_set_dt(&bat_en, 1);   /* Q1 导通，分压链路接通 */
	if (err) {
		LOG_ERR("BAT_ADC_EN 拉高失败: %d", err);
		return;
	}

	/* 分压节点建立：τ = (R6∥R8)×C29 = 50k×100nF = 5ms，5τ≈25ms */
	k_msleep(SETTLE_MS);

	for (int i = 0; i < SAMPLE_COUNT; i++) {
		int32_t mv;

		err = sample_once(&mv);
		if (err) {
			LOG_ERR("BAT_ADC 采样失败: %d", err);
			(void)gpio_pin_set_dt(&bat_en, 0);
			return;
		}

		if (i > 0) {
			/* 丢弃第 1 次（EN 开关瞬间的尖峰） */
			sum += mv;
		}
	}

	(void)gpio_pin_set_dt(&bat_en, 0);   /* 采样完立即断开，零静态漏电 */

	int32_t avg_mv = sum / (SAMPLE_COUNT - 1);
	int32_t vbat_mv = avg_mv * VBAT_RATIO;

	bat_mv = (uint16_t)CLAMP(vbat_mv, 0, 65535);

	int32_t pct = ((vbat_mv - VBAT_MIN_MV) * 100) /
		      (VBAT_MAX_MV - VBAT_MIN_MV);

	bat_percent = (uint8_t)CLAMP(pct, 0, 100);

	LOG_INF("电池: VBAT=%dmV (%u%%) [ADC=%dmV]", (int)vbat_mv,
		(unsigned int)bat_percent, (int)avg_mv);
}

static void bat_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		if (sampling) {
			measure();
			/* 满速：等满一个周期再测 */
			k_sem_take(&ctrl, K_MSEC(PERIOD_MS));
		} else {
			/* 暂停：挂死等待，恢复(sampling=true)时的 k_sem_give 唤醒后立刻补测一拍 */
			k_sem_take(&ctrl, K_FOREVER);
		}
	}
}

#define BAT_THREAD_STACK_SIZE 1024
#define BAT_THREAD_PRIORITY   7
K_THREAD_DEFINE(bat_thread, BAT_THREAD_STACK_SIZE, bat_thread_fn,
		NULL, NULL, NULL, BAT_THREAD_PRIORITY, 0, THREAD_DELAY_MS);

int kb_bat_adc_init(void)
{
	int err;

	if (!adc_is_ready_dt(&bat_adc)) {
		LOG_ERR("SAADC 设备未就绪");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&bat_en)) {
		LOG_ERR("BAT_ADC_EN GPIO 未就绪");
		return -ENODEV;
	}

	err = adc_channel_setup_dt(&bat_adc);
	if (err) {
		LOG_ERR("ADC 通道配置失败: %d", err);
		return err;
	}

	/* 默认输出低：Q1 关断，分压链路不漏电 */
	err = gpio_pin_configure_dt(&bat_en, GPIO_OUTPUT_INACTIVE);
	if (err) {
		LOG_ERR("BAT_ADC_EN 配置失败: %d", err);
		return err;
	}

	k_sem_init(&ctrl, 0, 1);
	sampling = true;

	LOG_INF("电池电压采样就绪（EN=P0.09 高有效，AIN7，周期 %us，"
		"线程延迟 %dms 启动）",
		(unsigned)(PERIOD_MS / 1000U), THREAD_DELAY_MS);

	return 0;
}

uint16_t kb_bat_adc_get_mv(void)
{
	return bat_mv;
}

uint8_t kb_bat_adc_get_percent(void)
{
	return bat_percent;
}

int kb_bat_adc_set_sampling(bool sampling_en)
{
	sampling = sampling_en;

	if (sampling_en) {
		/* 唤醒采样线程，立即补测一拍（低功耗退出时让电量马上刷新） */
		k_sem_give(&ctrl);
	}

	return 0;
}
