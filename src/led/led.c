/*
 * 按键 LED（WS2812）模块实现
 *
 * 行为：按键按下 -> 对应 LED 立即亮；按键抬起 -> 该 LED 渐灭。
 *
 * 线程模型：
 *   - kb_led_press/release 会在 input 子系统线程里被调用，因此它们【只改状态 +
 *     提交延时工作项】就返回，绝不做 SPI 传输（一次刷新约 866us，太慢）。
 *   - 真正的组帧与 SPI 刷新在工作队列里完成。
 *   - leds[] / cfg / dirty 被两个线程共享，用互斥锁保护；
 *     但 SPI 传输放在【解锁之后】执行，避免持锁 866us 把 input 线程卡住。
 *
 * 详见 按键led.txt
 */

#include "led.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "keys/keys.h"

LOG_MODULE_REGISTER(led, LOG_LEVEL_INF);

#define STRIP_NODE DT_ALIAS(led_strip)
#define VCC_NODE   DT_NODELABEL(rgb_vcc)

BUILD_ASSERT(DT_NODE_HAS_STATUS(STRIP_NODE, okay),
	     "未找到灯带节点：请检查设备树别名 led-strip 是否指向 ws2812 节点");
BUILD_ASSERT(DT_NODE_HAS_STATUS(VCC_NODE, okay),
	     "未找到灯带供电节点 rgb-vcc");

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);
static const struct device *const vcc   = DEVICE_DT_GET(VCC_NODE);

/* ---- 可调参数 ---- */
#define ANIM_PERIOD_MS     16    /* 渐灭动画节拍，约 60fps */
#define LED_PWR_STABLE_MS  1     /* 灯带 VCC 上电后的稳定等待 */
#define LED_CH_MA          20    /* WS2812 单个颜色通道满亮度电流(mA) */
#define LED_BUDGET_MA      200   /* 全灯链总电流预算(mA)，超出则整体按比例限幅 */
#define FADE_MS_MAX        5000

/* ---- 每颗灯的状态 ---- */
enum {
	LED_ST_IDLE = 0,   /* 熄灭，不参与刷新 */
	LED_ST_HELD,       /* 按住中，维持目标亮度 */
	LED_ST_FADE,       /* 松手后渐灭中 */
};

struct led_state {
	uint8_t level;   /* 当前亮度 0~255（相对目标颜色的比例） */
	uint8_t state;
};

/*
 * (行,列) -> 灯链序号
 *
 * 链路顺序：行优先（ROW0 -> ROW5），行内按列升序；
 * 无灯位置（6 个空位 + (0,3) EC11 旋钮按键）填 KB_LED_NONE。
 *
 * ⚠️ 这张表是按「行优先」推导出来的假设值，必须用 按键led.txt 13.2 的 L02 用例
 *    实测核对（逐个链路号点亮，看物理上是哪个键在亮）。
 *    若实际链路顺序不同（例如蛇形走线），【只改这张表】，不动任何逻辑。
 */
static const uint8_t led_index[KB_ROWS][KB_COLS] = {
	/*          COL0        COL1        COL2        COL3 */
	/* ROW0 */ { KB_LED_NONE, KB_LED_NONE, KB_LED_NONE, KB_LED_NONE }, /* (0,3)=EC11 无灯 */
	/* ROW1 */ { 0,           1,           2,           3          },
	/* ROW2 */ { 4,           5,           6,           KB_LED_NONE },
	/* ROW3 */ { 7,           8,           9,           10         },
	/* ROW4 */ { 11,          12,          13,          KB_LED_NONE },
	/* ROW5 */ { 14,          15,          KB_LED_NONE, 16         },
};

static struct led_state leds[KB_LED_COUNT];

/* 默认：纯绿、满亮度、按下亮/松手渐灭、渐灭 250ms */
static struct kb_led_config cfg = {
	.r = 0x66,
	.g = 0xcc,
	.b = 0xff,
	.brightness = 128,
	.effect = KB_LED_EFFECT_DEFAULT,
	.fade_ms = 250,
};

static struct k_mutex lock;
static bool dirty;
static bool powered;   /* 仅在工作队列上下文访问，无需加锁 */
static struct k_work_delayable anim_dwork;

static uint8_t led_index_of(uint8_t row, uint8_t col)
{
	if ((row >= KB_ROWS) || (col >= KB_COLS)) {
		return KB_LED_NONE;
	}

	return led_index[row][col];
}

/*
 * 组帧 + 总电流限幅。必须在持锁状态下调用。
 *
 * 单颗电流 ≈ (r+g+b)/255 * 20mA * level/255。
 * 17 颗全白可达 1A 以上，板载 LDO 扛不住，所以超预算时整体按比例压低。
 */
static void build_frame(struct led_rgb *px)
{
	uint8_t br, bg, bb;
	uint32_t level_sum = 0;
	uint32_t scale256 = 256;

	br = (uint16_t)cfg.r * cfg.brightness / 255U;
	bg = (uint16_t)cfg.g * cfg.brightness / 255U;
	bb = (uint16_t)cfg.b * cfg.brightness / 255U;

	for (int i = 0; i < KB_LED_COUNT; i++) {
		level_sum += leds[i].level;
	}

	if ((level_sum > 0) && ((br + bg + bb) > 0)) {
		uint32_t ma = level_sum * (br + bg + bb) * LED_CH_MA / 65025U;

		if (ma > LED_BUDGET_MA) {
			scale256 = LED_BUDGET_MA * 256U / ma;
		}
	}

	for (int i = 0; i < KB_LED_COUNT; i++) {
		uint32_t v = (uint32_t)leds[i].level * scale256 / 256U;

		px[i].r = (uint16_t)br * v / 255U;
		px[i].g = (uint16_t)bg * v / 255U;
		px[i].b = (uint16_t)bb * v / 255U;
	}
}

static int led_power_on(void)
{
	int err;

	if (powered) {
		return 0;
	}

	err = regulator_enable(vcc);
	if (err) {
		LOG_ERR("灯带供电使能失败: %d", err);
		return err;
	}

	/* 等灯带 VCC 稳定后再送数据，否则首帧可能丢失或乱码 */
	k_msleep(LED_PWR_STABLE_MS);
	powered = true;

	return 0;
}

static int led_power_off(void)
{
	struct led_rgb px[KB_LED_COUNT];
	int err;

	if (!powered) {
		return 0;
	}

	/*
	 * 先发全 0 帧：既熄灭所有灯，也让 MOSI 停在低电平。
	 * 若先断电、而 DIN 仍为高电平，电流会经 WS2812 输入端的 ESD 二极管
	 * 倒灌进 VDD，造成幻象供电（表现为关不掉 / 微亮 / 状态错乱）。
	 * 顺序不能反。
	 */
	memset(px, 0, sizeof(px));
	err = led_strip_update_rgb(strip, px, KB_LED_COUNT);
	if (err) {
		LOG_WRN("发送熄灯帧失败: %d", err);
	}

	err = regulator_disable(vcc);
	if (err) {
		LOG_ERR("灯带供电关闭失败: %d", err);
		return err;
	}

	powered = false;

	return 0;
}

static void anim_handler(struct k_work *work)
{
	struct led_rgb px[KB_LED_COUNT];
	bool changed;
	bool fading = false;
	bool active = false;
	uint16_t step;

	ARG_UNUSED(work);

	k_mutex_lock(&lock, K_FOREVER);

	step = (uint16_t)((255U * ANIM_PERIOD_MS) / cfg.fade_ms);
	if (step == 0) {
		step = 1;
	}

	changed = dirty;
	dirty = false;

	for (int i = 0; i < KB_LED_COUNT; i++) {
		switch (leds[i].state) {
		case LED_ST_HELD:
			if (leds[i].level != 255) {
				leds[i].level = 255;
				changed = true;
			}
			active = true;
			break;

		case LED_ST_FADE:
			if (leds[i].level > step) {
				leds[i].level -= step;
				changed = true;
				fading = true;
				active = true;
			} else {
				if (leds[i].level != 0) {
					changed = true;
				}
				leds[i].level = 0;
				leds[i].state = LED_ST_IDLE;
			}
			break;

		default:
			break;
		}
	}

	if (cfg.effect == KB_LED_EFFECT_OFF) {
		active = false;
		memset(px, 0, sizeof(px));
	} else {
		build_frame(px);
	}

	k_mutex_unlock(&lock);

	if (changed) {
		if (active) {
			/* 上电 -> 送数，顺序不能反 */
			if (led_power_on() == 0) {
				int err = led_strip_update_rgb(strip, px, KB_LED_COUNT);

				if (err) {
					LOG_WRN("灯带刷新失败: %d", err);
				}
			}
		} else {
			/* 全部熄灭：发全 0 帧后切断灯带供电 */
			(void)led_power_off();
		}
	}

	/*
	 * 只有「正在渐灭」才需要周期性节拍。
	 * 单纯按住（HELD）时画面是静止的，WS2812 会自行锁存，不必重复刷新；
	 * 松手时 kb_led_release 会再次提交工作项把动画拉起来。
	 */
	if (fading) {
		k_work_reschedule(&anim_dwork, K_MSEC(ANIM_PERIOD_MS));
	}
}

int kb_led_press(uint8_t row, uint8_t col)
{
	uint8_t idx = led_index_of(row, col);

	if (idx == KB_LED_NONE) {
		/* EC11 旋钮按键等无灯键位：静默忽略 */
		return 0;
	}

	k_mutex_lock(&lock, K_FOREVER);
	leds[idx].state = LED_ST_HELD;
	leds[idx].level = 255;
	dirty = true;
	k_mutex_unlock(&lock);

	k_work_reschedule(&anim_dwork, K_NO_WAIT);

	return 0;
}

int kb_led_release(uint8_t row, uint8_t col)
{
	uint8_t idx = led_index_of(row, col);

	if (idx == KB_LED_NONE) {
		return 0;
	}

	k_mutex_lock(&lock, K_FOREVER);
	if (leds[idx].state == LED_ST_HELD) {
		leds[idx].state = LED_ST_FADE;
		dirty = true;
	}
	k_mutex_unlock(&lock);

	k_work_reschedule(&anim_dwork, K_NO_WAIT);

	return 0;
}

void kb_led_all_off(void)
{
	k_mutex_lock(&lock, K_FOREVER);
	for (int i = 0; i < KB_LED_COUNT; i++) {
		leds[i].state = LED_ST_IDLE;
		leds[i].level = 0;
	}
	dirty = true;
	k_mutex_unlock(&lock);

	k_work_reschedule(&anim_dwork, K_NO_WAIT);
}

int kb_led_set_color(uint8_t r, uint8_t g, uint8_t b)
{
	k_mutex_lock(&lock, K_FOREVER);
	cfg.r = r;
	cfg.g = g;
	cfg.b = b;
	dirty = true;
	k_mutex_unlock(&lock);

	k_work_reschedule(&anim_dwork, K_NO_WAIT);

	return 0;
}

int kb_led_set_brightness(uint8_t level)
{
	k_mutex_lock(&lock, K_FOREVER);
	cfg.brightness = level;
	dirty = true;
	k_mutex_unlock(&lock);

	k_work_reschedule(&anim_dwork, K_NO_WAIT);

	return 0;
}

int kb_led_set_effect(uint8_t effect)
{
	if (effect == KB_LED_EFFECT_BREATH) {
		return -ENOTSUP;   /* 预留，暂未实现 */
	}

	if ((effect != KB_LED_EFFECT_DEFAULT) && (effect != KB_LED_EFFECT_OFF)) {
		return -EINVAL;
	}

	k_mutex_lock(&lock, K_FOREVER);
	cfg.effect = effect;
	dirty = true;
	k_mutex_unlock(&lock);

	k_work_reschedule(&anim_dwork, K_NO_WAIT);

	return 0;
}

int kb_led_set_fade_ms(uint16_t ms)
{
	if ((ms < ANIM_PERIOD_MS) || (ms > FADE_MS_MAX)) {
		return -EINVAL;
	}

	k_mutex_lock(&lock, K_FOREVER);
	cfg.fade_ms = ms;
	k_mutex_unlock(&lock);

	return 0;
}

int kb_led_get_config(struct kb_led_config *out)
{
	if (out == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&lock, K_FOREVER);
	*out = cfg;
	k_mutex_unlock(&lock);

	return 0;
}

int kb_led_init(void)
{
	size_t len;
	int err;

	if (!device_is_ready(strip)) {
		LOG_ERR("灯带设备未就绪: %s", strip->name);
		return -ENODEV;
	}

	if (!device_is_ready(vcc)) {
		LOG_ERR("灯带供电 regulator 未就绪: %s", vcc->name);
		return -ENODEV;
	}

	len = led_strip_length(strip);
	if (len != KB_LED_COUNT) {
		LOG_ERR("灯链长度 %u 与 KB_LED_COUNT(%d) 不一致，请核对设备树 chain-length",
			(unsigned int)len, KB_LED_COUNT);
		return -EINVAL;
	}

	err = k_mutex_init(&lock);
	if (err) {
		LOG_ERR("互斥锁初始化失败: %d", err);
		return err;
	}

	k_work_init_delayable(&anim_dwork, anim_handler);

	/* 确保在断电态起步（设备树已配 regulator-boot-off，这里再兜一次底） */
	(void)regulator_disable(vcc);
	powered = false;

	LOG_INF("灯带就绪: %s，共 %d 颗（EC11 旋钮按键无灯）",
		strip->name, KB_LED_COUNT);

	return 0;
}

/* ================= 上位机协议帧（预留，传输层后续对接） ================= */

#define FRAME_SOF 0xA5

enum {
	CMD_ACK           = 0x00,
	CMD_SET_COLOR_ALL = 0x01,
	CMD_SET_COLOR_KEY = 0x02,
	CMD_SET_BRIGHT    = 0x03,
	CMD_SET_EFFECT    = 0x04,
	CMD_SET_FADE_MS   = 0x05,
	CMD_SAVE          = 0x06,
	CMD_RESET         = 0x07,
	CMD_GET_CONFIG    = 0x81,
	CMD_NACK          = 0xFF,
};

static uint8_t frame_xor(const uint8_t *p, uint16_t len)
{
	uint8_t v = 0;

	for (uint16_t i = 0; i < len; i++) {
		v ^= p[i];
	}

	return v;
}

static void frame_build(uint8_t *buf, uint8_t cmd, const uint8_t *payload, uint8_t len)
{
	buf[0] = FRAME_SOF;
	buf[1] = cmd;
	buf[2] = len;
	if (len > 0) {
		memcpy(&buf[3], payload, len);
	}
	buf[3 + len] = frame_xor(buf, (uint16_t)(3 + len));
}

static int reply_err(uint8_t *resp, uint16_t *resp_len, int err)
{
	uint8_t code = (uint8_t)((unsigned int)(-err) & 0xFFU);

	frame_build(resp, CMD_NACK, &code, 1);
	*resp_len = 4;

	return 0;
}

int kb_led_host_cmd(const uint8_t *frame, uint16_t len,
		    uint8_t *resp, uint16_t *resp_len)
{
	uint8_t data[6];
	uint8_t dlen = 0;
	uint8_t cmd;
	uint8_t plen;
	const uint8_t *p;
	int err = 0;

	if ((resp == NULL) || (resp_len == NULL)) {
		return -EINVAL;
	}

	if ((frame == NULL) || (len < 4)) {
		return reply_err(resp, resp_len, -EBADMSG);
	}

	if (frame[0] != FRAME_SOF) {
		return reply_err(resp, resp_len, -EBADMSG);
	}

	cmd = frame[1];
	plen = frame[2];

	if ((uint16_t)(plen + 4U) != len) {
		return reply_err(resp, resp_len, -EBADMSG);
	}

	if (frame_xor(frame, (uint16_t)(len - 1)) != frame[len - 1]) {
		return reply_err(resp, resp_len, -EBADMSG);
	}

	p = &frame[3];

	switch (cmd) {
	case CMD_SET_COLOR_ALL:
		if (plen != 3) {
			err = -EBADMSG;
			break;
		}
		err = kb_led_set_color(p[0], p[1], p[2]);
		break;

	case CMD_SET_BRIGHT:
		if (plen != 1) {
			err = -EBADMSG;
			break;
		}
		err = kb_led_set_brightness(p[0]);
		break;

	case CMD_SET_EFFECT:
		if (plen != 1) {
			err = -EBADMSG;
			break;
		}
		err = kb_led_set_effect(p[0]);
		break;

	case CMD_SET_FADE_MS:
		if (plen != 2) {
			err = -EBADMSG;
			break;
		}
		err = kb_led_set_fade_ms((uint16_t)(((uint16_t)p[0] << 8) | p[1]));
		break;

	case CMD_GET_CONFIG:
		if (plen != 0) {
			err = -EBADMSG;
			break;
		}
		{
			struct kb_led_config c;

			err = kb_led_get_config(&c);
			if (err) {
				break;
			}
			data[0] = c.r;
			data[1] = c.g;
			data[2] = c.b;
			data[3] = c.brightness;
			data[4] = c.effect;
			dlen = 5;
		}
		break;

	case CMD_SET_COLOR_KEY:   /* 单键配色：预留 */
	case CMD_SAVE:            /* 持久化到 NVS：预留 */
	case CMD_RESET:           /* 恢复默认：预留 */
		err = -ENOTSUP;
		break;

	default:
		err = -ENOTSUP;
		break;
	}

	if (err) {
		return reply_err(resp, resp_len, err);
	}

	frame_build(resp, CMD_ACK, data, dlen);
	*resp_len = (uint16_t)(4 + dlen);

	return 0;
}

/* ================= shell 调试命令 =================
 * 主要用于 按键led.txt 13.2 的 L02 用例：逐个链路号点亮，核对物理灯序。
 * 灯序是「行优先」的推导值，必须实测确认；若不符，改 led_index[] 表即可。
 */

#if defined(CONFIG_SHELL)

#include <stdlib.h>

#include <zephyr/shell/shell.h>

static void test_light_one(int idx)
{
	k_mutex_lock(&lock, K_FOREVER);
	for (int i = 0; i < KB_LED_COUNT; i++) {
		leds[i].state = LED_ST_IDLE;
		leds[i].level = 0;
	}
	if (idx >= 0) {
		leds[idx].state = LED_ST_HELD;
		leds[idx].level = 255;
	}
	dirty = true;
	k_mutex_unlock(&lock);

	k_work_reschedule(&anim_dwork, K_NO_WAIT);
}

static int cmd_led_test(const struct shell *sh, size_t argc, char **argv)
{
	char *end;
	unsigned long idx;

	if (strcmp(argv[1], "off") == 0) {
		test_light_one(-1);
		shell_print(sh, "全部熄灭");
		return 0;
	}

	idx = strtoul(argv[1], &end, 10);
	if ((*end != '\0') || (idx >= KB_LED_COUNT)) {
		shell_error(sh, "用法: led test <0~%d | off>", KB_LED_COUNT - 1);
		return 0;
	}

	test_light_one((int)idx);
	shell_print(sh, "点亮链路号 %u —— 请核对物理上是哪个键在亮",
		    (unsigned int)idx);
	return 0;
}

static int cmd_led_color(const struct shell *sh, size_t argc, char **argv)
{
	uint8_t r, g, b;
	int err;

	r = (uint8_t)strtoul(argv[1], NULL, 10);
	g = (uint8_t)strtoul(argv[2], NULL, 10);
	b = (uint8_t)strtoul(argv[3], NULL, 10);

	err = kb_led_set_color(r, g, b);
	if (err) {
		shell_error(sh, "设置颜色失败: %d", err);
		return 0;
	}

	shell_print(sh, "全局颜色 -> R%u G%u B%u（默认 0 255 0 纯绿）", r, g, b);
	return 0;
}

static int cmd_led_info(const struct shell *sh, size_t argc, char **argv)
{
	struct kb_led_config c;
	int err;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	err = kb_led_get_config(&c);
	if (err) {
		shell_error(sh, "读取配置失败: %d", err);
		return 0;
	}

	shell_print(sh, "颜色 R%u G%u B%u | 亮度 %u | 灯效 %u | 渐灭 %ums",
		    c.r, c.g, c.b, c.brightness, c.effect, c.fade_ms);
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_led,
	SHELL_CMD_ARG(test, NULL,
		      "点亮指定链路号以核对灯序: led test <0~16|off>",
		      cmd_led_test, 2, 0),
	SHELL_CMD_ARG(color, NULL,
		      "设置全局颜色: led color <R> <G> <B>",
		      cmd_led_color, 4, 0),
	SHELL_CMD_ARG(info, NULL,
		      "显示当前 LED 配置",
		      cmd_led_info, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(led, &sub_led, "按键 LED 控制", NULL);

#endif /* CONFIG_SHELL */
