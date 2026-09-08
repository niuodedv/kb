/*
 * 低功耗（两级空闲降载 / 断电停机）实现 —— 详见 低功耗.txt
 *
 * 计时策略：
 *   - 任意“用户活动”（按键 keymap 事件 / 旋钮旋转 / 模式档位变化）
 *     都会把 1 分钟(档1)与 5 分钟(档2)计时器从“此刻”重新排；
 *   - 档1(IDLE)：LCD 停刷、背光降到 PM_IDLE_BRIGHT、电池采样暂停、
 *     mode 轮询放慢到 1s；IP5306 保活脉冲【继续】跑（每 10s），
 *     这是需求里 1~5 分钟窗口的硬要求，绝不能停；
 *   - 档2(POWER OFF)：先放行全键释放，再关灯/屏，然后停 IP5306 保活，
 *     最后 sys_poweroff() 进 CPU System Off —— IP5306 失去保活后会在
 *     轻载关机时间内自动待机切断 3V3，整机断电，用户用电源开关重启。
 *
 * 充电豁免：板上无 VBUS 检测，本模块每 2s 读一次 IP5306 充电状态：
 *   充电 -> 取消计时并退出档1；充电结束 -> 从零重新计时。
 *
 * 平台约束（nRF52840 无运行时变频）：所谓“降频”实际落地为“空闲降载”，
 * CPU 主频固定 64MHz，档1 只减少外设/轮询活动，平均功耗因此下降。
 */

#include "pm.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/poweroff.h>

#include "knob/knob.h"
#include "mode/mode.h"
#include "lcd/lcd.h"
#include "lcd/backlight.h"
#include "led/led.h"
#include "ble/ble_hid.h"
#include "usb/usb_hid.h"
#include "power/bat_adc.h"
#include "power/ip5306.h"

LOG_MODULE_REGISTER(pm, LOG_LEVEL_INF);

/* ---- 可调参数 ---- */
#define PM_IDLE_AFTER_MS     (60U * 1000U)  /* 持续无活动 1 分钟 -> 档1 */
#define PM_OFF_AFTER_MS      (5U * 60U * 1000U) /* 持续无活动 5 分钟 -> 档2 */
#define PM_CHG_POLL_MS       2000U           /* 充电状态监视周期 */
#define PM_IDLE_BRIGHT       5U              /* 档1 背光亮度（0~100） */
#define PM_MODE_POLL_IDLE_MS 1000U           /* 档1 模式轮询放慢到 1s */
#define PM_MODE_POLL_RUN_MS  100U            /* 满速默认 100ms（与 mode 模块一致） */
#define PM_OFF_GRACE_MS      30U             /* 断电前留给全 0 报告发出的时间 */
#define PM_WQ_STACK_SIZE     1024
#define PM_WQ_PRIORITY       5

#define PM_KEYMAP_NODE DT_NODELABEL(kbd_keymap)

/* ---- 内部状态 ---- */
static struct k_work_q pm_workq;
K_THREAD_STACK_DEFINE(pm_wq_stack, PM_WQ_STACK_SIZE);

static struct k_work_delayable idle_work;  /* 1 分钟到点进档1 */
static struct k_work_delayable off_work;   /* 5 分钟到点进档2 */
static struct k_work_delayable chg_work;   /* 充电状态监视 */
static struct k_work exit_work;            /* 档1 -> 满速（异步投递） */

static enum kb_pm_state state = KB_PM_ACTIVE;
static atomic_t auto_on;          /* 自动省电开关（默认开） */
static atomic_t charging_flag;    /* IP5306 充电中（true=充电豁免） */
static atomic_t pm_ready;         /* pm 初始化完成前忽略活动事件 */
static uint8_t saved_bright;      /* 进入档1前的背光亮度 */
static uint32_t act_cnt;          /* 活动次数（调试） */
static uint32_t idle_cnt;         /* 进入档1次数（调试） */
static uint32_t off_cnt;          /* 断电次数（调试） */

/* ---- 工具函数（pm wq 上下文内调用） ---- */

static void activity_arm(void)
{
	act_cnt++;
	k_work_reschedule_for_queue(&pm_workq, &idle_work,
				    K_MSEC(PM_IDLE_AFTER_MS));
	k_work_reschedule_for_queue(&pm_workq, &off_work,
				    K_MSEC(PM_OFF_AFTER_MS));
}

/* 读一次充电状态并同步 charging_flag；返回 0=成功。失败保留原判断。 */
static int charge_read_once(bool *charging)
{
	int err = ip5306_get_charge(charging);

	if (err == 0) {
		atomic_set(&charging_flag, *charging ? 1 : 0);
	}
	return err;
}

/* ---- 档1/档2 动作（只在 pm wq 上执行） ---- */

static void pm_enter_idle(void)
{
	uint8_t idle_pct;

	if (state != KB_PM_ACTIVE) {
		return;
	}

	state = KB_PM_IDLE;
	idle_cnt++;

	saved_bright = kb_backlight_get_percent();
	/* 用户本就把背光关了(0%)则维持 0，否则降到 PM_IDLE_BRIGHT */
	idle_pct = (saved_bright > 0) ? PM_IDLE_BRIGHT : 0;

	LOG_INF("无操作 1 分钟 -> 进入省电(Idle)：背光 %u%% -> %u%%，"
		"停 LCD 刷新/电池采样，模式轮询放慢到 1s "
		"（IP5306 保活每 10s 继续）",
		(unsigned int)saved_bright, (unsigned int)idle_pct);

	(void)kb_backlight_set_percent(idle_pct);
	kb_led_all_off();
	kb_bat_adc_set_sampling(false);
	(void)kb_mode_set_poll_period_ms(PM_MODE_POLL_IDLE_MS);
	kb_mode_set_heartbeat(false);   /* 档1 停 30s 心跳日志 */
	(void)kb_lcd_set_idle(true);
}

static void pm_exit_idle(bool by_charging)
{
	uint8_t cur;

	if (state != KB_PM_IDLE) {
		return;
	}

	state = KB_PM_ACTIVE;

	/* 先恢复常规节奏，再恢复背光（LCD 线程被唤醒后会全量重绘） */
	(void)kb_mode_set_poll_period_ms(PM_MODE_POLL_RUN_MS);
	kb_mode_set_heartbeat(true);   /* 恢复 30s 心跳日志 */
	kb_bat_adc_set_sampling(true);
	(void)kb_lcd_set_idle(false);

	/* 档1期间若用户调过亮度（旋钮导航模式），保留用户刚调的值；
	 * 只有没被碰过（仍是档1目标值）才恢复进档前的亮度。 */
	cur = kb_backlight_get_percent();
	if (cur == PM_IDLE_BRIGHT) {
		(void)kb_backlight_set_percent(saved_bright);
	}

	LOG_INF("退出省电(Idle) -> 满速：背光 %u%%%s",
		(unsigned int)kb_backlight_get_percent(),
		by_charging ? "（充电豁免）" : "");
}

static void pm_power_off(void)
{
	LOG_INF("无操作 5 分钟 -> 断电停机 (POWER OFF)");

	state = KB_PM_OFF;
	off_cnt++;

	/* 1) 全键释放：清 HID 状态，已连接则补发全 0 报告，避免主机残留按键 */
	kb_ble_hid_release_all();
	kb_usb_hid_release_all();

	/* 2) 关灯带（含供电门控）/停 LCD 刷新/背光灭/停电池采样 */
	kb_led_all_off();
	(void)kb_lcd_set_idle(true);
	(void)kb_backlight_set_percent(0);
	kb_bat_adc_set_sampling(false);

	/* 3) 停掉所有计时器 */
	k_work_cancel_delayable(&idle_work);
	k_work_cancel_delayable(&off_work);
	k_work_cancel_delayable(&chg_work);

	/* 4) 停 IP5306 保活：之后 PMIC 会在轻载关机时间内自动待机切断 3V3。
	 *    一旦停掉就没有回头路（IP5306 几十秒内断电），必须放在最后。 */
	ip5306_keepalive_stop();

	/* 5) 给正在发送的全 0 报告一点时间真正发出（USB TX 线程 / BLE 协议栈） */
	k_sleep(K_MSEC(PM_OFF_GRACE_MS));

	LOG_INF("进入 System Off，等待 IP5306 轻载自动待机切断 3V3 "
		"（重启：电源开关重新上电）");

#if defined(CONFIG_POWEROFF)
	sys_poweroff();  /* FUNC_NORETURN，正常不会返回 */
#endif

	/* 兜底：未开启 CONFIG_POWEROFF 或平台不支持时，挂死等 PMIC 掉电 */
	for (;;) {
		k_sleep(K_FOREVER);
	}
}

/* ---- 计时器处理（pm wq 上下文） ---- */

static void idle_work_handler(struct k_work *work)
{
	bool charging;

	ARG_UNUSED(work);

	if (state != KB_PM_ACTIVE) {
		return;
	}
	if (atomic_get(&auto_on) == 0) {
		return;
	}

	if (charge_read_once(&charging) == 0 && charging) {
		/* 到点瞬间正好插上电：不降载，并取消计时（监视器接管） */
		LOG_INF("进入省电到点但检测到充电，取消计时保持满速");
		k_work_cancel_delayable(&idle_work);
		k_work_cancel_delayable(&off_work);
		return;
	}

	pm_enter_idle();
}

static void off_work_handler(struct k_work *work)
{
	bool charging;

	ARG_UNUSED(work);

	if ((state != KB_PM_ACTIVE) && (state != KB_PM_IDLE)) {
		return;
	}
	if (atomic_get(&auto_on) == 0) {
		return;
	}

	if (charge_read_once(&charging) == 0 && charging) {
		LOG_INF("断电到点但检测到充电，取消断电计时（监视器接管）");
		k_work_cancel_delayable(&idle_work);
		k_work_cancel_delayable(&off_work);
		return;
	}

	/* 模式未定（开关停在两档之间）：不动设备，等档位落定后的活动重新计时 */
	if (kb_mode_get() == KB_MODE_UNKNOWN) {
		LOG_WRN("断电到点但模式未定（开关正在换档），暂缓断电");
		return;
	}

	pm_power_off();
}

static void exit_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	pm_exit_idle(false);
}

static void chg_work_handler(struct k_work *work)
{
	bool now;
	bool prev = (atomic_get(&charging_flag) != 0);

	ARG_UNUSED(work);

	if (charge_read_once(&now) != 0) {
		LOG_WRN("充电状态读取失败，稍后重试");
		goto retry;
	}

	if (prev == now) {
		goto retry;
	}

	if (now) {
		/* 插电开始充电：回到满速并暂停计时 */
		LOG_INF("检测到充电开始，取消低功耗计时（充电豁免）");
		k_work_cancel_delayable(&idle_work);
		k_work_cancel_delayable(&off_work);
		if (state == KB_PM_IDLE) {
			pm_exit_idle(true);
		}
	} else {
		/* 充电结束（拔线/充满）：从零重新计时 */
		LOG_INF("充电结束，重新开始低功耗计时");
		activity_arm();
	}

retry:
	k_work_reschedule_for_queue(&pm_workq, &chg_work,
				    K_MSEC(PM_CHG_POLL_MS));
}

/* ---- 活动源 ---- */

/* 任意按键（含 EC11 按压），keymap 设备上解析后的键事件 */
static void pm_key_evt_cb(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	if (evt->type != INPUT_EV_KEY) {
		return;
	}

	kb_pm_activity();
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(PM_KEYMAP_NODE), pm_key_evt_cb, NULL);

/* 旋钮旋转 */
static void pm_knob_cb(enum kb_knob_dir dir, int16_t step, int32_t total,
		       int32_t angle, void *user_data)
{
	ARG_UNUSED(dir);
	ARG_UNUSED(step);
	ARG_UNUSED(total);
	ARG_UNUSED(angle);
	ARG_UNUSED(user_data);

	kb_pm_activity();
}

/* 模式档位变化 */
static void pm_mode_cb(enum kb_mode prev, enum kb_mode now, void *user_data)
{
	ARG_UNUSED(prev);
	ARG_UNUSED(now);
	ARG_UNUSED(user_data);

	kb_pm_activity();
}

/* ---- 对外接口 ---- */

void kb_pm_activity(void)
{
	if (atomic_get(&pm_ready) == 0) {
		return;
	}
	if (state == KB_PM_OFF) {
		return;
	}
	if (atomic_get(&auto_on) == 0) {
		return;
	}
	if (atomic_get(&charging_flag)) {
		return;   /* 充电中不参与计时，充电结束由监视器重新计时 */
	}

	activity_arm();

	if (state == KB_PM_IDLE) {
		/* 退出档1的动作统一在 pm wq 执行，避免在 input 线程做重活 */
		(void)k_work_submit_to_queue(&pm_workq, &exit_work);
	}
}

enum kb_pm_state kb_pm_state_get(void)
{
	return state;
}

int kb_pm_idle_now(void)
{
	k_work_reschedule_for_queue(&pm_workq, &idle_work, K_NO_WAIT);
	return 0;
}

int kb_pm_off_now(void)
{
	k_work_reschedule_for_queue(&pm_workq, &off_work, K_NO_WAIT);
	return 0;
}

int kb_pm_auto_set(bool enable)
{
	atomic_set(&auto_on, enable ? 1 : 0);

	if (enable) {
		LOG_INF("自动省电已开启，重新计时");
		if (atomic_get(&charging_flag) == 0) {
			activity_arm();
		}
	} else {
		LOG_INF("自动省电已关闭，保持满速");
		k_work_cancel_delayable(&idle_work);
		k_work_cancel_delayable(&off_work);
		if (state == KB_PM_IDLE) {
			(void)k_work_submit_to_queue(&pm_workq, &exit_work);
		}
	}

	return 0;
}

bool kb_pm_auto_enabled(void)
{
	return atomic_get(&auto_on) != 0;
}

int kb_pm_init(void)
{
	int err;
	bool charging;

	atomic_set(&auto_on, 1);
	atomic_set(&charging_flag, 0);
	atomic_set(&pm_ready, 0);

	k_work_queue_start(&pm_workq, pm_wq_stack, PM_WQ_STACK_SIZE,
			  PM_WQ_PRIORITY, NULL);
	k_thread_name_set(&pm_workq.thread, "pm");

	k_work_init_delayable(&idle_work, idle_work_handler);
	k_work_init_delayable(&off_work, off_work_handler);
	k_work_init_delayable(&chg_work, chg_work_handler);
	k_work_init(&exit_work, exit_work_handler);

	/* 订阅活动源（放 main 最后调用，knob/mode 此时均已就绪） */
	err = kb_knob_register_cb(pm_knob_cb, NULL);
	if (err) {
		LOG_WRN("订阅旋钮活动失败: %d", err);
	}

	err = kb_mode_register_cb(pm_mode_cb, NULL);
	if (err) {
		LOG_WRN("订阅模式活动失败: %d", err);
	}

	/* 初始充电状态：充电(插着电源)不挂计时，等充电结束再挂 */
	charging = false;
	if (charge_read_once(&charging) != 0) {
		LOG_WRN("初始读取充电状态失败");
	}

	atomic_set(&pm_ready, 1);

	if (!charging) {
		activity_arm();
	}

	k_work_reschedule_for_queue(&pm_workq, &chg_work,
				    K_MSEC(PM_CHG_POLL_MS));

	LOG_INF("低功耗就绪：1 分钟无操作降载、5 分钟无操作断电"
		"（IP5306 保活持续到断电；充电时自动豁免）");

	return 0;
}

/* ==================== shell 调试命令 ==================== */

#if defined(CONFIG_SHELL)

#include <zephyr/shell/shell.h>

static const char *pm_state_str(enum kb_pm_state s)
{
	switch (s) {
	case KB_PM_ACTIVE:
		return "满速";
	case KB_PM_IDLE:
		return "省电(Idle)";
	case KB_PM_OFF:
		return "断电(Off)";
	default:
		return "?";
	}
}

static int cmd_pm_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "低功耗状态: %s / 自动: %s / 充电: %s / 背光: %u%%",
		    pm_state_str(state),
		    atomic_get(&auto_on) ? "开" : "关",
		    atomic_get(&charging_flag) ? "是" : "否",
		    (unsigned int)kb_backlight_get_percent());
	shell_print(sh, "活动=%u 档1进入=%u 断电=%u 档1背光目标=%u%%",
		    (unsigned int)act_cnt, (unsigned int)idle_cnt,
		    (unsigned int)off_cnt, (unsigned int)PM_IDLE_BRIGHT);

	return 0;
}

static int cmd_pm_idle(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	kb_pm_idle_now();
	shell_print(sh, "已请求立即进入档1（1 分钟超时同效果）");

	return 0;
}

static int cmd_pm_off(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "已请求立即执行断电流程（5 分钟超时同效果）");
	kb_pm_off_now();

	return 0;
}

static int cmd_pm_auto(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_print(sh, "用法: pm auto on|off");
		return -EINVAL;
	}

	if (strcmp(argv[1], "on") == 0) {
		kb_pm_auto_set(true);
		shell_print(sh, "自动省电: 开");
	} else if (strcmp(argv[1], "off") == 0) {
		kb_pm_auto_set(false);
		shell_print(sh, "自动省电: 关");
	} else {
		shell_print(sh, "用法: pm auto on|off");
		return -EINVAL;
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_pm,
	SHELL_CMD_ARG(status, NULL, "显示低功耗状态/计数", cmd_pm_status, 1, 0),
	SHELL_CMD_ARG(idle, NULL, "立即进入档1（省电降载）", cmd_pm_idle, 1, 0),
	SHELL_CMD_ARG(now, NULL, "立即进入档1（同 idle，档1 触发用）", cmd_pm_idle, 1, 0),
	SHELL_CMD_ARG(off, NULL, "立即执行断电流程", cmd_pm_off, 1, 0),
	SHELL_CMD_ARG(auto, NULL, "开关自动省电: on|off", cmd_pm_auto, 1, 1),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(pm, &sub_pm, "低功耗（空闲降载/断电）调试", NULL);

#endif /* CONFIG_SHELL */
