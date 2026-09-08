/*
 * BLE HID（HID over GATT 外设）模块实现
 *
 * 角色：nRF52840 跑 Zephyr 蓝牙主机栈 + 片上 2.4G 射频，对电脑呈现为一个
 *      「蓝牙键盘 + 消费控制设备（音量）」，无需驱动，系统蓝牙直接识别。
 *
 * 三种报告（Report ID 由 GATT 的 Report Reference 描述符告知主机，
 *           报文载荷本身【不含】Report ID 字节 —— 与 Nordic nrf_desktop 一致）：
 *      Report ID 1 = Keyboard Keys（输入，8 字节：修饰键 + 保留 + 6 键值）
 *      Report ID 2 = Keyboard LEDs（输出，1 字节：用于同步主机 NumLock）
 *      Report ID 3 = Consumer Control（输入，2 字节：16 位 Consumer 用法）
 *
 * 数据来源：
 *   - 按键：订阅 input-keymap 设备的 INPUT_EV_KEY 事件（得到 INPUT_KEY_xxx）
 *   - 旋钮：订阅 knob 模块的「每格」回调（得到 CW / CCW）
 *   - 静音：矩阵的 (0,3) 键，keymap 里是 INPUT_KEY_MUTE，改为走 Consumer Mute
 *
 * 模式门控（BLE通信.txt 第 8 章）：
 *   - 只有 kb_mode_get() == KB_MODE_BLE 时才组包发送；
 *   - 切进 BLE 档 -> 启动广播；切离 -> 先发一次「全键释放」再停广播。
 *
 * 线程模型：
 *   - 按键回调在 input 线程、旋钮回调在系统工作队列、模式回调在模式工作队列，
 *     三个上下文都会改 HID 状态；bt_hids_inp_rep_send 本身也不是线程安全的，
 *     因此统一用一把互斥锁保护「状态修改 + 发送」。
 *   - 主机 LED 输出报告的回调只写一个 bool，不持锁，避免与发送路径互相嵌套。
 *
 * 详见 BLE通信.txt
 */

#include "ble_hid.h"

#include <errno.h>
#include <string.h>

#include "hid/hid_keymap.h"
#include "knob/knob.h"
#include "led/led.h"
#include "mode/mode.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/input/input.h>
#include <zephyr/input/input_hid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>
#include <zephyr/usb/class/hid.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <bluetooth/services/hids.h>

LOG_MODULE_REGISTER(ble_hid, LOG_LEVEL_INF);

#define DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define BASE_USB_HID_SPEC_VERSION 0x0101

/* ---- 报告长度（均不含 Report ID 字节）---- */
#define INPUT_REP_KEYS_LEN     8   /* 1 修饰键 + 1 保留 + 6 键值 */
#define OUTPUT_REP_LEDS_LEN    1   /* LED 位图 */
#define INPUT_REP_CONSUMER_LEN 2   /* 16 位 Consumer 用法 */

/* ---- Report ID ---- */
#define REPORT_ID_KEYS     1
#define REPORT_ID_LEDS     2
#define REPORT_ID_CONSUMER 3

/*
 * 输入 / 输出报告的「内部索引」（与该报告在 init 时的登记顺序对应）。
 * 注意它和 Report ID 不是一回事：bt_hids_inp_rep_send 用的是索引。
 */
enum {
	INPUT_REP_KEYS_IDX = 0,
	INPUT_REP_CONSUMER_IDX,
	INPUT_REP_COUNT,
};

enum {
	OUTPUT_REP_LEDS_IDX = 0,
	OUTPUT_REP_COUNT,
};

/* ---- 键盘报告内部布局 ---- */
#define KEY_PRESS_MAX    6
#define MODIFIER_KEY_POS 0
#define RESERVED_KEY_POS 1
#define SCAN_CODE_POS    2

/* Keypad 的句点用法：zephyr/usb/class/hid.h 的枚举里没有，按 USB HID 标准补齐 */
#define HID_KEY_KP_DOT 0x63

/* Consumer 用法 / LED 位图 / Report ID 已移入公共层 hid/hid_keymap.h */

/* HIDS 实例：参数是各输入/输出报告的【长度】，用于算每连接的上下文内存 */
BT_HIDS_DEF(hids_obj,
	    INPUT_REP_KEYS_LEN,
	    OUTPUT_REP_LEDS_LEN,
	    INPUT_REP_CONSUMER_LEN);

#define MATRIX_KEYMAP_NODE DT_NODELABEL(kbd_keymap)

BUILD_ASSERT(DT_NODE_HAS_STATUS(MATRIX_KEYMAP_NODE, okay),
	     "未使能 input-keymap 节点：请检查 kb.dts 里 kbd_keymap 的 status");

static const struct device *const keymap_dev = DEVICE_DT_GET(MATRIX_KEYMAP_NODE);

static struct k_mutex hid_lock;

static bool is_connected;
static bool is_advertising;
/*
 * 当前连接的引用（bt_conn_ref 持有，用完必须 unref）。
 * 由 hid_lock 保护指针本身；ref/unref 一律放到锁外做，
 * 因为 unref 可能触发连接对象释放。
 */
static struct bt_conn *cur_conn;
static bool boot_mode;    /* 主机选择了 Boot Protocol（此时键盘报告走 boot 特征） */

/* ---- 配对管理 ----
 * has_bond：本端是否存有已配对主机（决定状态灯慢闪/快闪）。
 * 长按 NumLock 3 秒 -> 清除全部绑定并重新广播（配对模式，蓝灯快闪）。
 */
#define PAIRING_HOLD_MS 3000
static bool has_bond;

static void pairing_timer_expired(struct k_timer *timer);

K_TIMER_DEFINE(pairing_timer, pairing_timer_expired, NULL);

/* 统计已保存的绑定主机数（本 SDK 没有 bt_bond_count，用 bt_foreach_bond 实现） */
static void bond_count_cb(const struct bt_bond_info *info, void *user_data)
{
	ARG_UNUSED(info);
	(*(uint8_t *)user_data)++;
}

static uint8_t saved_bond_count(void)
{
	uint8_t n = 0;

	bt_foreach_bond(BT_ID_DEFAULT, bond_count_cb, &n);

	return n;
}

/* 按键状态机 / NumLock 双功能层翻译已移入公共层 src/hid/hid_keymap.c，
 * 与 USB transport（src/usb/usb_hid.c）共用同一份逻辑与状态。 */

/* ==================== 广播 ==================== */

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE,
		      (CONFIG_BT_DEVICE_APPEARANCE >> 0) & 0xff,
		      (CONFIG_BT_DEVICE_APPEARANCE >> 8) & 0xff),
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HIDS_VAL)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void advertising_start(void)
{
	const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
		BT_LE_ADV_OPT_CONN,
		BT_GAP_ADV_FAST_INT_MIN_2,
		BT_GAP_ADV_FAST_INT_MAX_2,
		NULL);
	int err;

	if (is_advertising || is_connected) {
		return;
	}

	err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		if (err == -EALREADY) {
			is_advertising = true;
		} else {
			LOG_WRN("广播启动失败: %d", err);
		}
		return;
	}

	is_advertising = true;
	LOG_INF("BLE 广播已启动，设备名: %s（%s）", DEVICE_NAME,
		has_bond ? "等待已配对电脑回连" : "配对模式：可被新主机搜索");
	/*
	 * 状态灯：有绑定 -> 蓝灯慢闪（等回连）；
	 *          无绑定 -> 蓝灯快闪（配对模式，等待新主机）。
	 */
	kb_led_status_set(has_bond ? KB_LED_STATUS_ADV_BONDED
				   : KB_LED_STATUS_PAIRING);
}

static void advertising_stop(void)
{
	int err;

	if (!is_advertising) {
		return;
	}

	err = bt_le_adv_stop();
	if (err) {
		LOG_WRN("广播停止失败: %d", err);
		return;
	}

	is_advertising = false;
	LOG_INF("BLE 广播已停止");
}

/* ==================== 报告发送 ==================== */

static int kbd_report_send(void)
{
	uint8_t buf[INPUT_REP_KEYS_LEN] = {0};
	const struct hid_kb_state *st = hid_kb_state_get();

	buf[MODIFIER_KEY_POS] = st->modifier;
	buf[RESERVED_KEY_POS] = 0;
	memcpy(&buf[SCAN_CODE_POS], st->keys, KEY_PRESS_MAX);

	if (boot_mode) {
		return bt_hids_boot_kb_inp_rep_send(&hids_obj, NULL, buf,
						    sizeof(buf), NULL);
	}

	return bt_hids_inp_rep_send(&hids_obj, NULL, INPUT_REP_KEYS_IDX, buf,
				    sizeof(buf), NULL);
}

/*
 * 发送一次 Consumer「点按」：先发用法，再发 0x0000 表示松手。
 * 不发松手帧的话，主机会认为该用法一直按着（音量会持续增减 / 静音粘住）。
 */
static int consumer_tap(uint16_t usage)
{
	uint8_t buf[INPUT_REP_CONSUMER_LEN];
	int err;

	buf[0] = (uint8_t)(usage & 0xff);
	buf[1] = (uint8_t)((usage >> 8) & 0xff);

	err = bt_hids_inp_rep_send(&hids_obj, NULL, INPUT_REP_CONSUMER_IDX, buf,
				   sizeof(buf), NULL);
	if (err) {
		return err;
	}

	buf[0] = 0;
	buf[1] = 0;

	return bt_hids_inp_rep_send(&hids_obj, NULL, INPUT_REP_CONSUMER_IDX, buf,
				    sizeof(buf), NULL);
}

/* ==================== 按键状态 ====================
 * 状态机已移入公共层（hid_kb_state_key / hid_kb_state_reset），
 * 本模块只负责「改状态 + 组包发送」。
 */

/* 清空所有按键并尝试发一次报告（用于切换模式 / 断开连接前的「全键释放」） */
static void kbd_release_all(void)
{
	hid_kb_state_reset();

	if (!is_connected) {
		return;
	}

	int err = kbd_report_send();

	if (err) {
		LOG_DBG("全键释放报告发送失败: %d", err);
	}
}

/*
 * INPUT_KEY_* -> HID 用法的翻译（含 NumLock 层）已移入公共层：
 * hid_keymap_resolve()，USB transport 与本模块共用。
 */

/* ==================== 输入事件 ==================== */

/*
 * 直接订阅 keymap 设备：它已经把 (行,列) 翻译成了标准 INPUT_KEY_* 键值，
 * 本模块不必关心矩阵。（input 子系统支持同一设备挂多个回调，
 * 所以 keys.c 里那份日志回调不受影响。）
 */
static void keymap_evt_cb(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);
	int err;

	if (evt->type != INPUT_EV_KEY) {
		return;
	}

	/* 模式门控：非 BLE 模式下按键不从蓝牙通道发出 */
	if (kb_mode_get() != KB_MODE_BLE) {
		return;
	}

	bool pressed = (evt->value != 0);

	/* 长按 NumLock 3 秒 -> 进入配对模式（按键本身仍正常上报） */
	if (evt->code == INPUT_KEY_NUMLOCK) {
		if (pressed) {
			/* 本地乐观切换：立即生效，主机 LED 报告到达后校正 */
			hid_numlock_toggle_local();

			k_timer_start(&pairing_timer,
				      K_MSEC(PAIRING_HOLD_MS), K_NO_WAIT);
		} else {
			k_timer_stop(&pairing_timer);
		}
	}

	/* 旋钮按键：走 Consumer 报告，不是普通键盘键 */
	if (evt->code == INPUT_KEY_MUTE) {
		if (!pressed) {
			return;  /* Consumer 用法在点按时自带「松手帧」，松开无需处理 */
		}

		k_mutex_lock(&hid_lock, K_FOREVER);
		err = is_connected ? consumer_tap(HID_CONSUMER_USAGE_MUTE) : 0;
		k_mutex_unlock(&hid_lock);

		if (err) {
			LOG_DBG("Mute 报告发送失败: %d", err);
		} else if (is_connected) {
			LOG_INF("BLE 上报: Consumer Mute（静音切换）");
		}
		return;
	}

	/* 修饰键走 Byte0 位图，不占 6 个键值槽，也不参与上面的键值翻译 */
	uint8_t mod_mask = hid_keymap_modifier(evt->code);
	uint8_t usage = 0;

	if (mod_mask == 0) {
		err = hid_keymap_resolve(evt->code, &usage);
		if (err) {
			LOG_WRN("未映射的键值 INPUT_KEY_%u (%s)",
				(unsigned int)evt->code,
				pressed ? "按下" : "松开");
			return;
		}
	}

	k_mutex_lock(&hid_lock, K_FOREVER);

	hid_kb_state_key(usage, mod_mask, pressed);

	err = is_connected ? kbd_report_send() : 0;

	k_mutex_unlock(&hid_lock);

	if (err) {
		LOG_DBG("键盘报告发送失败: %d", err);
	} else if (is_connected) {
		LOG_INF("BLE 上报: HID 0x%02X -> %s", usage,
			pressed ? "按下" : "松开");
	}
}

INPUT_CALLBACK_DEFINE(keymap_dev, keymap_evt_cb, NULL);

/* ==================== 旋钮：系统音量 ==================== */

static void knob_cb(enum kb_knob_dir dir, int16_t step, int32_t total,
		    int32_t angle, void *user_data)
{
	ARG_UNUSED(step);
	ARG_UNUSED(total);
	ARG_UNUSED(angle);
	ARG_UNUSED(user_data);
	int err;

	/* 方向（导航）模式（NumLock 关）时旋钮让给 LCD 调背光亮度；
	 * 数字模式（NumLock 开）才用旋钮调音量（见 lcd 显示.txt 5） */
	if (!hid_numlock_get()) {
		return;
	}

	if (kb_mode_get() != KB_MODE_BLE) {
		return;
	}

	bool up = (dir == KB_KNOB_CW);
	uint16_t usage = up ? HID_CONSUMER_USAGE_VOL_UP
			    : HID_CONSUMER_USAGE_VOL_DOWN;

	k_mutex_lock(&hid_lock, K_FOREVER);
	err = is_connected ? consumer_tap(usage) : 0;
	k_mutex_unlock(&hid_lock);

	if (err) {
		LOG_DBG("音量报告发送失败: %d", err);
	} else if (is_connected) {
		LOG_INF("BLE 上报: 音量 %s", up ? "+" : "-");
	}
}

/* ==================== 模式切换 ==================== */

static void mode_cb(enum kb_mode prev, enum kb_mode now, void *user_data)
{
	ARG_UNUSED(prev);
	ARG_UNUSED(user_data);

	if (now == KB_MODE_BLE) {
		/* 进入 BLE：清一次残留按键状态，然后开始广播等待电脑连接 */
		k_mutex_lock(&hid_lock, K_FOREVER);
		kbd_release_all();
		k_mutex_unlock(&hid_lock);

		advertising_start();
	} else {
		struct bt_conn *conn = NULL;

		/*
		 * 离开 BLE：顺序必须是「全键释放 -> 停广播 -> 主动断连」。
		 * 只停广播不断连是不够的：电脑那头的蓝牙连接依然存在，
		 * 看起来就像「拨了开关还是蓝牙模式」。
		 */
		k_mutex_lock(&hid_lock, K_FOREVER);
		kbd_release_all();
		conn = cur_conn;
		cur_conn = NULL;
		k_mutex_unlock(&hid_lock);

		advertising_stop();
		kb_led_status_set(KB_LED_STATUS_OFF);   /* 非 BLE 档：状态灯熄灭 */

		if (conn != NULL) {
			int err = bt_conn_disconnect(
				conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);

			LOG_INF("离开 BLE 模式：主动断开蓝牙连接 (err=%d)", err);
			bt_conn_unref(conn);
		} else {
			LOG_INF("离开 BLE 模式：广播已停止");
		}
	}
}

/* ==================== 主动配对 ====================
 * 触发：长按 NumLock 3 秒（pairing_timer 计时），或 shell 命令 `ble pair`。
 * 动作：清除本端全部绑定 -> 断开现有连接 -> 重新广播（无绑定 = 配对模式）。
 * 注意：已配对的电脑会被解绑，必须在其蓝牙设置里删除设备后重新搜索配对。
 */

/*
 * ⚠️ k_timer 回调跑在系统时钟线程，【不可阻塞】；
 * 而 bt_unpair 内部可能发同步 HCI 命令并等待 Controller 回应。
 * 在 timer 上下文里直接调用会 HCI 命令超时（opcode 0x0406, err -11）
 * -> 内核断言挂死。必须经由系统工作队列转一手。
 */
static void pairing_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (kb_mode_get() != KB_MODE_BLE) {
		return;
	}

	(void)kb_ble_hid_enter_pairing();
}

static K_WORK_DEFINE(pairing_work, pairing_work_handler);

static void pairing_timer_expired(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	LOG_INF("NumLock 长按 %u 秒 -> 进入配对模式",
		(unsigned)(PAIRING_HOLD_MS / 1000));
	k_work_submit(&pairing_work);
}

int kb_ble_hid_enter_pairing(void)
{
	LOG_INF("进入配对模式：清除全部绑定 -> 重新广播");

	has_bond = false;

	/*
	 * bt_unpair 会逐个终止与被解绑设备的连接，
	 * 由此触发 disconnected_cb -> 重新广播（灯效转配对快闪）。
	 */
	(void)bt_unpair(BT_ID_DEFAULT, BT_ADDR_LE_ANY);

	/* 若本就未连接，unpair 不会触发断连回调，这里显式保证处于广播态
	 * （stop/start 均幂等，重复调用无害）。 */
	advertising_stop();
	advertising_start();

	return 0;
}

/* ==================== 连接管理 ==================== */

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_WRN("连接失败 %s (0x%02x %s)", addr, err,
			bt_hci_err_to_str(err));
		return;
	}

	struct bt_conn *old = NULL;
	struct bt_conn *ref = bt_conn_ref(conn);  /* 先取一份引用，供后续主动断连用 */
	int hids_err;

	k_mutex_lock(&hid_lock, K_FOREVER);
	hids_err = bt_hids_connected(&hids_obj, conn);
	is_connected = true;
	is_advertising = false;
	hid_kb_state_reset();
	old = cur_conn;
	cur_conn = ref;
	k_mutex_unlock(&hid_lock);

	if (old != NULL) {
		bt_conn_unref(old);
	}

	if (hids_err) {
		LOG_ERR("通知 HIDS 连接失败: %d", hids_err);
	}

	/* 已连接：熄灭状态灯（不常亮），"/" 键的按键灯反馈随之恢复 */
	kb_led_status_set(KB_LED_STATUS_OFF);

	LOG_INF("BLE 已连接: %s（状态灯已熄灭）", addr);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	int err = bt_hids_disconnected(&hids_obj, conn);

	if (err) {
		LOG_WRN("通知 HIDS 断开失败: %d", err);
	}

	struct bt_conn *stale = NULL;

	k_mutex_lock(&hid_lock, K_FOREVER);
	if (cur_conn == conn) {
		stale = cur_conn;
		cur_conn = NULL;
	}
	is_connected = false;
	boot_mode = false;
	hid_kb_state_reset();   /* 丢掉残留的按下状态 */
	/*
	 * ⚠️ NumLock 状态不复位（保持跨连接/切档）：
	 * USB 主机连接后【不会】像 BLE 主机那样主动同步初始 LED 状态，
	 * 若在此复位，切到 USB 档时导航模式的 NumLock 指示灯会凭空熄灭，
	 * 表现为「USB 链接时 NumLock 功能开启不常亮，与 BLE 不一致」。
	 * 主机端 NumLock 状态通常不变，保持设备侧状态最接近真实。
	 */
	k_mutex_unlock(&hid_lock);

	kb_led_force_off(1, 0);                     /* 熄灭 NumLock 指示灯 */

	if (stale != NULL) {
		bt_conn_unref(stale);
	}

	LOG_INF("BLE 已断开: %s (0x%02x %s)", addr, reason,
		bt_hci_err_to_str(reason));

	/* 仍在 BLE 模式 -> 重新广播，等下一次连接 */
	if (kb_mode_get() == KB_MODE_BLE) {
		advertising_start();
	}
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level,
				enum bt_security_err err)
{
	if (err) {
		LOG_WRN("加密失败 level=%u err=%d", level, err);
	} else {
		LOG_INF("链路加密等级: %u", level);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.security_changed = security_changed_cb,
};

static void pairing_complete_cb(struct bt_conn *conn, bool bonded)
{
	ARG_UNUSED(conn);

	has_bond = true;
	/* 绿灯闪三下，结束后自动回到「已连接」常亮 */
	kb_led_status_set(KB_LED_STATUS_PAIR_OK);

	LOG_INF("配对完成 (bonded=%d)，绑定信息已写入 Flash", bonded ? 1 : 0);
}

static void pairing_failed_cb(struct bt_conn *conn, enum bt_security_err reason)
{
	ARG_UNUSED(conn);

	/* 配对失败：回到配对模式快闪，等待重试 */
	kb_led_status_set(KB_LED_STATUS_PAIRING);

	LOG_WRN("配对失败: %d", reason);
}

static struct bt_conn_auth_info_cb auth_info_callbacks = {
	.pairing_complete = pairing_complete_cb,
	.pairing_failed = pairing_failed_cb,
};

/* ==================== HID 输出报告：同步主机 NumLock ==================== */

static void led_report_apply(uint8_t leds)
{
	/* NumLock 同步与 (1,0) 指示灯逻辑在公共层（与 USB transport 共用） */
	hid_numlock_set((leds & HID_LED_BIT_NUM_LOCK) != 0);
}

static void hids_outp_rep_handler(struct bt_hids_rep *rep, struct bt_conn *conn,
				  bool write)
{
	ARG_UNUSED(conn);

	if (!write || (rep->size < OUTPUT_REP_LEDS_LEN)) {
		return;
	}

	led_report_apply(rep->data[0]);
}

static void hids_boot_kb_outp_rep_handler(struct bt_hids_rep *rep,
					  struct bt_conn *conn, bool write)
{
	ARG_UNUSED(conn);

	if (!write || (rep->size < 1)) {
		return;
	}

	led_report_apply(rep->data[0]);
}

static void hids_pm_evt_handler(enum bt_hids_pm_evt evt, struct bt_conn *conn)
{
	ARG_UNUSED(conn);

	if (evt == BT_HIDS_PM_EVT_BOOT_MODE_ENTERED) {
		boot_mode = true;
		LOG_INF("主机进入 Boot Protocol 模式");
	} else if (evt == BT_HIDS_PM_EVT_REPORT_MODE_ENTERED) {
		boot_mode = false;
		LOG_INF("主机进入 Report Protocol 模式");
	}
}

/* ==================== HIDS 初始化 ==================== */

/* HID 报告描述符已移入公共层 src/hid/hid_keymap.c（与 USB transport 共用） */

static int hid_service_init(void)
{
	int err;
	struct bt_hids_init_param hids_init_obj = {0};
	struct bt_hids_inp_rep *hids_inp_rep;
	struct bt_hids_outp_feat_rep *hids_outp_rep;

	hids_init_obj.rep_map.data = hid_report_map;
	hids_init_obj.rep_map.size = hid_report_map_size;

	hids_init_obj.info.bcd_hid = BASE_USB_HID_SPEC_VERSION;
	hids_init_obj.info.b_country_code = 0x00;
	hids_init_obj.info.flags = (BT_HIDS_REMOTE_WAKE |
				    BT_HIDS_NORMALLY_CONNECTABLE);

	/* 输入报告 0：键盘键值（8 字节） */
	hids_inp_rep = &hids_init_obj.inp_rep_group_init.reports[INPUT_REP_KEYS_IDX];
	hids_inp_rep->id = REPORT_ID_KEYS;
	hids_inp_rep->size = INPUT_REP_KEYS_LEN;
	hids_init_obj.inp_rep_group_init.cnt++;

	/* 输入报告 1：消费控制 / 音量（2 字节） */
	hids_inp_rep = &hids_init_obj.inp_rep_group_init.reports[INPUT_REP_CONSUMER_IDX];
	hids_inp_rep->id = REPORT_ID_CONSUMER;
	hids_inp_rep->size = INPUT_REP_CONSUMER_LEN;
	hids_init_obj.inp_rep_group_init.cnt++;

	/* 输出报告 0：键盘 LED，用来同步主机 NumLock（1 字节） */
	hids_outp_rep = &hids_init_obj.outp_rep_group_init.reports[OUTPUT_REP_LEDS_IDX];
	hids_outp_rep->id = REPORT_ID_LEDS;
	hids_outp_rep->size = OUTPUT_REP_LEDS_LEN;
	hids_outp_rep->handler = hids_outp_rep_handler;
	hids_init_obj.outp_rep_group_init.cnt++;

	hids_init_obj.is_kb = true;
	hids_init_obj.boot_kb_outp_rep_handler = hids_boot_kb_outp_rep_handler;
	hids_init_obj.pm_evt_handler = hids_pm_evt_handler;

	err = bt_hids_init(&hids_obj, &hids_init_obj);
	if (err) {
		LOG_ERR("HIDS 初始化失败: %d", err);
		return err;
	}

	return 0;
}

/* ==================== 对外接口 ==================== */

bool kb_ble_hid_is_connected(void)
{
	return is_connected;
}

bool kb_ble_hid_is_advertising(void)
{
	return is_advertising;
}

bool kb_ble_hid_numlock_get(void)
{
	return hid_numlock_get();
}

int kb_ble_hid_init(void)
{
	int err;

	if (!device_is_ready(keymap_dev)) {
		LOG_ERR("键值映射设备未就绪: %s", keymap_dev->name);
		return -ENODEV;
	}

	k_mutex_init(&hid_lock);

	err = hid_service_init();
	if (err) {
		return err;
	}

	err = bt_conn_auth_info_cb_register(&auth_info_callbacks);
	if (err) {
		LOG_WRN("注册配对信息回调失败: %d", err);
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("蓝牙协议栈启动失败: %d", err);
		return err;
	}

	LOG_INF("蓝牙协议栈就绪，设备名: %s", DEVICE_NAME);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		/* 载入已保存的配对/绑定信息，实现重连免配对 */
		err = settings_load();
		if (err) {
			LOG_WRN("settings 载入失败: %d（首次上电无绑定时正常）", err);
		} else {
			LOG_INF("已载入绑定信息");
		}
	}

	/* 有绑定 -> 蓝灯慢闪等回连；无绑定 -> 蓝灯快闪提示可配对 */
	has_bond = (saved_bond_count() > 0);
	LOG_INF("已保存绑定主机数: %u", (unsigned int)saved_bond_count());

	err = kb_mode_register_cb(mode_cb, NULL);
	if (err) {
		LOG_ERR("订阅模式变化失败: %d", err);
		return err;
	}

	err = kb_knob_register_cb(knob_cb, NULL);
	if (err) {
		LOG_ERR("订阅旋钮事件失败: %d", err);
		return err;
	}

	/* 上电就在 BLE 档：直接开始广播（省得再拨一次开关） */
	if (kb_mode_get() == KB_MODE_BLE) {
		advertising_start();
	} else {
		LOG_INF("当前模式 %s：BLE 暂不广播，切换到 BLE 档后自动开启",
			kb_mode_name(kb_mode_get()));
	}

	return 0;
}

/* ==================== shell 调试命令 ==================== */

#if defined(CONFIG_SHELL)

#include <zephyr/shell/shell.h>

static int cmd_ble_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "BLE 状态: %s / 广播: %s / 协议: %s / NumLock: %s",
		    is_connected ? "已连接" : "未连接",
		    is_advertising ? "是" : "否",
		    boot_mode ? "Boot" : "Report",
		    hid_numlock_get() ? "开(数字层)" : "关(导航层)");
	shell_print(sh, "当前模式: %s（仅 BLE 档才从蓝牙发出）",
		    kb_mode_name(kb_mode_get()));
	shell_print(sh, "绑定状态: %s",
		    has_bond ? "已有配对主机（等回连，蓝灯慢闪）"
			     : "无（配对模式，蓝灯快闪）");
	{
		const struct hid_kb_state *st = hid_kb_state_get();

		shell_print(sh, "按下状态: 修饰键=0x%02X 键值=%02X %02X %02X %02X %02X %02X",
			    st->modifier, st->keys[0], st->keys[1],
			    st->keys[2], st->keys[3], st->keys[4], st->keys[5]);
	}

	return 0;
}

static int cmd_ble_pair(const struct shell *sh, size_t argc, char **argv)
{
	int err;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	err = kb_ble_hid_enter_pairing();
	shell_print(sh, "已进入配对模式（清除绑定+重新广播），err=%d", err);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_ble,
	SHELL_CMD_ARG(status, NULL, "显示 BLE 连接/广播/NumLock 状态",
		      cmd_ble_status, 1, 0),
	SHELL_CMD_ARG(pair, NULL, "清除全部绑定并进入配对模式",
		      cmd_ble_pair, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(ble, &sub_ble, "BLE HID（蓝牙通信）", NULL);

#endif /* CONFIG_SHELL */
