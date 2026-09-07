/*
 * USB HID（三模开关 USB 档）实现
 *
 * 角色与 src/ble/ble_hid.c 对称：同一个 keymap 设备上再挂一个 input 回调，
 * 仅在 KB_MODE_USB 时处理按键；翻译/状态机/NumLock 同步走公共层
 * src/hid/hid_keymap.c，本文件只负责「USB 传输」。
 *
 * ⚠️ 与 BLE 的报文差异：USB 中断传输报文首字节【必须】带 Report ID
 *    （键盘 9 字节 / Consumer 3 字节），Boot 协议模式下不带（8 字节）。
 *
 * 架构与 API 对齐官方样例 samples/subsys/usb/hid-keyboard/（USB_DEVICE_STACK_NEXT）。
 *
 * 详见 usb 通信.txt
 */

#include "usb_hid.h"

#include <errno.h>
#include <string.h>

#include "hid/hid_keymap.h"
#include "mode/mode.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/usb/usbd.h>

LOG_MODULE_REGISTER(usb_hid, LOG_LEVEL_INF);

#define MATRIX_KEYMAP_NODE DT_NODELABEL(kbd_keymap)

BUILD_ASSERT(DT_NODE_HAS_STATUS(MATRIX_KEYMAP_NODE, okay),
	     "未使能 input-keymap 节点：请检查 kb.dts 里 kbd_keymap 的 status");

static const struct device *const keymap_dev = DEVICE_DT_GET(MATRIX_KEYMAP_NODE);
static const struct device *const hid_dev = DEVICE_DT_GET_ONE(zephyr_hid_device);

/*
 * VID/PID：开发期暂用 Zephyr 样例 VID 0x2FE3（官方声明仅限样例使用）。
 * ⚠️ 量产前必须更换为正式 VID/PID（usb.org 购买），并同步改产品名字符串。
 */
#define USB_VID 0x2FE3
#define USB_PID 0x000A

/* ==================== usbd 上下文组装 ====================
 * 精简自 samples/subsys/usb/common/sample_usbd_init.c：
 * nRF52840 只有 Full-Speed，去掉 HS 分支；板上无 VBUS 检测，直接 enable。
 */
USBD_DEVICE_DEFINE(usb_ctx, DEVICE_DT_GET(DT_NODELABEL(usbd)), USB_VID, USB_PID);
USBD_DESC_LANG_DEFINE(usb_lang);
USBD_DESC_MANUFACTURER_DEFINE(usb_mfr, "kb");
USBD_DESC_PRODUCT_DEFINE(usb_product, "BLE Keypad");
USBD_DESC_CONFIG_DEFINE(usb_fs_cfg_desc, "FS Configuration");

/* bus powered + 远程唤醒：
 * ⚠️ Windows 会在键盘空闲几秒后 selective suspend 挂起设备（日志可见
 * SUSPEND/RESUMING）。挂起期间 IN 报文发不出去——若不开远程唤醒，
 * 短按 NumLock 等按键会被主机遗漏（表现为功能不切换、灯效混乱）。
 * 声明 REMOTE_WAKEUP 后设备可主动唤醒主机。 */
USBD_CONFIGURATION_DEFINE(usb_fs_config, USB_SCD_REMOTE_WAKEUP, 100,
			  &usb_fs_cfg_desc);

static bool usb_ready;      /* HID 接口就绪（主机已完成配置） */
static bool usb_boot_mode;  /* 主机选择了 Boot Protocol */

static void usbd_msg_cb(struct usbd_context *const usbd_ctx,
			const struct usbd_msg *const msg)
{
	ARG_UNUSED(usbd_ctx);

	if (msg->type == USBD_MSG_CONFIGURATION) {
		LOG_INF("USB 配置值: %d（枚举完成）", msg->status);
	} else if (msg->type == USBD_MSG_SUSPEND) {
		LOG_INF("USB 挂起");
	} else if (msg->type == USBD_MSG_RESUME) {
		LOG_INF("USB 恢复");
	}
}

/* ==================== HID 类回调 ==================== */

static void usb_iface_ready(const struct device *dev, const bool ready)
{
	ARG_UNUSED(dev);

	usb_ready = ready;
	LOG_INF("USB HID 接口%s", ready ? "就绪" : "未就绪");
}

static int usb_get_report(const struct device *dev, const uint8_t type,
			  const uint8_t id, const uint16_t len,
			  uint8_t *const buf)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(type);
	ARG_UNUSED(id);
	ARG_UNUSED(len);
	ARG_UNUSED(buf);

	/* 不支持按 Report ID 取值（主机正常流程不会用到） */
	return 0;
}

static int usb_set_report(const struct device *dev, const uint8_t type,
			  const uint8_t id, const uint16_t len,
			  const uint8_t *const buf)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(id);

	if (type != HID_REPORT_TYPE_OUTPUT) {
		return -ENOTSUP;
	}

	if ((buf == NULL) || (len < 1)) {
		return -EINVAL;
	}

	/* 调试期：打印主机 LED 报告原始数据（确认 Report ID 剥离是否正确） */
	if (len <= 2) {
		LOG_INF("USB LED 报告: len=%u data=%02X%02X", (unsigned int)len,
			(len > 1) ? buf[0] : 0U, buf[len - 1]);
	}

	/* 主机 NumLock 同步（公共层处理双功能层切换与 (1,0) 指示灯） */
	hid_numlock_set((buf[0] & HID_LED_BIT_NUM_LOCK) != 0);

	return 0;
}

static void usb_output_report(const struct device *dev, const uint16_t len,
			      const uint8_t *const buf)
{
	const uint8_t *leds = buf;
	uint16_t leds_len = len;

	/*
	 * ⚠️ OUT 中断管道的报文首字节是 Report ID（usbd_hid 类透传不剥）：
	 * 主机发的是 [0x02, LED位图]。控制管道 Set_Report 则不含 ID 前缀。
	 * 这里剥掉 ID 后统一交给 usb_set_report。
	 */
	if ((buf != NULL) && (len >= 2) && (buf[0] == HID_REPORT_ID_LEDS)) {
		leds = &buf[1];
		leds_len = len - 1;
	}

	(void)usb_set_report(dev, HID_REPORT_TYPE_OUTPUT, 0, leds_len, leds);
}

static void usb_set_idle(const struct device *dev, const uint8_t id,
			 const uint32_t duration)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(id);
	ARG_UNUSED(duration);
	/* Idle rate 暂不实现 */
}

static uint32_t usb_get_idle(const struct device *dev, const uint8_t id)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(id);

	return 0;
}

static void usb_set_protocol(const struct device *dev, const uint8_t proto)
{
	ARG_UNUSED(dev);

	usb_boot_mode = (proto == 0U);
	LOG_INF("USB 协议切换: %s", usb_boot_mode ? "Boot" : "Report");
}

static struct hid_device_ops usb_hid_ops = {
	.iface_ready = usb_iface_ready,
	.get_report = usb_get_report,
	.set_report = usb_set_report,
	.set_idle = usb_set_idle,
	.get_idle = usb_get_idle,
	.set_protocol = usb_set_protocol,
	.output_report = usb_output_report,
};

/* ==================== 报告发送（首字节带 Report ID） ====================
 * ⚠️ hid_device_submit_report 可能阻塞等待端点，绝不能在 input 线程直接调用
 *    （否则一次阻塞 = 按键全停）。input 回调只把报文入队，
 *    由下面的专用线程负责发送（官方 hid-keyboard 样例同为队列模式）。
 */
struct usb_tx_msg {
	uint8_t len;
	uint8_t data[9];   /* 最长：Report 协议键盘报文 9 字节 */
};

K_MSGQ_DEFINE(usb_tx_msgq, sizeof(struct usb_tx_msg), 8, 1);

static void usb_tx_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct usb_tx_msg m;

	for (;;) {
		k_msgq_get(&usb_tx_msgq, &m, K_FOREVER);

		if (!usb_ready) {
			continue;
		}

		/*
		 * ⚠️ 主机 selective suspend 期间 IN 传输无法完成，报文会卡死。
		 * 检测到挂起先请求远程唤醒并等总线恢复（最多 ~2s），再发送。
		 * 对齐官方 hid-keyboard 样例的挂起处理。
		 */
		if (usbd_is_suspended(&usb_ctx)) {
			int err = usbd_wakeup_request(&usb_ctx);

			if (err) {
				LOG_WRN("远程唤醒请求失败: %d", err);
			}

			for (int i = 0; i < 200 && usbd_is_suspended(&usb_ctx);
			     i++) {
				k_msleep(10);
			}

			if (usbd_is_suspended(&usb_ctx)) {
				LOG_WRN("USB 仍挂起，丢弃报告");
				continue;
			}
		}

		int err = hid_device_submit_report(hid_dev, m.len, m.data);

		if (err) {
			LOG_ERR("USB 报告发送失败: %d", err);
		}
	}
}

#define USB_TX_STACK_SIZE 1536
#define USB_TX_PRIORITY   7
K_THREAD_DEFINE(usb_tx_thread, USB_TX_STACK_SIZE, usb_tx_thread_fn,
		NULL, NULL, NULL, USB_TX_PRIORITY, 0, 0);

static void usb_enqueue(const uint8_t *buf, uint16_t len)
{
	struct usb_tx_msg m;

	if (!usb_ready || (len > sizeof(m.data))) {
		return;
	}

	m.len = (uint8_t)len;
	memcpy(m.data, buf, len);

	if (k_msgq_put(&usb_tx_msgq, &m, K_NO_WAIT) != 0) {
		LOG_WRN("USB 发送队列满，报告被丢弃");
	}
}

static void usb_kbd_report_send(void)
{
	const struct hid_kb_state *st = hid_kb_state_get();

	if (usb_boot_mode) {
		/* Boot 协议键盘报文（8 字节，不带 Report ID） */
		uint8_t buf[8] = {0};

		buf[0] = st->modifier;
		memcpy(&buf[2], st->keys, 6);

		usb_enqueue(buf, sizeof(buf));
		return;
	}

	/* Report 协议键盘报文（9 字节：Report ID + 修饰键 + 保留 + 6 键值） */
	uint8_t buf[1 + 8] = { HID_REPORT_ID_KEYS };

	buf[1] = st->modifier;
	memcpy(&buf[3], st->keys, 6);

	usb_enqueue(buf, sizeof(buf));
}

static void usb_consumer_tap(uint16_t usage)
{
	/* Consumer 报文（3 字节：Report ID + 16 位用法） */
	uint8_t buf[3] = {
		HID_REPORT_ID_CONSUMER,
		(uint8_t)(usage & 0xff),
		(uint8_t)(usage >> 8),
	};

	usb_enqueue(buf, sizeof(buf));

	/* 发 0x0000 松手帧，否则主机会认为该用法一直按着 */
	buf[1] = 0;
	buf[2] = 0;
	usb_enqueue(buf, sizeof(buf));
}

/* ==================== 输入事件 ==================== */

static void usb_keymap_evt_cb(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);
	int err;

	if (evt->type != INPUT_EV_KEY) {
		return;
	}

	/* 模式门控：仅 USB 档从 USB 通道发出 */
	if (kb_mode_get() != KB_MODE_USB) {
		return;
	}

	bool pressed = (evt->value != 0);

	/* 旋钮按键：走 Consumer 报告，不是普通键盘键 */
	if (evt->code == INPUT_KEY_MUTE) {
		if (!pressed) {
			return;  /* Consumer 点按自带「松手帧」，松开无需处理 */
		}

		usb_consumer_tap(HID_CONSUMER_USAGE_MUTE);
		LOG_INF("USB 上报: Consumer Mute（静音切换）");
		return;
	}

	/* 物理 NumLock 按下：本地乐观切换层与灯，主机 LED 报告到达后校正
	 * （主机 LED 报告往往延迟 0.5~1 秒，只等回环会让短按体验明显滞后） */
	if ((evt->code == INPUT_KEY_NUMLOCK) && pressed) {
		hid_numlock_toggle_local();
	}

	/* 修饰键走位图，不占 6 个键值槽 */
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

	hid_kb_state_key(usage, mod_mask, pressed);

	usb_kbd_report_send();
}

INPUT_CALLBACK_DEFINE(keymap_dev, usb_keymap_evt_cb, NULL);

/* ==================== 模式联动 ==================== */

static void usb_mode_cb(enum kb_mode prev, enum kb_mode now, void *user_data)
{
	ARG_UNUSED(prev);
	ARG_UNUSED(user_data);
	int err;

	/*
	 * 档位变化时清一次公共按键状态，防止切档瞬间的残留按下在新通道粘住。
	 * BLE 离开时 ble_hid 的 mode_cb 也会做同样处理，重复无害。
	 */
	hid_kb_state_reset();
	k_msgq_purge(&usb_tx_msgq);   /* 丢弃切档前残留的待发报文 */

	if (now == KB_MODE_USB) {
		/*
		 * ⚠️ USB 控制器只在 USB 档上电：
		 * nRF52840 上 USB（需要 HFCLK 供 SOF）与 BLE 控制器（MPSL）
		 * 同时活跃存在时钟仲裁约束，USB 抢不到时钟会挂死。
		 * 三模开关保证二者物理互斥 —— 离开 USB 档立即关闭 USB。
		 */
		usb_ready = false;
		err = usbd_enable(&usb_ctx);
		if (err && (err != -EALREADY)) {
			LOG_ERR("USBD 使能失败: %d", err);
			return;
		}

		/* 立即发一帧全 0，清掉主机侧可能的残留按键 */
		(void)usb_kbd_report_send();
	} else {
		usb_ready = false;
		err = usbd_disable(&usb_ctx);
		if (err && (err != -EALREADY)) {
			LOG_WRN("USBD 关闭失败: %d", err);
		}
	}
}

/* ==================== 初始化 ==================== */

int kb_usb_hid_init(void)
{
	static const char *const class_blocklist[] = { NULL };
	int err;

	if (!device_is_ready(hid_dev)) {
		LOG_ERR("USB HID 设备未就绪: %s", hid_dev->name);
		return -ENODEV;
	}

	if (!device_is_ready(keymap_dev)) {
		LOG_ERR("键值映射设备未就绪: %s", keymap_dev->name);
		return -ENODEV;
	}

	err = hid_device_register(hid_dev, hid_report_map, hid_report_map_size,
				  &usb_hid_ops);
	if (err) {
		LOG_ERR("USB HID 注册失败: %d", err);
		return err;
	}

	err = kb_mode_register_cb(usb_mode_cb, NULL);
	if (err) {
		LOG_ERR("订阅模式变化失败: %d", err);
		return err;
	}

	/* ---- 组装 usbd 上下文 ---- */
	err = usbd_add_descriptor(&usb_ctx, &usb_lang);
	if (err) {
		LOG_ERR("语言描述符失败: %d", err);
		return err;
	}

	err = usbd_add_descriptor(&usb_ctx, &usb_mfr);
	if (err) {
		LOG_ERR("厂商描述符失败: %d", err);
		return err;
	}

	err = usbd_add_descriptor(&usb_ctx, &usb_product);
	if (err) {
		LOG_ERR("产品描述符失败: %d", err);
		return err;
	}

	err = usbd_add_configuration(&usb_ctx, USBD_SPEED_FS, &usb_fs_config);
	if (err) {
		LOG_ERR("FS 配置注册失败: %d", err);
		return err;
	}

	err = usbd_register_all_classes(&usb_ctx, USBD_SPEED_FS, 1,
					class_blocklist);
	if (err) {
		LOG_ERR("USB 类注册失败: %d", err);
		return err;
	}

	usbd_device_set_code_triple(&usb_ctx, USBD_SPEED_FS, 0, 0, 0);

	err = usbd_msg_register_cb(&usb_ctx, usbd_msg_cb);
	if (err) {
		LOG_ERR("注册 USBD 消息回调失败: %d", err);
		return err;
	}

	err = usbd_init(&usb_ctx);
	if (err) {
		LOG_ERR("USBD 初始化失败: %d", err);
		return err;
	}

	/*
	 * ⚠️ 初始化时【不】usbd_enable：
	 * USB 与 BLE 控制器（MPSL）同时活跃存在 HFCLK 仲裁冲突，
	 * 上电即开 USB 会导致蓝牙模式也卡死。USB 控制器由 mode_cb
	 * 在进入 USB 档时才使能、离开时关闭（与三模开关互斥语义一致）。
	 */
	if (kb_mode_get() == KB_MODE_USB) {
		/* 上电就是 USB 档：mode_cb 不会再收到通知，这里直接使能 */
		err = usbd_enable(&usb_ctx);
		if (err && (err != -EALREADY)) {
			LOG_ERR("USBD 使能失败: %d", err);
			return err;
		}
	}

	LOG_INF("USB HID 就绪（USB 控制器仅在 USB 档使能，插入后由主机枚举）");

	return 0;
}
