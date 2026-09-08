/*
 * 上位机控制通道（USB CDC-ACM 虚拟串口）
 *
 * 主机侧：工程根目录 host_cli.py（Python / pyserial）。链路协议：
 *   帧：55 AA <len> <payload>            （仿 host_cli.make_frame，len<=64）
 *   payload：protobuf 变长编码的 DeviceMessage 子集
 *     field 10 (varint) : msg_id（应答回显）
 *     field  1 (bytes)  : hello_req  {1: protocol_version}
 *     field  2 (bytes)  : hello_rsp  {1: protocol_version, 2: vendor_id,
 *                                     3: product_id, 4: fw_major,
 *                                     5: fw_minor, 6: caps 位图}
 *     field  6 (bytes)  : time_sync  {1: version, 3: tz_offset_min
 *                                     (sint32 zigzag), 4: utc_ms (fixed64),
 *                                     5: accuracy_ms (fixed32)}
 *     field  7 (bytes)  : theme_rgb  {1: r, 2: g, 3: b}
 *     field  8 (bytes)  : response   {1: error_code}
 *   能力位：bit1 = 时间同步(TimeSync)、bit2 = RGB 主题色(ThemeRgb)。
 *
 * 时间语义：kb_time 把 epoch 秒直接按“本地日历”显示（LCD 第二行）。
 * 收到 utc_ms 后换算 本地epoch = utc_ms/1000 + tz_offset_min*60 再写入，
 * 与上位机打印的本地时间保持一致。tz_offset_min 东区为正（如 UTC+8=480）。
 *
 * 实现说明：CDC-ACM uart 的 fifo_read/fifo_fill 只允许在类驱动的内部
 * 工作队列上下文调用（见 usbd_cdc_acm.c check_wq_ctx），因此收帧/解析/
 * 应答全部在 IRQ 回调里完成，不上线程。命令量小、逻辑轻，符合该约束。
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "led/led.h"
#include "time/kb_time.h"
#include "usb_host_cdc.h"

LOG_MODULE_REGISTER(usb_host, LOG_LEVEL_INF);

/* ---------------- 协议常量（与 host_cli.py 严格对应） ---------------- */

#define FRAME_MAGIC0      0x55u
#define FRAME_MAGIC1      0xaau
#define FRAME_PAYLOAD_MAX 64u

/* protobuf wire types */
#define PB_WT_VARINT 0u
#define PB_WT_I64    1u
#define PB_WT_LEN    2u
#define PB_WT_I32    5u

/* DeviceMessage 顶层字段号 */
#define MSG_FIELD_MSG_ID    10u
#define MSG_FIELD_HELLO_REQ 1u
#define MSG_FIELD_HELLO_RSP 2u
#define MSG_FIELD_TIME_SYNC 6u
#define MSG_FIELD_THEME_RGB 7u
#define MSG_FIELD_RESPONSE  8u

/* 错误码（与 host_cli 一致） */
#define ERR_OK             0u
#define ERR_UNKNOWN_TYPE   1u
#define ERR_INVALID_LENGTH 2u
#define ERR_INVALID_PARAM  3u
#define ERR_NOT_READY      4u

/* hello_rsp 内容 */
#define PROTO_VERSION 1u
#define HOST_VID      0x2FE3u
#define HOST_PID      0x000Au
#define FW_MAJOR      1u
#define FW_MINOR      0u
/* caps 位图：bit1 TimeSync | bit2 ThemeRgb */
#define HOST_CAPS     0x06u

#define CDC_DEV DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0))

/* ---------------- 收帧状态机（单帧 ≤ 64B payload） ---------------- */

enum rx_state {
	RX_WAIT_M0,   /* 找 0x55 */
	RX_WAIT_M1,   /* 找 0xAA */
	RX_WAIT_LEN,  /* 读长度 */
	RX_PAYLOAD,   /* 收 payload */
};

static enum rx_state rx_st;
static uint8_t rx_len;
static uint8_t rx_idx;
static uint8_t rx_payload[FRAME_PAYLOAD_MAX];

/* ---------------- protobuf 解码（子集） ---------------- */

static int pb_varint(const uint8_t *p, uint16_t len, uint16_t *off, uint64_t *val)
{
	uint64_t v = 0;
	int shift = 0;

	while (*off < len) {
		uint8_t b = p[(*off)++];

		v |= (uint64_t)(b & 0x7fu) << shift;
		if (!(b & 0x80u)) {
			*val = v;
			return 0;
		}
		shift += 7;
		if (shift >= 64) {
			return -1;
		}
	}
	return -1;
}

static int pb_field(const uint8_t *p, uint16_t len, uint16_t *off,
		    uint8_t *field, uint8_t *wire)
{
	uint64_t tag;

	if (pb_varint(p, len, off, &tag) != 0) {
		return -1;
	}
	*field = (uint8_t)(tag >> 3);
	*wire = (uint8_t)(tag & 0x7u);
	return 0;
}

static int pb_skip(const uint8_t *p, uint16_t len, uint16_t *off, uint8_t wire)
{
	uint64_t dummy;

	switch (wire) {
	case PB_WT_VARINT:
		return pb_varint(p, len, off, &dummy);
	case PB_WT_I64:
		if (len - *off < 8) {
			return -1;
		}
		*off += 8;
		return 0;
	case PB_WT_I32:
		if (len - *off < 4) {
			return -1;
		}
		*off += 4;
		return 0;
	case PB_WT_LEN: {
		uint64_t n;

		if (pb_varint(p, len, off, &n) != 0 || n > len - *off) {
			return -1;
		}
		*off += (uint16_t)n;
		return 0;
	}
	default:
		return -1;
	}
}

/* ---------------- protobuf 编码（子集） ---------------- */

static void pb_put_varint(uint8_t *buf, uint16_t *off, uint8_t field, uint64_t v)
{
	uint8_t tag = (uint8_t)((field << 3) | PB_WT_VARINT);

	buf[(*off)++] = tag;
	while (v >= 0x80u) {
		buf[(*off)++] = (uint8_t)((v & 0x7fu) | 0x80u);
		v >>= 7;
	}
	buf[(*off)++] = (uint8_t)v;
}

static void pb_put_bytes(uint8_t *buf, uint16_t *off, uint8_t field,
			 const uint8_t *data, uint8_t n)
{
	uint8_t tag = (uint8_t)((field << 3) | PB_WT_LEN);

	buf[(*off)++] = tag;
	buf[(*off)++] = n;
	if (n > 0) {
		memcpy(&buf[*off], data, n);
		*off += n;
	}
}

/* 解析 time_sync 子消息 */
static int parse_time_sync(const uint8_t *p, uint16_t len, int32_t *tz_min,
			   uint64_t *utc_ms, bool *has_utc)
{
	uint16_t off = 0;

	*tz_min = 0;
	*utc_ms = 0;
	*has_utc = false;

	while (off < len) {
		uint8_t field, wire;
		uint64_t v;

		if (pb_field(p, len, &off, &field, &wire) != 0) {
			return -1;
		}

		if (field == 1 && wire == PB_WT_VARINT) {
			/* version：忽略，仅当前版本协议 */
			if (pb_varint(p, len, &off, &v) != 0) {
				return -1;
			}
		} else if (field == 3 && wire == PB_WT_VARINT) {
			/* tz_offset_min：sint32 zigzag */
			uint64_t z;

			if (pb_varint(p, len, &off, &z) != 0) {
				return -1;
			}
			*tz_min = (int32_t)((z >> 1) ^ -(int64_t)(z & 1));
		} else if (field == 4 && wire == PB_WT_I64) {
			uint64_t le = 0;
			int i;

			if (len - off < 8) {
				return -1;
			}
			for (i = 7; i >= 0; i--) {
				le = (le << 8) | p[off + (uint16_t)i];
			}
			off += 8;
			*utc_ms = le;
			*has_utc = true;
		} else if (pb_skip(p, len, &off, wire) != 0) {
			return -1;
		}
	}

	return 0;
}

/* 解析 theme_rgb 子消息；rgb 越界返回 -EINVAL */
static int parse_theme_rgb(const uint8_t *p, uint16_t len,
			   uint8_t *r, uint8_t *g, uint8_t *b)
{
	uint16_t off = 0;
	bool have_r = false, have_g = false, have_b = false;

	while (off < len) {
		uint8_t field, wire;
		uint64_t v;

		if (pb_field(p, len, &off, &field, &wire) != 0) {
			return -1;
		}

		if ((field >= 1 && field <= 3) && wire == PB_WT_VARINT) {
			if (pb_varint(p, len, &off, &v) != 0 || v > 255) {
				return -EINVAL;
			}
			if (field == 1) {
				*r = (uint8_t)v;
				have_r = true;
			} else if (field == 2) {
				*g = (uint8_t)v;
				have_g = true;
			} else {
				*b = (uint8_t)v;
				have_b = true;
			}
		} else if (pb_skip(p, len, &off, wire) != 0) {
			return -1;
		}
	}

	return (have_r && have_g && have_b) ? 0 : -EINVAL;
}

/* ---------------- 应答 ---------------- */

static void frame_send(const uint8_t *payload, uint16_t plen)
{
	uint8_t frame[3 + FRAME_PAYLOAD_MAX];

	if (plen > FRAME_PAYLOAD_MAX) {
		plen = FRAME_PAYLOAD_MAX;
	}
	frame[0] = FRAME_MAGIC0;
	frame[1] = FRAME_MAGIC1;
	frame[2] = (uint8_t)plen;
	memcpy(&frame[3], payload, plen);

	if (!device_is_ready(CDC_DEV)) {
		LOG_WRN("CDC 设备未就绪，应答丢弃");
		return;
	}
	(void)uart_fifo_fill(CDC_DEV, frame, 3u + plen);
}

/* response{ error_code } */
static void send_response(uint64_t msg_id, uint8_t err)
{
	uint8_t payload[16];
	uint16_t off = 0;
	uint8_t body[4];
	uint16_t boff = 0;

	pb_put_varint(body, &boff, 1, err);   /* error_code */
	pb_put_varint(payload, &off, MSG_FIELD_MSG_ID, msg_id);
	pb_put_bytes(payload, &off, MSG_FIELD_RESPONSE, body, boff);

	frame_send(payload, off);
}

/* hello_rsp{ ... } */
static void send_hello_rsp(uint64_t msg_id)
{
	uint8_t payload[40];
	uint16_t off = 0;
	uint8_t body[24];
	uint16_t boff = 0;

	pb_put_varint(body, &boff, 1, PROTO_VERSION);
	pb_put_varint(body, &boff, 2, HOST_VID);
	pb_put_varint(body, &boff, 3, HOST_PID);
	pb_put_varint(body, &boff, 4, FW_MAJOR);
	pb_put_varint(body, &boff, 5, FW_MINOR);
	pb_put_varint(body, &boff, 6, HOST_CAPS);

	pb_put_varint(payload, &off, MSG_FIELD_MSG_ID, msg_id);
	pb_put_bytes(payload, &off, MSG_FIELD_HELLO_RSP, body, boff);

	frame_send(payload, off);
}

/* ---------------- 帧处理 ---------------- */

static void handle_frame(const uint8_t *pl, uint16_t plen)
{
	uint16_t off = 0;
	uint64_t msg_id = 0;
	bool has_hello = false, has_time = false, has_theme = false;
	const uint8_t *hello_p = NULL;
	const uint8_t *time_p = NULL;
	const uint8_t *theme_p = NULL;
	uint16_t hello_len = 0, time_len = 0, theme_len = 0;

	while (off < plen) {
		uint8_t field, wire;
		uint64_t sub_len;

		if (pb_field(pl, plen, &off, &field, &wire) != 0) {
			return;   /* 截断/乱序，丢弃整帧 */
		}

		if (field == MSG_FIELD_MSG_ID && wire == PB_WT_VARINT) {
			if (pb_varint(pl, plen, &off, &msg_id) != 0) {
				return;
			}
		} else if (wire == PB_WT_LEN &&
			   (field == MSG_FIELD_HELLO_REQ ||
			    field == MSG_FIELD_TIME_SYNC ||
			    field == MSG_FIELD_THEME_RGB)) {
			if (pb_varint(pl, plen, &off, &sub_len) != 0 ||
			    sub_len > plen - off) {
				return;
			}
			if (field == MSG_FIELD_HELLO_REQ) {
				hello_p = &pl[off];
				hello_len = (uint16_t)sub_len;
				has_hello = true;
			} else if (field == MSG_FIELD_TIME_SYNC) {
				time_p = &pl[off];
				time_len = (uint16_t)sub_len;
				has_time = true;
			} else {
				theme_p = &pl[off];
				theme_len = (uint16_t)sub_len;
				has_theme = true;
			}
			off += (uint16_t)sub_len;
		} else if (pb_skip(pl, plen, &off, wire) != 0) {
			return;
		}
	}

	if (has_hello) {
		send_hello_rsp(msg_id);
		return;
	}

	if (has_time) {
		int32_t tz_min;
		uint64_t utc_ms;
		bool has_utc;
		int64_t epoch;

		if (parse_time_sync(time_p, time_len, &tz_min, &utc_ms,
				    &has_utc) != 0) {
			send_response(msg_id, ERR_INVALID_PARAM);
			return;
		}
		if (!has_utc || (utc_ms / 1000u) > UINT32_MAX) {
			send_response(msg_id, ERR_INVALID_PARAM);
			return;
		}

		/* 本地 epoch = UTC 秒 + 时区偏移分钟 */
		epoch = (int64_t)(utc_ms / 1000u) + (int64_t)tz_min * 60;
		if (epoch < 0 || epoch > UINT32_MAX) {
			send_response(msg_id, ERR_INVALID_PARAM);
			return;
		}

		kb_time_set_epoch((uint32_t)epoch);
		LOG_INF("时间同步成功（本地 epoch=%lld, tz=%d min）",
			(long long)epoch, (int32_t)tz_min);
		send_response(msg_id, ERR_OK);
		return;
	}

	if (has_theme) {
		uint8_t r, g, b;

		if (parse_theme_rgb(theme_p, theme_len, &r, &g, &b) != 0) {
			send_response(msg_id, ERR_INVALID_PARAM);
			return;
		}

		kb_led_set_color(r, g, b);
		LOG_INF("RGB 主题色设置: #%02X%02X%02X", r, g, b);
		send_response(msg_id, ERR_OK);
		return;
	}

	send_response(msg_id, ERR_UNKNOWN_TYPE);
}

/* ---------------- CDC uart IRQ 回调（驱动工作队列上下文） ---------------- */

static void rx_feed(uint8_t b)
{
	switch (rx_st) {
	case RX_WAIT_M0:
		if (b == FRAME_MAGIC0) {
			rx_st = RX_WAIT_M1;
		}
		break;

	case RX_WAIT_M1:
		if (b == FRAME_MAGIC1) {
			rx_st = RX_WAIT_LEN;
		} else if (b != FRAME_MAGIC0) {
			rx_st = RX_WAIT_M0;
		}
		break;

	case RX_WAIT_LEN:
		rx_len = b;
		rx_idx = 0;
		if (rx_len > FRAME_PAYLOAD_MAX) {
			LOG_WRN("帧超长(%u)，重同步", (unsigned)rx_len);
			rx_st = RX_WAIT_M0;
		} else if (rx_len == 0) {
			handle_frame(NULL, 0);
			rx_st = RX_WAIT_M0;
		} else {
			rx_st = RX_PAYLOAD;
		}
		break;

	case RX_PAYLOAD:
		rx_payload[rx_idx++] = b;
		if (rx_idx == rx_len) {
			handle_frame(rx_payload, rx_len);
			rx_st = RX_WAIT_M0;
		}
		break;
	}
}

static void cdc_uart_irq_cb(const struct device *dev, void *user_data)
{
	uint8_t buf[FRAME_PAYLOAD_MAX];
	int n;

	(void)user_data;

	if (!uart_irq_update(dev)) {
		return;
	}
	if (!uart_irq_is_pending(dev)) {
		return;
	}

	if (uart_irq_rx_ready(dev)) {
		while ((n = uart_fifo_read(dev, buf, sizeof(buf))) > 0) {
			for (int i = 0; i < n; i++) {
				rx_feed(buf[i]);
			}
		}
	}
}

/* ---------------- 初始化 ---------------- */

int kb_usb_host_cdc_init(void)
{
	if (!device_is_ready(CDC_DEV)) {
		LOG_ERR("上位机 CDC-ACM 设备未就绪");
		return -ENODEV;
	}

	/* 清掉复位前可能残留的半帧 */
	rx_st = RX_WAIT_M0;
	rx_len = 0;
	rx_idx = 0;

	uart_irq_callback_set(CDC_DEV, cdc_uart_irq_cb);
	uart_irq_rx_enable(CDC_DEV);

	LOG_INF("上位机 CDC 通道就绪（host_cli.py 协议，VID 0x%04X PID 0x%04X）",
		(unsigned)HOST_VID, (unsigned)HOST_PID);
	return 0;
}
