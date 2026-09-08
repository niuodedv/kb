#!/usr/bin/env python3
"""
Mini Keyboard 命令行上位机工具

用法:
  python host_cli.py <COM口> time             同步系统时间到设备
  python host_cli.py <COM口> rgb <R> <G> <B>  设置 RGB 主题颜色

示例:
  python host_cli.py COM3 time
  python host_cli.py COM3 time --tz 480
  python host_cli.py COM3 rgb 255 0 0
  python host_cli.py COM3 rgb 76 158 245
"""

import struct
import sys
import time as time_mod
from datetime import datetime, timezone

try:
    import serial
except ImportError:
    print("请先安装 pyserial: pip install pyserial")
    sys.exit(1)

# =============================================================================
# 协议常量
# =============================================================================
FRAME_MAGIC = 0xAA55          # 帧魔数
FRAME_HEADER_SIZE = 3         # magic(2) + len(1)
MAX_PAYLOAD_LEN = 64          # 最大载荷长度
PROTOCOL_VERSION = 1           # 协议版本
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 5.0           # 串口读取超时
SERIAL_WRITE_TIMEOUT = 3.0

# =============================================================================
# Protobuf 手动编码（避免依赖 protobuf 编译器）
# =============================================================================

def pb_varint(value: int) -> bytes:
    """编码无符号 varint"""
    result = []
    v = value & 0xFFFFFFFF
    while v > 0x7F:
        result.append((v & 0x7F) | 0x80)
        v >>= 7
    result.append(v & 0x7F)
    return bytes(result)


def pb_zigzag(value: int) -> int:
    """sint32 zigzag 编码: (n << 1) ^ (n >> 31)"""
    return (value << 1) ^ (value >> 31)


def pb_tag(field_number: int, wire_type: int) -> int:
    """protobuf tag: (field_number << 3) | wire_type"""
    return (field_number << 3) | wire_type


WIRE_VARINT = 0
WIRE_LEN = 2
WIRE_FIXED64 = 1
WIRE_FIXED32 = 5


def pb_field_varint(field_number: int, value: int) -> bytes:
    return pb_varint(pb_tag(field_number, WIRE_VARINT)) + pb_varint(value)


def pb_field_len(field_number: int, data: bytes) -> bytes:
    return pb_varint(pb_tag(field_number, WIRE_LEN)) + pb_varint(len(data)) + data


def pb_field_fixed64(field_number: int, value: int) -> bytes:
    return pb_varint(pb_tag(field_number, WIRE_FIXED64)) + struct.pack("<Q", value)


def pb_field_fixed32(field_number: int, value: int) -> bytes:
    return pb_varint(pb_tag(field_number, WIRE_FIXED32)) + struct.pack("<I", value)


# =============================================================================
# 帧操作
# =============================================================================

def make_frame(payload: bytes) -> bytes:
    """构造帧: magic(0x55 0xAA) + len(1B) + payload"""
    if len(payload) > MAX_PAYLOAD_LEN:
        raise ValueError(f"Payload too long: {len(payload)} > {MAX_PAYLOAD_LEN}")
    return struct.pack("<H", FRAME_MAGIC) + bytes([len(payload)]) + payload


def read_raw(ser: serial.Serial, timeout: float = 0.5) -> bytes:
    """读取串口所有可用数据 (用于调试)"""
    import time
    data = b""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            data += chunk
    return data


def read_frame(ser: serial.Serial, debug: bool = False) -> tuple[bytes, bytes] | None:
    """从串口读取一帧, 返回 (raw_frame, payload) 或 None"""
    magic_bytes = struct.pack("<H", FRAME_MAGIC)  # 0x55 0xAA
    sync_buf = b""
    scanned = 0
    while True:
        ch = ser.read(1)
        if not ch:
            if debug and scanned:
                print(f"[DEBUG] 魔数扫描超时, 已扫描 {scanned} 字节")
            return None
        scanned += 1
        sync_buf += ch
        if sync_buf.endswith(magic_bytes):
            if debug and scanned > 2:
                print(f"[DEBUG] 魔数匹配, 跳过了 {scanned - 2} 字节")
            sync_buf = b""
            break
        if len(sync_buf) >= 2:
            sync_buf = sync_buf[-1:]

    len_byte = ser.read(1)
    if not len_byte:
        if debug:
            print("[DEBUG] 未收到帧长度字节")
        return None
    payload_len = len_byte[0]
    if payload_len > MAX_PAYLOAD_LEN:
        print(f"[WARN] 收到超长帧 len={payload_len}, 跳过")
        return None

    payload = ser.read(payload_len)
    if len(payload) < payload_len:
        if debug:
            print(f"[DEBUG] 帧载荷不完整: 期望 {payload_len}, 收到 {len(payload)}")
        return None

    return (magic_bytes + len_byte + payload, payload)


# =============================================================================
# 消息构造
# =============================================================================

_msg_id_counter = 1


def next_msg_id() -> int:
    global _msg_id_counter
    mid = _msg_id_counter
    _msg_id_counter += 1
    return mid


def encode_hello_req(msg_id: int) -> bytes:
    """DeviceMessage { msg_id, hello_req { protocol_version: 1 } }"""
    body = pb_field_varint(1, PROTOCOL_VERSION)  # hello_req.protocol_version
    msg = b""
    msg += pb_field_varint(10, msg_id)            # msg_id
    msg += pb_field_len(1, body)                  # hello_req (field 1)
    return msg


def encode_time_sync(msg_id: int, tz_min: int, utc_ms: int, accuracy_ms: int = 0) -> bytes:
    """DeviceMessage { msg_id, time_sync { version, flags, timezone_min, utc_ms, accuracy_ms } }"""
    body = b""
    body += pb_field_varint(1, 1)                  # version = 1
    body += pb_field_varint(3, pb_zigzag(tz_min))  # timezone_min (sint32, zigzag)
    body += pb_field_fixed64(4, utc_ms)            # utc_ms (fixed64)
    body += pb_field_fixed32(5, accuracy_ms)       # accuracy_ms (fixed32)
    msg = b""
    msg += pb_field_varint(10, msg_id)             # msg_id
    msg += pb_field_len(6, body)                   # time_sync (field 6)
    return msg


def encode_theme_rgb(msg_id: int, r: int, g: int, b: int) -> bytes:
    """DeviceMessage { msg_id, theme_rgb { red, green, blue } }"""
    body = b""
    body += pb_field_varint(1, r)   # red
    body += pb_field_varint(2, g)   # green
    body += pb_field_varint(3, b)   # blue
    msg = b""
    msg += pb_field_varint(10, msg_id)  # msg_id
    msg += pb_field_len(7, body)        # theme_rgb (field 7)
    return msg


# =============================================================================
# 简易响应解码
# =============================================================================

def decode_varint(data: bytes, offset: int) -> tuple[int, int]:
    """解码 varint, 返回 (value, new_offset)"""
    value = 0
    shift = 0
    while offset < len(data):
        byte = data[offset]
        value |= (byte & 0x7F) << shift
        offset += 1
        if not (byte & 0x80):
            break
        shift += 7
    return value, offset


def decode_response(payload: bytes) -> dict:
    """简易 protobuf 解码 DeviceMessage, 返回字段字典"""
    result = {}
    offset = 0
    while offset < len(payload):
        tag, offset = decode_varint(payload, offset)
        field_number = tag >> 3
        wire_type = tag & 0x07

        if wire_type == WIRE_VARINT:
            value, offset = decode_varint(payload, offset)
            result[field_number] = value
        elif wire_type == WIRE_LEN:
            length, offset = decode_varint(payload, offset)
            data = payload[offset:offset + length]
            offset += length
            # 递归解码子消息
            if field_number in (1, 2, 6, 7, 8):  # hello_rsp, response等
                result[field_number] = decode_response(data)
            else:
                result[field_number] = data
        elif wire_type == WIRE_FIXED64:
            result[field_number] = struct.unpack("<Q", payload[offset:offset + 8])[0]
            offset += 8
        elif wire_type == WIRE_FIXED32:
            result[field_number] = struct.unpack("<I", payload[offset:offset + 4])[0]
            offset += 4
        else:
            break
    return result


# =============================================================================
# 会话管理
# =============================================================================

def handshake(ser: serial.Serial, debug: bool = False) -> bool:
    """发送 HelloReq, 等待 HelloRsp"""
    msg_id = next_msg_id()
    payload = encode_hello_req(msg_id)
    frame = make_frame(payload)

    print(f"[→] 发送 HelloReq (msg_id={msg_id}) ", end="")
    if debug:
        print(f"frame={frame.hex()}")
    else:
        print()

    # 发送前清空输入缓冲
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()

    result = read_frame(ser, debug=debug)
    if result is None:
        print("[-] 未收到 HelloRsp 响应")
        if debug:
            # 再检查一次是否有任何残留数据
            raw = read_raw(ser, 0.3)
            if raw:
                print(f"[DEBUG] 但收到原始数据 ({len(raw)} 字节): {raw.hex()}")
                print(f"[DEBUG] 可能是帧格式不匹配, 请检查编码")
        return False

    raw_frame, rsp_payload = result
    if debug:
        print(f"[DEBUG] 收到帧 (hex): {raw_frame.hex()}")
        print(f"[DEBUG] 载荷 (hex): {rsp_payload.hex()}")

    rsp = decode_response(rsp_payload)
    if debug:
        print(f"[DEBUG] 解码字段: {list(rsp.keys())}")

    if 2 in rsp:
        hr = rsp[2]
        print(f"[+] HelloRsp 收到:")
        print(f"      协议版本: {hr.get(1, '?')}")
        print(f"      厂商 ID:   0x{hr.get(2, 0):04X}")
        print(f"      产品 ID:   0x{hr.get(3, 0):04X}")
        print(f"      固件版本:  v{hr.get(4, 0)}.{hr.get(5, 0)}")
        caps = hr.get(6, 0)
        print(f"      能力标志:  0x{caps:02X} ", end="")
        cap_names = {0: "Bitmap", 1: "TimeSync", 2: "ThemeRgb", 3: "LedState", 4: "FnKeyEvent"}
        active = [cap_names[i] for i in range(5) if caps & (1 << i)]
        print("(" + ", ".join(active) + ")" if active else "(无)")
        return True
    elif 8 in rsp:
        resp = rsp[8]
        error_codes = {0: "OK", 1: "UNKNOWN_TYPE", 2: "INVALID_LENGTH",
                       3: "INVALID_PARAM", 4: "NOT_READY"}
        print(f"[-] 收到错误响应: {error_codes.get(resp.get(1, -1), 'UNKNOWN')}")
        return False
    else:
        print(f"[-] 未识别的响应: {rsp}")
        return False


# =============================================================================
# 命令实现
# =============================================================================

def cmd_time(ser: serial.Serial, tz_min: int, debug: bool = False) -> bool:
    """同步系统时间到设备"""
    if not handshake(ser, debug=debug):
        return False

    now = datetime.now(timezone.utc)
    utc_ms = int(now.timestamp() * 1000)
    msg_id = next_msg_id()

    payload = encode_time_sync(msg_id, tz_min, utc_ms)
    frame = make_frame(payload)

    from datetime import timedelta
    local_dt = datetime.now()
    print(f"[→] 发送 TimeSync (msg_id={msg_id})")
    print(f"      UTC:  {datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M:%S')}.{utc_ms % 1000:03d}")
    print(f"      本地: {local_dt.strftime('%Y-%m-%d %H:%M:%S')} (偏移 {tz_min} 分钟)")
    ser.write(frame)
    ser.flush()

    result = read_frame(ser)
    if result is None:
        print("[-] 未收到响应")
        return False

    _, rsp_payload = result
    rsp = decode_response(rsp_payload)

    if 8 in rsp:
        # proto3 默认值省略: error_code=0 时字段不存在, 取 0 即为 OK
        err = rsp[8].get(1, 0) if isinstance(rsp[8], dict) else 0
        if err == 0:
            print("[+] 时间同步成功")
            return True
        else:
            error_names = {0: "OK", 1: "UNKNOWN_TYPE", 2: "INVALID_LENGTH",
                           3: "INVALID_PARAM", 4: "NOT_READY"}
            print(f"[-] 时间同步失败: {error_names.get(err, f'UNKNOWN({err})')}")
            return False
    else:
        print(f"[-] 未识别的响应")
        return False


def cmd_rgb(ser: serial.Serial, r: int, g: int, b: int, debug: bool = False) -> bool:
    """设置 RGB 主题颜色"""
    if not handshake(ser, debug=debug):
        return False

    msg_id = next_msg_id()
    payload = encode_theme_rgb(msg_id, r, g, b)
    frame = make_frame(payload)

    print(f"[→] 发送 ThemeRgb (msg_id={msg_id}) R={r} G={g} B={b}")
    ser.write(frame)
    ser.flush()

    result = read_frame(ser)
    if result is None:
        print("[-] 未收到响应")
        return False

    _, rsp_payload = result
    rsp = decode_response(rsp_payload)

    if 8 in rsp:
        err = rsp[8].get(1, 0) if isinstance(rsp[8], dict) else 0
        if err == 0:
            print(f"[+] 主题色设置成功 (R={r}, G={g}, B={b})")
            return True
        else:
            error_names = {0: "OK", 1: "UNKNOWN_TYPE", 2: "INVALID_LENGTH",
                           3: "INVALID_PARAM", 4: "NOT_READY"}
            print(f"[-] 设置失败: {error_names.get(err, f'UNKNOWN({err})')}")
            return False
    else:
        print(f"[-] 未识别的响应")
        return False


def cmd_scan() -> list[str]:
    """扫描所有 COM 口, 找出有 HelloRsp 响应的端口"""
    import serial.tools.list_ports

    ports = serial.tools.list_ports.comports()
    found = []
    for port_info in ports:
        port = port_info.device
        vid = f"VID:{port_info.vid:04X}" if port_info.vid else ""
        pid = f"PID:{port_info.pid:04X}" if port_info.pid else ""
        desc = port_info.description or ""

        # 只看可能的 CDC/串口设备
        print(f"[*] 尝试 {port} ({vid} {pid} {desc[:40]}) ... ", end="", flush=True)
        try:
            ser = serial.Serial(port, SERIAL_BAUDRATE,
                               timeout=1.0, write_timeout=SERIAL_WRITE_TIMEOUT)
            ser.dtr = True
        except Exception as e:
            print(f"打不开 ({e})")
            continue

        try:
            msg_id = 1
            payload = encode_hello_req(msg_id)
            frame = make_frame(payload)
            ser.reset_input_buffer()
            ser.write(frame)
            ser.flush()
        except Exception as e:
            print(f"写入失败 ({e})")
            ser.close()
            continue

        result = read_frame(ser, debug=False)
        ser.close()

        if result is not None:
            _, rsp_payload = result
            rsp = decode_response(rsp_payload)
            if 2 in rsp:
                hr = rsp[2]
                print(f"+ 设备响应! VID=0x{hr.get(2,0):04X} PID=0x{hr.get(3,0):04X}")
                found.append(port)
            else:
                print("收到数据但不是 HelloRsp")
        else:
            print("无响应")

    return found


# =============================================================================
# CLI 入口
# =============================================================================

def print_usage():
    print(__doc__)
    print("当前可用命令:")
    print("  scan              扫描所有 COM 口, 找到键盘设备")
    print("  time              同步操作系统时间到设备")
    print("  rgb <R> <G> <B>   设置 RGB 主题颜色 (0-255)")
    print()
    print("选项:")
    print("  --tz <分钟>       时区偏移(分钟), 默认 480 (UTC+8)")
    print("                    例如 UTC+8 = 480, UTC+0 = 0, UTC-5 = -300")
    print("  --debug           打印调试信息 (原始字节、响应详情)")
    print("  --no-handshake    跳过握手, 直接发送命令")


def parse_args(argv: list[str]) -> dict:
    """解析命令行参数"""
    args = {
        "port": None,
        "command": None,
        "r": 0, "g": 0, "b": 0,
        "tz_min": 480,  # 默认 UTC+8 (中国), 正值=东区
        "no_handshake": False,
        "debug": False,
    }

    i = 1
    while i < len(argv):
        arg = argv[i]
        if arg == "--tz":
            i += 1
            args["tz_min"] = int(argv[i])
        elif arg == "--no-handshake":
            args["no_handshake"] = True
        elif arg == "--debug":
            args["debug"] = True
        elif arg == "scan":
            args["command"] = "scan"
        elif args["port"] is None:
            args["port"] = arg
        elif args["command"] is None:
            args["command"] = arg
            if arg == "rgb":
                args["r"] = int(argv[i + 1])
                args["g"] = int(argv[i + 2])
                args["b"] = int(argv[i + 3])
                i += 3
        i += 1

    return args


def main():
    args = parse_args(sys.argv)

    # scan 命令不需要 COM 口
    cmd = args["command"]
    if cmd == "scan":
        print(f"[*] 扫描所有 COM 口...")
        found = cmd_scan()
        if found:
            print(f"\n[+] 找到 {len(found)} 个设备: {', '.join(found)}")
            print(f"    使用方式: python host_cli.py {found[0]} time")
        else:
            print("\n[-] 未找到设备 — 请检查:")
            print("    1. 键盘是否已插入 USB")
            print("    2. 模式开关是否拨到 USB 档")
            print("    3. 设备管理器是否能看到两个新 COM 口")
        sys.exit(0 if found else 1)

    if len(sys.argv) < 3:
        print_usage()
        sys.exit(1)

    # 验证端口
    port = args["port"]
    if not port.startswith("COM") and not port.startswith("/dev/"):
        if port.isdigit():
            port = f"COM{port}"

    # 打开串口
    print(f"[*] 打开串口: {port}, 波特率: {SERIAL_BAUDRATE}")
    try:
        ser = serial.Serial(port, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT)
    except serial.SerialException as e:
        print(f"[-] 无法打开串口 {port}: {e}")
        sys.exit(1)

    # CDC ACM 必须置 DTR
    ser.dtr = True
    print(f"[*] DTR 已置位")

    # 等待设备处理 DTR 并完成状态机转换:
    #   BUS_OFFLINE → WAIT_DTR → SESSION_READY → LINK_READY → WAIT_HELLO
    time_mod.sleep(1.0)
    print(f"[*] 等待设备就绪 (1s)")

    # 执行命令
    success = False

    if cmd == "time":
        print(f"[*] 命令: 同步系统时间 (时区偏移 {args['tz_min']} 分钟)")
        success = cmd_time(ser, args["tz_min"], debug=args["debug"])
    elif cmd == "rgb":
        r, g, b = args["r"], args["g"], args["b"]
        if not (0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255):
            print("[-] RGB 值必须在 0-255 范围内")
            ser.close()
            sys.exit(1)
        print(f"[*] 命令: 设置主题色 R={r} G={g} B={b}")
        success = cmd_rgb(ser, r, g, b, debug=args["debug"])
    else:
        print(f"[-] 未知命令: {cmd}")
        print_usage()
        ser.close()
        sys.exit(1)

    ser.close()
    print(f"\n[*] {'+ 完成' if success else '- 失败'}")
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
