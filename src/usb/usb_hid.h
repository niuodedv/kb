/*
 * USB HID 模块对外接口（三模开关 USB 档）
 *
 * USB 档下：矩阵按键 -> USB HID Keyboard 报告（9 字节，含 Report ID），
 *           EC11 旋钮/静音 -> Consumer 报告（3 字节，含 Report ID），
 *           主机 NumLock LED Output Report -> 公共层同步。
 *
 * 方案详见 usb 通信.txt
 */

#ifndef USB_HID_H_
#define USB_HID_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 USB HID
 *
 * 注册 HID 类与报告描述符、组装 usbd 上下文并 usbd_enable()。
 * 板上无 VBUS 检测引脚，插入 USB 线后由主机完成枚举；
 * 按键是否发送由三模开关档位（KB_MODE_USB）门控，与枚举无关。
 *
 * @retval 0 成功
 * @retval 负数 设备未就绪 / usbd 组装失败
 */
int kb_usb_hid_init(void);

/**
 * @brief 释放全部按键（断电/关机前调用，尽力而为）
 *
 * 清空公共 HID 键盘状态、丢弃队列中残留报文，并在 USB 就绪时补发一帧
 * 全 0 键盘报告，避免主机残留按键。本接口不切模式、不使能/关闭 USB。
 */
void kb_usb_hid_release_all(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_HID_H_ */
