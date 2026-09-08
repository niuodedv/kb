/*
 * 上位机控制通道（USB CDC-ACM 虚拟串口）—— 对外接口
 *
 * 协议实现见 usb_host_cdc.c，主机侧工具为工程根目录的 host_cli.py。
 */
#ifndef KB_USB_HOST_CDC_H
#define KB_USB_HOST_CDC_H

/**
 * @brief 初始化上位机 CDC 控制通道
 *
 * 需在 kb_usb_hid_init() 之后调用（类已登记进 usbd 上下文）。
 * 通道随 USB 控制器门控：仅三模开关处于 USB 档时主机可见 COM 口。
 *
 * @retval 0 成功
 */
int kb_usb_host_cdc_init(void);

#endif /* KB_USB_HOST_CDC_H */
