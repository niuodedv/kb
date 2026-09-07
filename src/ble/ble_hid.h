/*
 * BLE HID 模块对外接口
 *
 * 本模块只负责「BLE 这一路怎么发」：
 *   - 矩阵按键           -> HID Keyboard Report（8 字节：修饰键 + 保留 + 6 键值）
 *   - EC11 旋转         -> HID Consumer Report（系统音量加减）
 *   - EC11 自带按键(Mute)-> HID Consumer Report（静音）
 *
 * 是否发送由「模式门控」决定：仅当 kb_mode_get() == KB_MODE_BLE 时才从 BLE 发出
 * （对应 BLE通信.txt 第 8 章）。后续若引入 hid_bridge 统一路由，
 * 把门控上移到 hid_bridge 即可，本模块的发送接口不用改。
 *
 * INPUT_KEY_* 标准键值 -> HID 用法 的翻译、NumLock 双功能层、音量映射见实现文件。
 *
 * 详见 BLE通信.txt
 */

#ifndef BLE_HID_H_
#define BLE_HID_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 BLE HID
 *
 * 依次完成：HIDS 服务注册 -> bt_enable() -> 加载绑定信息 -> 订阅模式与旋钮回调
 *          -> 若当前已是 BLE 模式则立即开始广播。
 *
 * @retval 0 成功
 * @retval 负数 bt_enable / 回调注册失败
 *
 * @note 必须在 kb_mode_init() 之后调用：初始化时会读一次当前模式决定是否广播。
 */
int kb_ble_hid_init(void);

/** @brief 当前是否已有 BLE 主机连接并订阅 HID 报告 */
bool kb_ble_hid_is_connected(void);

/** @brief 当前是否正在广播（可被电脑搜索到） */
bool kb_ble_hid_is_advertising(void);

/** @brief 当前 NumLock 状态（由主机 LED Output Report 同步，默认开） */
bool kb_ble_hid_numlock_get(void);

/**
 * @brief 主动进入配对模式
 *
 * 清除本端全部绑定 -> 断开现有连接 -> 重新广播（无绑定 = 配对模式）。
 * 状态灯转为蓝灯快闪；已配对的电脑被解绑，需在其蓝牙设置里删除设备后
 * 重新搜索配对。触发方式：长按 NumLock 3 秒，或 shell 命令 `ble pair`。
 *
 * @retval 0 成功
 */
int kb_ble_hid_enter_pairing(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_HID_H_ */
