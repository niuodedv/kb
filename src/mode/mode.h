/*
 * 模式检测模块对外接口（三模开关 ADC 判档）
 *
 * 本模块只负责「检测与标记」：
 *   读 MODE 引脚电压 -> 判档 -> 对外提供查询与变化通知。
 * 不实现 USB / BLE / 2.4G 的任何通信，接口留给后续模块订阅。
 *
 * 详见 模式切换.txt
 */

#ifndef MODE_H_
#define MODE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 工作模式
 *
 * 数值仅为枚举标识，不要用它的整数值做业务判断。
 */
enum kb_mode {
	KB_MODE_UNKNOWN = 0,   /**< 未确定：尚未采样到有效档，或开关停在两档之间 */
	KB_MODE_USB,           /**< USB 有线 */
	KB_MODE_24G,           /**< 2.4G（本项目只需检测并标记，LCD 显示即可） */
	KB_MODE_BLE,           /**< 蓝牙 */
};

/**
 * @brief 模式变化回调
 *
 * @param prev 变化前的模式
 * @param now  变化后的模式
 * @param user_data 注册时传入的用户数据
 *
 * @note 在【工作队列上下文】执行，回调内不要做阻塞操作；
 *       需要重活的订阅者应自行再投递事件。
 */
typedef void (*kb_mode_cb_t)(enum kb_mode prev, enum kb_mode now, void *user_data);

/**
 * @brief 初始化模式检测
 *
 * 配置 SAADC 通道、上电立即定档、启动周期轮询。
 *
 * @retval 0 成功
 * @retval -ENODEV SAADC 设备未就绪
 * @retval 其他负值 ADC 通道配置或采样失败
 */
int kb_mode_init(void);

/**
 * @brief 查询当前模式
 *
 * 非阻塞，可在任何上下文调用（内部是单字长读取，无需加锁）。
 */
enum kb_mode kb_mode_get(void);

/**
 * @brief 订阅模式变化
 *
 * 同一回调只注册一次；容量见实现（当前 4 个）。
 *
 * @retval 0 成功
 * @retval -EINVAL 回调为空
 * @retval -EALREADY 该回调已注册
 * @retval -ENOMEM 回调槽已满
 */
int kb_mode_register_cb(kb_mode_cb_t cb, void *user_data);

/**
 * @brief 强制设定模式（调试 / 上位机用）
 *
 * 传入非 KB_MODE_UNKNOWN 时覆盖硬件检测结果并停止自动判档；
 * 传入 KB_MODE_UNKNOWN 时解除覆盖，恢复由 ADC 自动检测。
 */
int kb_mode_override(enum kb_mode mode);

/**
 * @brief 读一次 MODE 引脚电压，用于标定与调试
 *
 * @param mv  输出毫伏，可为 NULL
 * @param raw 输出原始码值，可为 NULL
 */
int kb_mode_read(int32_t *mv, int32_t *raw);

/** @brief 模式名（日志用） */
const char *kb_mode_name(enum kb_mode mode);

#ifdef __cplusplus
}
#endif

#endif /* MODE_H_ */
