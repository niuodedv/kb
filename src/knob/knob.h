/*
 * 旋钮编码器（EC11）对外接口
 *
 * 本模块只负责「旋转方向 + 累计角度」这一层：
 *   读取 EC11 A/B 两相正交信号 -> 查表判方向、累计步数 -> 换算角度
 *   （EC11 自带的按压开关走矩阵 (0,3)，不在此模块处理）
 *
 * 详见 旋钮编码器.txt
 */

#ifndef KNOB_H_
#define KNOB_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 旋转方向 */
enum kb_knob_dir {
	KB_KNOB_NONE = 0,   /**< 无旋转 / 初始化 */
	KB_KNOB_CW,         /**< 正转（顺时针，符号以实测校准，见 旋钮编码器.txt K02） */
	KB_KNOB_CCW,        /**< 反转（逆时针） */
};

/**
 * @brief 旋转回调：每累计 1 档(detent) 触发一次
 *
 * @param dir    本档方向
 * @param step   本档净步数（恒为 +2 或 -2，等价于 ±1 档）
 * @param total  当前累计步数（带符号，跨圈连续；1 圈 = 60 步）
 * @param angle  当前累计角度（度，= total * 6，可超过 360）
 * @param user_data
 */
typedef void (*kb_knob_cb_t)(enum kb_knob_dir dir, int16_t step,
			     int32_t total, int32_t angle, void *user_data);

/**
 * @brief 初始化：配置 A/B 中断、上电定初态、安装回调入口
 *
 * @retval 0 成功
 * @retval -ENODEV 任一 GPIO 设备未就绪
 * @retval 其它 gpio 配置错误码
 */
int kb_knob_init(void);

/**
 * @brief 查询当前累计步数（非阻塞，原子读）
 * @return 带符号累计步数（1 圈 = 60 步）
 */
int32_t kb_knob_get_steps(void);

/**
 * @brief 查询当前累计角度（度），等同 steps * 6
 */
int32_t kb_knob_get_angle(void);

/**
 * @brief 清零累计（步数/角度归零，不改变硬件状态与初态）
 */
void kb_knob_reset(void);

/**
 * @brief 订阅旋转事件；同一回调只注册一次。
 * @retval 0 成功
 * @retval -EALREADY 已注册过同一回调
 * @retval -ENOMEM 回调槽已满
 */
int kb_knob_register_cb(kb_knob_cb_t cb, void *user_data);

/**
 * @brief 方向名（日志用）
 */
const char *kb_knob_dir_name(enum kb_knob_dir dir);

#ifdef __cplusplus
}
#endif

#endif /* KNOB_H_ */
