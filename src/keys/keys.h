/*
 * 按键模块对外接口
 *
 * 本模块只负责「按键检测」这一层：
 *   矩阵驱动 -> 键值映射驱动 -> 本模块（日志输出）
 * 后续接 CAF / HID 时，在这里把事件转成 button_event 即可。
 */

#ifndef KEYS_H_
#define KEYS_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 矩阵行数 */
#define KB_ROWS 6
/** 矩阵列数 */
#define KB_COLS 4

/**
 * @brief 由 (行, 列) 计算逻辑键编号
 *
 * 取值范围 0 ~ (KB_ROWS * KB_COLS - 1)，空位不使用。
 */
#define KB_KEY_ID(row, col) (((row) * KB_COLS) + (col))

/**
 * @brief 按键模块初始化
 *
 * 事件回调本身在编译期通过 INPUT_CALLBACK_DEFINE 注册，
 * 这里只做设备就绪检查与信息打印。
 *
 * @retval 0 成功
 * @retval -ENODEV 矩阵或键值映射设备未就绪
 */
int keys_init(void);

/**
 * @brief 查询指定 (行, 列) 的按键名称
 *
 * @param row 行号（0 起）
 * @param col 列号（0 起）
 * @return 按键名称；未定义的位置返回 NULL
 */
const char *keys_key_name(uint8_t row, uint8_t col);

#ifdef __cplusplus
}
#endif

#endif /* KEYS_H_ */
