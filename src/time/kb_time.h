/*
 * 软件时钟模块对外接口（LCD 第二行时间 + 上位机同步预留）
 *
 * 本模块只做「绝对时间换算」这一层：
 *   记录同步基点(epoch 秒) + 系统 uptime 运行时长 -> 推算当前年月日时分秒。
 * 上电默认 2026-01-01 00:00:00，掉电不保持；供后续上位机/串口/蓝牙同步。
 *
 * 详见 lcd 显示.txt【6】
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 拆分后的本地时间（年 = 公历 4 位，其余从 1/0 起） */
struct kb_time {
	uint16_t year;   /* e.g. 2026 */
	uint8_t month;   /* 1..12 */
	uint8_t day;     /* 1..31 */
	uint8_t hour;    /* 0..23 */
	uint8_t minute;  /* 0..59 */
	uint8_t second;  /* 0..59 */
};

/**
 * @brief 初始化软件时钟
 *
 * 把基点设为默认时间（2026-01-01 00:00:00）。之后即可用 kb_time_get()。
 *
 * @retval 0 成功
 */
int kb_time_init(void);

/**
 * @brief 用年月日时分秒设置当前时间（上位机/调试同步入口）
 *
 * 以「调用瞬间」为新的同步基点，之后秒级走 uptime。
 *
 * @param t 要设置的时间（NULL 返回 -EINVAL；字段超范围返回 -EINVAL）
 * @retval 0 成功
 */
int kb_time_set(const struct kb_time *t);

/**
 * @brief 用 unix epoch（秒，UTC）设置当前时间
 *
 * @param sec epoch 秒数
 * @retval 0 成功
 */
int kb_time_set_epoch(uint32_t sec);

/**
 * @brief 读取当前本地时间
 *
 * @param t 输出（NULL 返回 -EINVAL）
 * @retval 0 成功
 */
int kb_time_get(struct kb_time *t);

#ifdef __cplusplus
}
#endif
