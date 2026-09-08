/*
 * 低功耗（两级空闲降载 / 断电停机）对外接口
 *
 * 本模块只做「策略编排」，不直接碰底层硬件：
 *   - 订阅活动源（按键/旋钮/切档），维护 1 分钟 / 5 分钟计时；
 *   - 到点后经各模块既有/新增接口执行降载或断电（见 低功耗.txt）。
 *
 * 状态机：ACTIVE --(1 分钟无活动)--> IDLE --(累计 5 分钟无活动)--> POWER OFF
 *         任意活动（按键/旋钮/切档）立即回到 ACTIVE 并重新计时。
 * 充电期间不进任何档位，充电结束后从零重新计时。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 低功耗状态 */
enum kb_pm_state {
	KB_PM_ACTIVE = 0,  /**< 满速工作（默认） */
	KB_PM_IDLE,        /**< 档1：空闲降载（停 LCD 刷新、降背光、放慢轮询） */
	KB_PM_OFF,         /**< 档2：断电停机（System Off + IP5306 待机切断 3V3） */
};

/**
 * @brief 初始化低功耗状态机
 *
 * 启动专用工作队列、挂 1min/5min 计时器与充电监视，订阅活动源。
 * 必须在 keys/knob/mode/lcd/ip5306 等初始化【之后】调用（建议放 main 末尾）。
 *
 * @retval 0 成功
 */
int kb_pm_init(void);

/** @brief 查询当前低功耗状态（非阻塞） */
enum kb_pm_state kb_pm_state_get(void);

/**
 * @brief 上报一次用户活动（按键 / 旋钮 / 切换模式）
 *
 * 任意线程可调用（input 线程、系统 wq、模式 wq）。内部只做
 * 原子更新时间戳 + 重排计时器；若正在档1 则投递“退出省电”到 pm wq。
 */
void kb_pm_activity(void);

/** @brief 立即进入档1（省电降载），用于 shell / 电流测量 */
int kb_pm_idle_now(void);

/** @brief 立即执行档2 断电流程，用于 shell / 电流测量（仍做安全检查） */
int kb_pm_off_now(void);

/**
 * @brief 开关自动省电
 *
 * @param enable false：保持满速，取消计时与挂起（档1 立即退出）；
 *               true：重新开始计时。
 */
int kb_pm_auto_set(bool enable);

/** @brief 查询自动省电是否开启 */
bool kb_pm_auto_enabled(void);

#ifdef __cplusplus
}
#endif
