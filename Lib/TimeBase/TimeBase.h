/**
 * @file    TimeBase.h
 * @brief   全系统时间基准接口
 *
 * @details
 * 时间基准基于 TIM2 32 位硬件定时器 + 软件 32 位溢出计数，组合为 64 位
 * 微秒计数器。毫秒接口由微秒值除以 1000 派生。
 *
 * 时间源：
 *   TIM2 由 CubeMX 配置为 1 MHz 向上计数（CNT 每 1 us 递增），
 *   ARR = 0xFFFFFFFF（满 32 位后溢出产生更新中断）。
 *   TimeBase_OnTimPeriodElapsed() 在 TIM2 更新中断 ISR 中递增软件溢出字。
 *
 * 关键不变量：
 *   - TimeBase_Micros() 可在 ISR 和普通上下文中安全调用（临界区保护）。
 *   - 64 位计数器在 1 MHz 下溢出周期约 58 万年，工程上可视为不回绕。
 *   - 32 位毫秒值溢出周期约 49.7 天；调用者（scheduler/ICM42688）使用
 *     有符号差值 (int32_t)(now - target) >= 0 处理溢出。
 *
 * 与系统的关系：
 *   - Scheduler_Run() 用 TimeBase_Millis() 驱动周期任务调度。
 *   - EXTI ISR 用 TimeBase_Micros() 记录 data-ready 时间戳。
 *   - ICM42688P 用 ComputeBatchDeltaTimeSeconds() 基于相邻 data-ready
 *     MCU 时间戳计算实测 batch dt。
 */

#pragma once

#include <stdint.h>
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  启动 TIM2 硬件定时器和更新中断
 * @note   必须在 scheduler 启动前调用；仅调用一次。
 */
void TimeBase_Start(void);

/**
 * @brief  读取当前 64 位微秒时间戳
 * @retval 系统启动以来的微秒数
 * @note   可在 ISR 和普通上下文中安全调用（内部关中断保护原子读取）。
 */
uint64_t TimeBase_Micros(void);

/**
 * @brief  读取当前 32 位毫秒时间戳
 * @retval 系统启动以来的毫秒数
 * @note   由 TimeBase_Micros() / 1000 派生。
 *         调用者须用有符号差值处理 32 位回绕。
 */
uint32_t TimeBase_Millis(void);

/**
 * @brief  TIM2 更新中断回调——递增软件溢出高字
 * @param  htim 触发回调的 TIM 句柄
 * @note   由 HAL_TIM_IRQHandler 在 ISR 中调用；
 *         仅处理 TIM2 实例，忽略其他定时器。
 */
void TimeBase_OnTimPeriodElapsed(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif
