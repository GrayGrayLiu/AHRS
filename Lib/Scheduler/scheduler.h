/**
 * @file    scheduler.h
 * @brief   裸机 cooperative 调度器接口
 *
 * @details
 * 调度器分为两层：
 *
 * 1. 高优先级事件（响应式）
 *    ISR 通过 Scheduler_PostHighPriorityEventFromISR() 原子置位事件位图；
 *    主循环中 Scheduler_HighPriorityPoll() 轮询位图并分发到对应 handler。
 *    当前只有 SCHED_HP_EVENT_IMU_DRDY 一个事件，对应 icm42688_service::Run。
 *
 * 2. 周期轮询任务
 *    静态任务表 sched_tasks[] 定义了一组从 1000 Hz 到 0.1 Hz 的回调；
 *    Scheduler_Run() 每 tick 检查所有任务是否到期并执行。
 *    1 kHz 任务固定调用 icm42688_service::Run() 作为 IMU 周期兜底路径。
 *
 * ISR ↔ 普通上下文边界：
 *    - PostHighPriorityEventFromISR 可在 ISR 调用，仅操作 volatile 位图。
 *    - 事件 handler（Service::Run）在普通上下文中由 HighPriorityPoll 调用。
 *    - 周期任务回调均在普通上下文中执行。
 */

#ifndef ELECTROMAGNETICARTILLERY_SCHEDULER_H
#define ELECTROMAGNETICARTILLERY_SCHEDULER_H

#include "stm32h7xx.h"

// 调度器基础 tick 为 1 ms，每秒 1000 tick。
#define TICK_PER_SECOND 1000

// ── 高优先级事件位定义 ──
// IMU data-ready 事件位。ISR adapter 通过该事件通知主循环尽快运行
// IMU Service，ISR 本身不执行 SPI 读取或驱动处理逻辑。
#define SCHED_HP_EVENT_IMU_DRDY (1u << 0)

// ── 周期轮询任务表项 ──
typedef struct
{
    // 周期任务函数指针，由 Scheduler_Run() 在任务到期时调用。
    void(*task_func)(void);
    // 任务期望执行频率，单位 Hz。
    float rate_hz;
    // 执行间隔，单位为 1 ms tick；由 Scheduler_Setup() 根据 rate_hz 计算。
    uint16_t interval_ticks;
    // 任务上一次执行的时间戳，单位 ms。
    uint32_t last_run;
}sched_task_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化调度器，根据各周期任务频率计算 interval_ticks
 * @note   在 main 中调用一次，先于 Scheduler_Run。
 */
void Scheduler_Setup(void);

/**
 * @brief  cooperative 调度入口，在 main while(1) 中反复调用
 *
 * @note   负责分发高优先级事件并遍历周期任务表。
 *         在每次任务检查前后插入 Scheduler_HighPriorityPoll()，
 *         使 ISR 投递的事件尽快在普通上下文得到处理。
 */
void Scheduler_Run(void);

/**
 * @brief  ISR 级事件投递入口：原子置位高优先级事件，不直接执行 handler
 * @param  event 高优先级事件位（如 SCHED_HP_EVENT_IMU_DRDY）
 *
 * @note   仅操作位图，不在 ISR 上下文执行 SPI、FIFO 或 printf。
 *         可在 ISR 中安全调用。
 */
void Scheduler_PostHighPriorityEventFromISR(uint32_t event);

#ifdef __cplusplus
}
#endif

#endif //ELECTROMAGNETICARTILLERY_SCHEDULER_H
