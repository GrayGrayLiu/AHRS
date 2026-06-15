//
// Created by Gray on 2026/1/7.
//

#ifndef ELECTROMAGNETICARTILLERY_SCHEDULER_H
#define ELECTROMAGNETICARTILLERY_SCHEDULER_H

#include "stm32h7xx.h"

// 调度器采用 1 ms 作为基础 tick，因此每秒包含 1000 个调度 tick。
#define TICK_PER_SECOND	1000

// IMU data-ready 高优先级事件位。ISR adapter 通过该事件通知主循环尽快运行
// IMU Service，ISR 本身不执行 SPI 读取或驱动处理逻辑。
#define SCHED_HP_EVENT_IMU_DRDY (1u << 0)

// 周期轮询任务表项：描述任务入口、期望频率以及运行时调度状态。
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

// 初始化调度器自身，包括根据任务频率计算各周期任务的 interval_ticks。
void Scheduler_Setup(void);

// cooperative 调度入口，应在 main 的 while (1) 中反复调用；负责分发
// 高优先级事件并检查、执行到期的周期轮询任务。
void Scheduler_Run(void);

// 由 ISR 或 ISR adapter 调用，仅向高优先级事件位图置位，不直接执行 handler。
void Scheduler_PostHighPriorityEventFromISR(uint32_t event);

#ifdef __cplusplus
}
#endif

#endif //ELECTROMAGNETICARTILLERY_SCHEDULER_H
