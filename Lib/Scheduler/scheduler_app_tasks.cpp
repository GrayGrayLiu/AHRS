/**
 * @file    scheduler_app_tasks.cpp
 * @brief   Generic Scheduler 应用/调试任务实现
 *
 * @details
 * 本文件实现所有挂载到 generic Scheduler 的应用级任务 callback 和注册逻辑。
 * 当前包含阶段 4C 的 smoke heartbeat task 和 stats reader task；
 * 后续可在此新增 IMU debug print、LED 管理等低优先级任务。
 *
 * 不依赖 ICM42688P / SPI / FIFO / EXIT / TimeBase。
 * 不实现任何调度核心逻辑。
 */

#include "scheduler_app_tasks.h"
#include "scheduler.h"
#include <stdio.h>

namespace
{

// ============================================================================
// Task ID（文件作用域，stats task 通过 s_smoke_id 读取 smoke 的统计）
// ============================================================================

static SchedulerTaskId s_smoke_id = SCHEDULER_TASK_ID_INVALID;
static SchedulerTaskId s_stats_id = SCHEDULER_TASK_ID_INVALID;

// ============================================================================
// Smoke heartbeat task — 验证 generic Scheduler 周期调度链路
// ============================================================================

/**
 * @brief  阶段 4C/5B 临时 smoke heartbeat task。
 * @note   只做 heartbeat printf，验证 generic Scheduler 周期调度链路存活。
 *         不访问 ICM42688P / SPI / FIFO / 飞控数据链路。
 *         后续可用正式低优先级任务替换或删除。
 */
static void SmokeTask(SchedulerRunReason reason, SchedulerEventMask events,
                      uint32_t now_ms, uint64_t now_us, void *context)
{
    (void)reason; (void)events; (void)now_ms; (void)now_us; (void)context;

    static uint32_t count = 0u;
    ++count;

    printf("[sched] heartbeat count=%lu\r\n", (unsigned long)count);
}

// ============================================================================
// Stats reader task — 验证 Scheduler_GetTaskStats 和多 task 并行
// ============================================================================

/**
 * @brief  阶段 4C/5B 临时 stats reader task。
 * @note   验证 Scheduler_GetTaskStats() 和多个 generic periodic task 并行调度。
 *         只读取 smoke task 的调度统计并 printf，不访问 ICM42688P / SPI / FIFO。
 *         后续可用正式任务替换或删除。
 */
static void StatsTask(SchedulerRunReason reason, SchedulerEventMask events,
                      uint32_t now_ms, uint64_t now_us, void *context)
{
    (void)reason; (void)events; (void)now_ms; (void)now_us; (void)context;

    if (s_smoke_id == SCHEDULER_TASK_ID_INVALID) {
        return;
    }

    SchedulerTaskStats stats;
    if (Scheduler_GetTaskStats(s_smoke_id, &stats) != 0u) {
        printf("[sched] stats smoke_run=%lu interval=%lu last_us=%lu max_us=%lu\r\n",
               (unsigned long)stats.run_count,
               (unsigned long)stats.interval_run_count,
               (unsigned long)stats.last_runtime_us,
               (unsigned long)stats.max_runtime_us);
    }
}

} // namespace

// ============================================================================
// 对外注册入口
// ============================================================================

/**
 * @brief  注册所有应用/调试任务到 generic Scheduler。
 * @note   必须在 Scheduler_Init() 成功后调用。
 *         注册失败只打印错误，不进入 Error_Handler。
 */
extern "C" void SchedulerAppTasks_RegisterAll(void)
{
    s_smoke_id = Scheduler_RegisterPeriodicTask(
        "scheduler_smoke", 2000u, SmokeTask, NULL, SCHEDULER_PRIORITY_LOW);

    if (s_smoke_id == SCHEDULER_TASK_ID_INVALID) {
        printf("[sched] smoke register failed\r\n");
        return;
    }

    s_stats_id = Scheduler_RegisterPeriodicTask(
        "scheduler_stats", 5000u, StatsTask, NULL, SCHEDULER_PRIORITY_LOW);

    if (s_stats_id == SCHEDULER_TASK_ID_INVALID) {
        printf("[sched] stats register failed\r\n");
    }
}
