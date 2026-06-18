/**
 * @file    scheduler_app_tasks.cpp
 * @brief   Generic Scheduler 应用/调试任务实现
 *
 * @details
 * 本文件实现所有挂载到 generic Scheduler 的应用级任务 callback 和注册逻辑。
 * 当前包含 smoke heartbeat task、stats reader task 和 IMU debug print task；
 * 后续可在此新增 LED 管理等低优先级任务。
 *
 * 不依赖 SPI / FIFO / EXIT / TimeBase。不实现任何调度核心逻辑。
 */

#include "scheduler_app_tasks.h"
#include "scheduler.h"
#include "ICM42688_Service.hpp"
#include <stdio.h>

namespace
{

// ============================================================================
// Task ID（文件作用域，stats task 通过 s_smoke_id 读取 smoke 的统计）
// ============================================================================

constexpr uint32_t IMU_DRDY_DEADLINE_MS = 5u;

static SchedulerTaskId s_smoke_id = SCHEDULER_TASK_ID_INVALID;
static SchedulerTaskId s_stats_id = SCHEDULER_TASK_ID_INVALID;
static SchedulerTaskId s_imu_debug_id = SCHEDULER_TASK_ID_INVALID;
static SchedulerTaskId s_imu_drdy_id = SCHEDULER_TASK_ID_INVALID;
static SchedulerTaskId s_imu_drdy_diag_id = SCHEDULER_TASK_ID_INVALID;

// ── 6B-6C 过渡诊断计数器（阶段 6D 删除） ──
static volatile uint32_t s_imu_drdy_total_count    = 0u;
static volatile uint32_t s_imu_drdy_event_count    = 0u;
static volatile uint32_t s_imu_drdy_deadline_count = 0u;

// ============================================================================
// Smoke heartbeat task — 验证 generic Scheduler 周期调度链路
// ============================================================================

/**
 * @brief  阶段 4C/5B 临时 smoke heartbeat task。
 * @note   6C 阶段已禁用 printf：阻塞式 UART 输出会延迟 generic RunOnce 返回，
 *         导致 ImuDrdyTask 不能及时消费 FIFO。计数器仍正常更新，可通过调试器读取。
 */
static void SmokeTask(SchedulerRunReason reason, SchedulerEventMask events,
                      uint32_t now_ms, uint64_t now_us, void *context)
{
    (void)reason; (void)events; (void)now_ms; (void)now_us; (void)context;

    static uint32_t count = 0u;
    ++count;
    // printf disabled in 6C to avoid blocking main loop during FIFO consumption.
}

// ============================================================================
// Stats reader task — 验证 Scheduler_GetTaskStats 和多 task 并行
// ============================================================================

/**
 * @brief  阶段 4C/5B 临时 stats reader task。
 * @note   6C 阶段已禁用 printf：阻塞式 UART 输出会导致 FIFO batch 抖动。
 *         Scheduler_GetTaskStats 调用保留，可通过调试器观察统计值。
 */
static void StatsTask(SchedulerRunReason reason, SchedulerEventMask events,
                      uint32_t now_ms, uint64_t now_us, void *context)
{
    (void)reason; (void)events; (void)now_ms; (void)now_us; (void)context;

    if (s_smoke_id == SCHEDULER_TASK_ID_INVALID) {
        return;
    }

    SchedulerTaskStats stats;
    (void)Scheduler_GetTaskStats(s_smoke_id, &stats);
    // printf disabled in 6C to avoid blocking main loop during FIFO consumption.
}

// ============================================================================
// IMU debug print task — 从 legacy Loop_1Hz 迁移到 generic Scheduler
// ============================================================================

/**
 * @brief  IMU delta 调试 print task（1 Hz 低优先级）。
 * @note   等价于旧 Print_ICM42688_Delta_Debug()。只读取 Service 缓存，不访问
 *         SPI / FIFO / EXIT，不修改 ICM42688P 驱动状态机。自行维护 sample_counter 去重。
 */
static void ImuDebugTask(SchedulerRunReason reason, SchedulerEventMask events,
                         uint32_t now_ms, uint64_t now_us, void *context)
{
    (void)reason; (void)events; (void)now_ms; (void)now_us; (void)context;

    icm42688_service::DeltaSample imu{};
    const ICM42688P::Status status = icm42688_service::GetDeltaLatest(&imu);

    if (status != ICM42688P::Status::Ok) {
        printf("[imu] st=%d\r\n", static_cast<int>(status));
        return;
    }

    static uint32_t s_imu_last_sample_counter = 0u;

    if (imu.sample_counter == s_imu_last_sample_counter) {
        return;
    }
    s_imu_last_sample_counter = imu.sample_counter;

    if (imu.delta_time_s <= 0.0f) {
        printf("[imu] bad dt\r\n");
        return;
    }

    float gyro_rad_s[3];
    float force_m_s2[3];

    for (uint8_t i = 0; i < 3; i++) {
        gyro_rad_s[i] = imu.delta_angle_rad[i] / imu.delta_time_s;
        force_m_s2[i] = imu.delta_velocity_m_s[i] / imu.delta_time_s;
    }

    printf("t=%llu c=%lu n=%u dt=%.6f "
           "w=%.1f %.1f %.1f "
           "f=%.2f %.2f %.2f "
           "T=%.2f\r\n",
           (unsigned long long)imu.timestamp_us,
           (unsigned long)imu.sample_counter,
           (unsigned int)imu.delta_samples,
           imu.delta_time_s,
           gyro_rad_s[0] * 57.2957795f,
           gyro_rad_s[1] * 57.2957795f,
           gyro_rad_s[2] * 57.2957795f,
           force_m_s2[0],
           force_m_s2[1],
           force_m_s2[2],
           imu.temperature_deg_c);
}

// ============================================================================
// ICM42688P event+deadline task（阶段 6B 接入，6C 后成为主数据路径）
// ============================================================================

/**
 * @brief  ICM42688P data-ready event+deadline task（HIGH priority）。
 * @note   6B 阶段验证 generic event path 全链路（ISR→PostEvent→RunOnce→callback）；
 *         6C 停用 Loop_1000Hz 后本 task 成为 IMU 主数据路径。
 *         不直接访问 SPI/FIFO，只调用 icm42688_service::Run()。
 */
static void ImuDrdyTask(SchedulerRunReason reason, SchedulerEventMask events,
                        uint32_t now_ms, uint64_t now_us, void *context)
{
    (void)events; (void)now_ms; (void)now_us; (void)context;

    ++s_imu_drdy_total_count;

    if ((reason & SCHEDULER_REASON_EVENT) != 0u)    { ++s_imu_drdy_event_count; }
    if ((reason & SCHEDULER_REASON_DEADLINE) != 0u) { ++s_imu_drdy_deadline_count; }

    icm42688_service::Run();
}

// ============================================================================
// 6B 过渡诊断 print task（30 s，阶段 6D 删除）
// ============================================================================

/**
 * @brief  6B/6C 过渡诊断 task（30 s，阶段 6D 删除）。
 * @note   6C 阶段已禁用 printf：阻塞式 UART 输出会延迟 RunOnce 返回，导致下一轮
 *         ImuDrdyTask 无法及时消费 FIFO，引发 batch 抖动和 n>20 异常。
 *         计数器 s_imu_drdy_*_count 仍由 ImuDrdyTask 正常更新，可通过调试器读取。
 */
static void ImuDrdyDiagTask(SchedulerRunReason reason, SchedulerEventMask events,
                            uint32_t now_ms, uint64_t now_us, void *context)
{
    (void)reason; (void)events; (void)now_ms; (void)now_us; (void)context;
    // printf disabled in 6C to avoid blocking the main loop during FIFO consumption.
    // Counters are still updated by ImuDrdyTask and readable via debugger.
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

    s_imu_debug_id = Scheduler_RegisterPeriodicTask(
        "imu_debug", 1000u, ImuDebugTask, NULL, SCHEDULER_PRIORITY_LOW);

    if (s_imu_debug_id == SCHEDULER_TASK_ID_INVALID) {
        printf("[sched] imu_debug register failed\r\n");
    }

    s_imu_drdy_id = Scheduler_RegisterEventDeadlineTask(
        "imu_drdy", SCHED_HP_EVENT_IMU_DRDY, IMU_DRDY_DEADLINE_MS,
        ImuDrdyTask, NULL, SCHEDULER_PRIORITY_HIGH);

    if (s_imu_drdy_id == SCHEDULER_TASK_ID_INVALID) {
        printf("[sched] imu_drdy register failed\r\n");
    }

    s_imu_drdy_diag_id = Scheduler_RegisterPeriodicTask(
        "imu_drdy_diag", 30000u, ImuDrdyDiagTask, NULL, SCHEDULER_PRIORITY_LOW);

    if (s_imu_drdy_diag_id == SCHEDULER_TASK_ID_INVALID) {
        printf("[sched] imu_drdy_diag register failed\r\n");
    }
}
