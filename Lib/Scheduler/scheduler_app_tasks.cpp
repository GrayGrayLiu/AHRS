/**
 * @file    scheduler_app_tasks.cpp
 * @brief   Generic Scheduler 应用/调试任务实现
 *
 * @details
 * 本文件实现挂载到 generic Scheduler 的应用级任务 callback 和注册逻辑。
 * 不实现任何调度核心逻辑。
 */

#include "scheduler_app_tasks.h"
#include "scheduler.h"
#include "scheduler_app_events.h"
#include "ICM42688_Service.hpp"
#include "AidedInsService.hpp"
#include <cstring>
#include <stdio.h>

namespace
{

constexpr uint32_t IMU_DEBUG_PERIOD_MS = 1000u;
constexpr uint32_t INS_DEBUG_PERIOD_MS = 1000u;
constexpr uint32_t IMU_DRDY_DEADLINE_MS = 5u;

// ============================================================================
// IMU debug print task — IMU 状态输出
// ============================================================================

/**
 * @brief  IMU delta 状态 print task（1 Hz，priority=200）。
 * @note   只读取 Service 缓存，不访问 SPI / FIFO / EXTI，不修改 ICM42688P 驱动状态机。
 *         自行维护 sample_counter 去重，避免相同数据重复输出。
 */
void ImuDebugTask(SchedulerRunReason reason, SchedulerEventMask events,
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
// Aided INS stats debug task（1 Hz，用于验证 IMU→INS 数据链路）
// ============================================================================

/**
 * @brief  Aided INS Service 低频统计 print task（1 Hz，priority=210）。
 * @note   只读取 aided_ins_service::GetStats() 并 printf；
 *         不访问 SPI/FIFO/EXTI，不修改驱动或算法状态。
 */
void InsDebugTask(SchedulerRunReason reason, SchedulerEventMask events,
                  uint32_t now_ms, uint64_t now_us, void *context)
{
    (void)reason; (void)events; (void)now_ms; (void)now_us; (void)context;

    const auto stats = aided_ins_service::GetStats();
    printf("[INS_SVC] run=%lu valid=%lu agg=%lu ins=%lu fail=%lu inv=%lu dup=%lu ts_err=%lu que=%lu drop=%lu dis=%lu ins_us=%lu max=%lu srv_us=%lu max=%lu\r\n",
           static_cast<unsigned long>(stats.run_calls),
           static_cast<unsigned long>(stats.valid_samples),
           static_cast<unsigned long>(stats.aggregated_samples),
           static_cast<unsigned long>(stats.ins_run_calls),
           static_cast<unsigned long>(stats.get_latest_failures),
           static_cast<unsigned long>(stats.invalid_samples),
           static_cast<unsigned long>(stats.duplicate_samples),
           static_cast<unsigned long>(stats.timestamp_errors),
           static_cast<unsigned long>(stats.queued_imu_samples),
           static_cast<unsigned long>(stats.dropped_imu_samples),
           static_cast<unsigned long>(stats.ins_run_disabled),
           static_cast<unsigned long>(stats.ins_run_last_us),
           static_cast<unsigned long>(stats.ins_run_max_us),
           static_cast<unsigned long>(stats.service_run_last_us),
           static_cast<unsigned long>(stats.service_run_max_us));

    // [PROFILE] 临时分段耗时输出
    printf("[INS_PROF] pnd=%lu %lu prp=%lu %lu mec=%lu %lu fmx=%lu %lu ekf=%lu %lu afb=%lu %lu\r\n",
           static_cast<unsigned long>(stats.pnd_us),
           static_cast<unsigned long>(stats.pnd_max),
           static_cast<unsigned long>(stats.prp_us),
           static_cast<unsigned long>(stats.prp_max),
           static_cast<unsigned long>(stats.mec_us),
           static_cast<unsigned long>(stats.mec_max),
           static_cast<unsigned long>(stats.fmx_us),
           static_cast<unsigned long>(stats.fmx_max),
           static_cast<unsigned long>(stats.ekf_us),
           static_cast<unsigned long>(stats.ekf_max),
           static_cast<unsigned long>(stats.afb_us),
           static_cast<unsigned long>(stats.afb_max));

    // [PROFILE] Phase-2: fmx 子段耗时输出
    printf("[INS_PROF2] alc=%lu %lu fill=%lu %lu q1=%lu %lu q2=%lu %lu\r\n",
           static_cast<unsigned long>(stats.alc_us),
           static_cast<unsigned long>(stats.alc_max),
           static_cast<unsigned long>(stats.fill_us),
           static_cast<unsigned long>(stats.fill_max),
           static_cast<unsigned long>(stats.q1_us),
           static_cast<unsigned long>(stats.q1_max),
           static_cast<unsigned long>(stats.q2_us),
           static_cast<unsigned long>(stats.q2_max));
}

// ============================================================================
// ICM42688P event+deadline task — IMU data-ready 主处理路径
// ============================================================================

/**
 * @brief  ICM42688P data-ready event+deadline task（priority=10）。
 * @note   不直接访问 SPI/FIFO，只调用 icm42688_service::Run()。
 */
void ImuDrdyTask(SchedulerRunReason reason, SchedulerEventMask events,
                 uint32_t now_ms, uint64_t now_us, void *context)
{
    (void)reason; (void)events; (void)now_ms; (void)now_us; (void)context;

    icm42688_service::Run();
    (void)aided_ins_service::Run();
}

// ============================================================================
// 统一应用任务注册表
// ============================================================================

// 所有挂载到 generic Scheduler 的应用任务集中在此表中定义。
// 新增任务只需添加一条 SchedulerTaskConfig 配置，不需要修改注册循环。
// 表内顺序即注册顺序；运行调度顺序由 Scheduler core 根据 priority、event/deadline/period 决定。
constexpr SchedulerTaskConfig kAppTasks[] = {
    // priority: 0=highest, 255=lowest
    {
        "imu_debug",                        // IMU 状态 print（1 Hz），暂时关闭以验证 INS Service
        ImuDebugTask,
        nullptr,
        200u,
        0u,
        IMU_DEBUG_PERIOD_MS,
        0u,
        0u,
        0u,                                 // 暂时关闭：验证 Aided INS Service 统计输出
    },
    {
        "imu_drdy",                         // ICM42688P data-ready 主处理路径
        ImuDrdyTask,
        nullptr,
        10u,                                // priority: 高优先级，接近 data-ready 响应
        scheduler_app_events::IMU_DRDY,
        0u,
        0u,
        IMU_DRDY_DEADLINE_MS,
        1u,
    },
    {
        "ins_debug",                        // Aided INS Service 统计 print（1 Hz）
        InsDebugTask,
        nullptr,
        210u,                               // priority: 低于 IMU driver debug
        0u,
        INS_DEBUG_PERIOD_MS,
        0u,
        0u,
        1u,
    },
};

} // namespace

// ============================================================================
// 对外入口
// ============================================================================

/**
 * @brief  应用/业务模块初始化入口。
 * @note   当前用于初始化 Aided INS Service；
 *         本函数不注册 scheduler task；任务注册由 SchedulerAppTasks_RegisterAll() 负责。
 */
extern "C" void App_Init(void)
{
    (void)aided_ins_service::Init();
}

// ============================================================================

/**
 * @brief  遍历 kAppTasks[] 表并向 generic Scheduler 注册所有应用任务。
 * @note   必须在 Scheduler_Init() 和 App_Init() 之后调用。注册失败只打印错误，不进入 Error_Handler。
 */
extern "C" void SchedulerAppTasks_RegisterAll(void)
{
    constexpr size_t kAppTaskCount = sizeof(kAppTasks) / sizeof(kAppTasks[0]);

    for (size_t i = 0u; i < kAppTaskCount; ++i) {
        const SchedulerTaskId id = Scheduler_RegisterTask(&kAppTasks[i]);

        if (id == SCHEDULER_TASK_ID_INVALID) {
            printf("[sched] %s register failed\r\n",
                   kAppTasks[i].name != nullptr ? kAppTasks[i].name : "<unnamed>");
        } else if (kAppTasks[i].name != nullptr && std::strcmp(kAppTasks[i].name, "imu_drdy") == 0) {
            // 仅在注册成功时将 imu_drdy task id 注入 Service，使其能在 Run() 中调用 Scheduler_Schedule*
            icm42688_service::SetSchedulerTaskId(id);
        }
    }
}
