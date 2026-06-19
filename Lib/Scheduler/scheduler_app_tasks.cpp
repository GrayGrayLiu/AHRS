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
#include <cstring>
#include <stdio.h>

namespace
{

constexpr uint32_t IMU_DEBUG_PERIOD_MS = 1000u;
constexpr uint32_t IMU_DRDY_DEADLINE_MS = 5u;

// ============================================================================
// IMU debug print task — 低优先级 IMU 状态输出
// ============================================================================

/**
 * @brief  IMU delta 状态 print task（1 Hz，LOW priority）。
 * @note   只读取 Service 缓存，不访问 SPI / FIFO / EXIT，不修改 ICM42688P 驱动状态机。
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
// ICM42688P event+deadline task — IMU data-ready 主处理路径
// ============================================================================

/**
 * @brief  ICM42688P data-ready event+deadline task（HIGH priority）。
 * @note   不直接访问 SPI/FIFO，只调用 icm42688_service::Run()。
 */
void ImuDrdyTask(SchedulerRunReason reason, SchedulerEventMask events,
                 uint32_t now_ms, uint64_t now_us, void *context)
{
    (void)reason; (void)events; (void)now_ms; (void)now_us; (void)context;

    icm42688_service::Run();
}

// ============================================================================
// 统一应用任务注册表
// ============================================================================

// 所有挂载到 generic Scheduler 的应用任务集中在此表中定义。
// 新增任务只需添加一条 SchedulerTaskConfig 配置，不需要修改注册循环。
// 表内顺序即注册顺序；运行调度顺序由 Scheduler core 根据 priority、event/deadline/period 决定。
constexpr SchedulerTaskConfig kAppTasks[] = {
    {
        "imu_debug",
        ImuDebugTask,
        nullptr,
        SCHEDULER_PRIORITY_LOW,
        0u,
        IMU_DEBUG_PERIOD_MS,
        0u,
        0u,
        1u,
    },
    {
        "imu_drdy",
        ImuDrdyTask,
        nullptr,
        SCHEDULER_PRIORITY_HIGH,
        scheduler_app_events::IMU_DRDY,
        0u,
        0u,
        IMU_DRDY_DEADLINE_MS,
        1u,
    },
};

} // namespace

// ============================================================================
// 对外注册入口
// ============================================================================

/**
 * @brief  遍历 kAppTasks[] 表并向 generic Scheduler 注册所有应用任务。
 * @note   必须在 Scheduler_Init() 成功后调用。注册失败只打印错误，不进入 Error_Handler。
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
