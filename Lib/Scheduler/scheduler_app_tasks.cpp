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
#include "Aided_INS_DebugConfig.hpp"
#include <stdio.h>

namespace
{

constexpr uint32_t IMU_DEBUG_PERIOD_MS = 1000u;
constexpr uint32_t INS_DEBUG_PERIOD_MS = 5000u;
constexpr uint32_t INS_CONSUMER_PERIOD_MS = 1u;
constexpr uint8_t INS_DEBUG_TASK_ENABLED =
    (AIDED_INS_ENABLE_SERVICE_PRINT ||
     AIDED_INS_ENABLE_PROFILING_PRINT ||
     AIDED_INS_ENABLE_DEBUG_PRINT) ? 1u : 0u;
constexpr uint32_t IMU_DRDY_DEADLINE_MS = 5u;
constexpr uint32_t ATTITUDE_TELEMETRY_PERIOD_MS = 40u;  // 25 Hz
constexpr uint8_t ATTITUDE_TELEMETRY_TASK_ENABLED =
    AIDED_INS_ENABLE_ATTITUDE_TELEMETRY ? 1u : 0u;

// ============================================================================
// IMU debug print task — IMU 状态输出
// ============================================================================

/**
 * @brief  IMU delta 状态 print task（1 Hz，priority=200）。
 * @note   只读取 Service 缓存，不访问 SPI / FIFO / EXTI，不修改 ICM42688P 驱动状态机。
 *         自行维护 sample_counter 去重，避免相同数据重复输出。
 */
void ImuDebugTask(SchedulerTaskId self_id, SchedulerRunReason reason, SchedulerEventMask events,
                  uint32_t now_ms, uint64_t now_us, void *context)
{
    (void)self_id; (void)reason; (void)events; (void)now_ms; (void)now_us; (void)context;

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
// Aided INS stats debug task — 运行期统计输出（默认关闭）
// ============================================================================

/**
 * @brief  Aided INS Service 低频统计 print task（默认关闭，启用时周期由 INS_DEBUG_PERIOD_MS 决定）。
 * @note   只读取 aided_ins_service::GetStats() 并 printf；
 *         不访问 SPI/FIFO/EXTI，不修改驱动或算法状态。
 */
void InsDebugTask(SchedulerTaskId self_id, SchedulerRunReason reason, SchedulerEventMask events,
                  uint32_t now_ms, uint64_t now_us, void *context)
{
    (void)self_id; (void)reason; (void)events; (void)now_ms; (void)now_us; (void)context;

    const auto stats = aided_ins_service::GetStats();
#if AIDED_INS_ENABLE_SERVICE_PRINT
    printf("[INS_SVC] run=%lu valid=%lu agg=%lu ins=%lu fail=%lu inv=%lu dup=%lu ts_err=%lu que=%lu drop=%lu ins_us=%lu max=%lu srv_us=%lu max=%lu\r\n",
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
           static_cast<unsigned long>(stats.ins_run_last_us),
           static_cast<unsigned long>(stats.ins_run_max_us),
           static_cast<unsigned long>(stats.service_run_last_us),
           static_cast<unsigned long>(stats.service_run_max_us));
#endif // AIDED_INS_ENABLE_SERVICE_PRINT

#if AIDED_INS_ENABLE_PROFILING_PRINT
    // [PROFILE] 分段耗时输出
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

    // [PROFILE] afb 拆分
    printf("[INS_PROF3] acc_prep=%lu %lu acc_ekf=%lu %lu fb=%lu %lu\r\n",
           static_cast<unsigned long>(stats.acc_prep_us),
           static_cast<unsigned long>(stats.acc_prep_max),
           static_cast<unsigned long>(stats.acc_ekf_us),
           static_cast<unsigned long>(stats.acc_ekf_max),
           static_cast<unsigned long>(stats.feedback_us),
           static_cast<unsigned long>(stats.feedback_max));

    // [PROFILE] EkfUpdateAcc3 内部分段
    printf("[INS_PROF4] phs=%lu %lu kdx=%lu %lu khp=%lu %lu\r\n",
           static_cast<unsigned long>(stats.acc_phs_us),
           static_cast<unsigned long>(stats.acc_phs_max),
           static_cast<unsigned long>(stats.acc_kdx_us),
           static_cast<unsigned long>(stats.acc_kdx_max),
           static_cast<unsigned long>(stats.acc_p_khp_us),
           static_cast<unsigned long>(stats.acc_p_khp_max));
#endif // AIDED_INS_ENABLE_PROFILING_PRINT

#if AIDED_INS_ENABLE_DEBUG_PRINT
    // [ACC_DBG] AccUpdate 触发诊断
    printf("[ACC_DBG] try=%lu ok=%lu small=%lu norm=%lu cos=%lu fb=%lu f_norm=%.3f g=%.3f diff=%.3f cos_last=%.4f f=(%.2f,%.2f,%.2f) g_b=(%.2f,%.2f,%.2f)\r\n",
           static_cast<unsigned long>(stats.acc_try),
           static_cast<unsigned long>(stats.acc_accept),
           static_cast<unsigned long>(stats.acc_fail_small),
           static_cast<unsigned long>(stats.acc_fail_norm),
           static_cast<unsigned long>(stats.acc_fail_cos),
           static_cast<unsigned long>(stats.acc_feedback),
           static_cast<double>(stats.f_norm),
           static_cast<double>(stats.f_gravity),
           static_cast<double>(stats.f_norm_diff),
           static_cast<double>(stats.f_cos_gn_gb),
           static_cast<double>(stats.f_f_b[0]), static_cast<double>(stats.f_f_b[1]), static_cast<double>(stats.f_f_b[2]),
           static_cast<double>(stats.f_g_b_ByImu[0]), static_cast<double>(stats.f_g_b_ByImu[1]), static_cast<double>(stats.f_g_b_ByImu[2]));

    // [EKF_DBG] EKFPredict 分段计时
    printf("[EKF_DBG] dx=%lu phip=%lu mphitq=%lu\r\n",
           static_cast<unsigned long>(stats.ekf_dx_us),
           static_cast<unsigned long>(stats.ekf_phi_p_us),
           static_cast<unsigned long>(stats.ekf_m_phi_t_q_us));

    // [ATT_DBG] 姿态/重力/比力一致性诊断
    printf("[ATT_DBG] eul=(%.2f,%.2f,%.2f) g_n=(%.3f,%.3f,%.3f) g_b=(%.3f,%.3f,%.3f) Cbn*f=(%.3f,%.3f,%.3f) Cbn*f+g=(%.3f,%.3f,%.3f) |Cf+g|=%.4f cos_f_gb=%.4f\r\n",
           static_cast<double>(stats.euler_r), static_cast<double>(stats.euler_p), static_cast<double>(stats.euler_y),
           static_cast<double>(stats.g_l_n[0]), static_cast<double>(stats.g_l_n[1]), static_cast<double>(stats.g_l_n[2]),
           static_cast<double>(stats.g_b[0]), static_cast<double>(stats.g_b[1]), static_cast<double>(stats.g_b[2]),
           static_cast<double>(stats.cbn_f[0]), static_cast<double>(stats.cbn_f[1]), static_cast<double>(stats.cbn_f[2]),
           static_cast<double>(stats.cbn_f_g[0]), static_cast<double>(stats.cbn_f_g[1]), static_cast<double>(stats.cbn_f_g[2]),
           static_cast<double>(stats.cfn_g_norm), static_cast<double>(stats.cos_fg));
#endif // AIDED_INS_ENABLE_DEBUG_PRINT
}

// ============================================================================
// INS consumer task — 定期消费聚合后的 200 Hz IMU
// ============================================================================

/**
 * @brief  INS consumer task（1 ms polling，priority=20）。
 * @note   不访问 ICM42688P；只从 AidedInsService 取最新聚合 IMU 并调用 INS Run。
 *
 *         设计目的：避免在 IMU DRDY producer 路径中同步执行 Aided_INS::Run()。
 *         观测条件：STM32H723 (cortex-m7)，FPU fpv5-d16 硬浮点，
 *         RelWithDebInfo 工程配置，runtime printf 及 profiling/debug/telemetry
 *         输出默认关闭。
 *
 *         在上述条件下，ins_consumer 内 Aided_INS::Run() 约 0.93 ms，
 *         明显长于 IMU DRDY producer 路径的数微秒级耗时；因此 producer 只负责
 *         400 Hz IMU sample 读取、校验、聚合和缓存，INS 运算交由独立 1 ms
 *         consumer task 异步消费 200 Hz IMU。
 *
 *         上述耗时为历史测量参考，不作为跨编译配置、跨平台或最坏情况实时上界。
 */
void InsConsumerTask(SchedulerTaskId self_id, SchedulerRunReason reason, SchedulerEventMask events,
                     uint32_t now_ms, uint64_t now_us, void *context)
{
    (void)self_id; (void)reason; (void)events; (void)now_ms; (void)now_us; (void)context;
    (void)aided_ins_service::RunInsConsumerOnce();
}
// ============================================================================
// ICM42688P event+deadline task — IMU data-ready 主处理路径
// ============================================================================

/**
 * @brief  ICM42688P data-ready event+deadline task（priority=10）。
 * @note   不直接访问 SPI/FIFO，只将当前 task id 传给 icm42688_service::Run(self_id)。
 */
void ImuDrdyTask(SchedulerTaskId self_id, SchedulerRunReason reason, SchedulerEventMask events,
                 uint32_t now_ms, uint64_t now_us, void *context)
{
    (void)reason; (void)events; (void)now_ms; (void)now_us; (void)context;

    icm42688_service::Run(self_id);
    (void)aided_ins_service::Run();
}

// ============================================================================
// 姿态遥测 task — 低优先级定时输出（默认关闭）
// ============================================================================

/**
 * @brief  姿态遥测 printf task（25 Hz，priority=220，默认关闭）。
 * @note   不访问 SPI/FIFO/EXTI/ICM42688P，不修改 INS 状态。
 *         按需启用 AIDED_INS_ENABLE_ATTITUDE_TELEMETRY 后以 25 Hz 输出姿态。
 *         输出模式由 AIDED_INS_ATTITUDE_TELEMETRY_MODE 控制：
 *           MODE_QUAT(0)       — qw,qx,qy,qz（q_b^n: FRD→NED）
 *           MODE_EULER_DEG(1)  — roll_deg,pitch_deg,yaw_deg（NED/FRD 约定）
 *         仅 INS Running 后才输出；初始对准期间静默。
 */
void AttitudeTelemetryTask(SchedulerTaskId self_id, SchedulerRunReason reason, SchedulerEventMask events,
                           uint32_t now_ms, uint64_t now_us, void *context)
{
    (void)self_id; (void)reason; (void)events; (void)now_ms; (void)now_us; (void)context;

    // 初始对准完成前不输出姿态
    if (!aided_ins_service::IsInsRunning()) { return; }

    const auto att = aided_ins_service::GetAttitudeTelemetry();

#if AIDED_INS_ATTITUDE_TELEMETRY_MODE == AIDED_INS_ATTITUDE_TELEMETRY_MODE_QUAT
    printf("%.6f,%.6f,%.6f,%.6f\n",
           static_cast<double>(att.qw),
           static_cast<double>(att.qx),
           static_cast<double>(att.qy),
           static_cast<double>(att.qz));
#else
    printf("%.3f,%.3f,%.3f\n",
           static_cast<double>(att.roll_deg),
           static_cast<double>(att.pitch_deg),
           static_cast<double>(att.yaw_deg));
#endif
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
        "imu_debug",                        // IMU 状态 print task，默认关闭；不访问 SPI/FIFO/EXTI
        ImuDebugTask,
        nullptr,
        200u,
        0u,
        IMU_DEBUG_PERIOD_MS,
        0u,
        0u,
        0u,                                 // enabled=0：由 kAppTasks 中 enabled 字段控制注册/运行
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
        "ins_consumer",                     // INS consumer task：定期消费 200 Hz 聚合 IMU
        InsConsumerTask,
        nullptr,
        20u,                                // priority: 低于 imu_drdy(10)，高于 debug tasks
        0u,
        INS_CONSUMER_PERIOD_MS,
        0u,
        0u,
        1u,
    },
    {
        "ins_debug",                        // Aided INS Service 统计 print task（周期由 INS_DEBUG_PERIOD_MS 决定，默认关闭）
        InsDebugTask,
        nullptr,
        210u,                               // priority: 低于 IMU driver debug
        0u,
        INS_DEBUG_PERIOD_MS,
        0u,
        0u,
        INS_DEBUG_TASK_ENABLED,
    },
    {
        "att_telem",                        // 姿态遥测 printf（25 Hz，默认关闭）
        AttitudeTelemetryTask,
        nullptr,
        220u,                               // priority: 低于 ins_debug(210)
        0u,
        ATTITUDE_TELEMETRY_PERIOD_MS,
        0u,
        0u,
        ATTITUDE_TELEMETRY_TASK_ENABLED,
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
        }
    }
}
