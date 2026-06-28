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
#include "IST8310_Calibration.hpp"
#include "IST8310_DebugConfig.hpp"
#include "IST8310_Service.hpp"
#include "IST8310_Registers.hpp"
#include "i2c.h"
#include "main.h"
#include <stdio.h>
#include <cmath>

namespace
{

constexpr uint32_t IMU_DEBUG_PERIOD_MS = 1000u;
constexpr uint32_t INS_DEBUG_PERIOD_MS = 5000u;
constexpr uint32_t INS_CONSUMER_PERIOD_MS = 1u;
constexpr uint32_t MAG_TASK_PERIOD_MS = 1u;
constexpr uint32_t MAG_DEBUG_PERIOD_MS = 1000u;
constexpr uint8_t MAG_DEBUG_TASK_ENABLED =
    IST8310_ENABLE_MAG_DEBUG_PRINT ? 1u : 0u;
constexpr uint8_t INS_DEBUG_TASK_ENABLED =
    (AIDED_INS_ENABLE_SERVICE_PRINT ||
     AIDED_INS_ENABLE_PROFILING_PRINT ||
     AIDED_INS_ENABLE_DEBUG_PRINT) ? 1u : 0u;
constexpr uint32_t IMU_DRDY_DEADLINE_MS = 5u;
constexpr uint32_t ATTITUDE_TELEMETRY_PERIOD_MS = 40u;  // 25 Hz
constexpr uint8_t ATTITUDE_TELEMETRY_TASK_ENABLED =
    AIDED_INS_ENABLE_ATTITUDE_TELEMETRY ? 1u : 0u;

// -- candidate summary / recommendation 常量 --
constexpr float MAG_CAL_SELF_CHECK_MAX_STD_UT    = 5.0F;
constexpr float MAG_CAL_SELF_CHECK_MIN_MEAN_UT   = 30.0F;
constexpr float MAG_CAL_SELF_CHECK_MAX_MEAN_UT   = 60.0F;
constexpr float MAG_CAL_SELF_CHECK_MIN_NORM_UT   = 30.0F;
constexpr float MAG_CAL_SELF_CHECK_MAX_NORM_UT   = 60.0F;
constexpr float MAG_CAL_SELF_CHECK_MAX_ERR_RATIO = 0.30F;

constexpr float MAG_CAL_RECOMMEND_STD_RATIO     = 0.90F;
constexpr float MAG_CAL_MAXERR_SOFT_TOLERANCE   = 0.02F;
constexpr float MAG_CAL_RANGE_TIE_TOLERANCE_UT  = 0.5F;
constexpr float MAG_CAL_MAXERR_TIE_TOLERANCE    = 0.005F;

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
// IST8310 mag task — 磁力计非阻塞周期性采样
// ============================================================================

/**
 * @brief  IST8310 磁力计周期性采样 task（1 ms，priority=30）。
 * @note   本 callback 本身不直接访问 I2C 寄存器、不直接调用 driver 接口；
 *         实际采样由 ist8310_service::Run() 内部非阻塞状态机推进。
 *         不接 Aided_INS，不触发 MagUpdate。
 */
void MagTask(SchedulerTaskId self_id, SchedulerRunReason reason, SchedulerEventMask events,
             uint32_t now_ms, uint64_t now_us, void *context)
{
    (void)self_id; (void)reason; (void)events; (void)now_ms; (void)now_us; (void)context;
    ist8310_service::Run();
}

// ============================================================================
// IST8310 磁力计校准 task — 按键触发 + 30s 收集 + printf 输出参数
// ============================================================================

enum class MagCalUiState : uint8_t
{
    Idle,
    WaitingRelease,
    Collecting,
    Success,
    Failed,
};

/**
 * @brief  IST8310 磁力计 MCU 端校准 task（10 ms，priority=100）。
 * @note   按键触发（PE15, PULLDOWN, 按下=SET）。
 *         收集 30s body-frame uT sample，计算 hard-iron bias + 三轴 scale。
 *         不控制 LED，不访问 driver/I2C，不接 Aided_INS。
 *         校准结果通过 printf 输出，用户手动复制到代码常量后重新编译烧录。
 */
void MagCalTask(SchedulerTaskId self_id, SchedulerRunReason reason, SchedulerEventMask events,
                uint32_t now_ms, uint64_t now_us, void *context)
{
    (void)self_id; (void)reason; (void)events; (void)now_us; (void)context;

    static MagCalUiState ui_state = MagCalUiState::Idle;
    static uint8_t  key_cnt = 0u;
    static uint32_t cal_start_ms = 0u;
    static uint32_t last_sample_counter = 0u;
    static uint32_t last_progress_ms = 0u;

    const bool key_down = (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET);

    switch (ui_state) {
    case MagCalUiState::Idle:
        if (key_down) {
            ++key_cnt;
            if (key_cnt >= 3u) {
                ui_state = MagCalUiState::WaitingRelease;
                key_cnt = 0u;
                printf("[mag_cal] key detected, release to start\r\n");
            }
        } else {
            key_cnt = 0u;
        }
        break;

    case MagCalUiState::WaitingRelease:
        if (!key_down) {
            ++key_cnt;
            if (key_cnt >= 3u) {
                ist8310_calibration::Reset();
                cal_start_ms = now_ms;
                last_sample_counter = 0u;
                last_progress_ms = now_ms;
                ui_state = MagCalUiState::Collecting;
                key_cnt = 0u;
                printf("[mag_cal] start: 30s, rotate board through all orientations\r\n");
#if IST8310_ENABLE_CAL_CSV_OUTPUT
                printf("[mag_csv] counter,timestamp_us,body_x_uT,body_y_uT,body_z_uT\r\n");
#endif
            }
        } else {
            key_cnt = 0u;
        }
        break;

    case MagCalUiState::Collecting: {
        // 按键在收集期间被忽略

        // 消费新 sample
        ist8310_service::MagSample s{};
        if (ist8310_service::CopyLatest(&s) && s.sample_counter != last_sample_counter) {
            last_sample_counter = s.sample_counter;
            ist8310_calibration::FeedSample(s.mag_uT_body);
#if IST8310_ENABLE_CAL_CSV_OUTPUT
            printf("[mag_csv] %lu,%llu,%.3f,%.3f,%.3f\r\n",
                   static_cast<unsigned long>(s.sample_counter),
                   static_cast<unsigned long long>(s.read_timestamp_us),
                   static_cast<double>(s.mag_uT_body[0]),
                   static_cast<double>(s.mag_uT_body[1]),
                   static_cast<double>(s.mag_uT_body[2]));
#endif
        }

        // 进度输出（每 10s 一次）
        if (now_ms - last_progress_ms >= 10000u) {
            last_progress_ms = now_ms;
            printf("[mag_cal] progress: %lu/30s samples=%lu\r\n",
                   static_cast<unsigned long>((now_ms - cal_start_ms) / 1000u),
                   static_cast<unsigned long>(ist8310_calibration::GetSampleCount()));
        }

        // 30s 到期
        if (now_ms - cal_start_ms >= 30000u) {
            printf("[mag_cal] done: calculating\r\n");
            const auto result = ist8310_calibration::Finish();

            if (result.valid) {
                printf("[mag_cal] success samples=%lu quality=%.2f radius_avg_uT=%.2f\r\n",
                       static_cast<unsigned long>(result.sample_count),
                       static_cast<double>(result.quality_score),
                       static_cast<double>(result.radius_avg_uT));
                ui_state = MagCalUiState::Success;

                // ── 可复制 C++ constexpr 参数块（仅成功时输出） ──
                printf("[mag_cal] // ===== copy to IST8310_Calibration_Config.hpp =====\r\n");
                printf("[mag_cal] // frame: body-frame (X forward, Y right, Z down)\r\n");
                printf("[mag_cal] // mapping: body_x=sensor_y, body_y=-sensor_x, body_z=-sensor_z\r\n");
                printf("[mag_cal] // formula: mag_cal = (mag_body - bias_body) * scale_body\r\n");
                printf("[mag_cal] // unit: uT\r\n");
                printf("[mag_cal] // samples=%lu quality=%.2f radius_avg_uT=%.2f\r\n",
                       static_cast<unsigned long>(result.sample_count),
                       static_cast<double>(result.quality_score),
                       static_cast<double>(result.radius_avg_uT));
                printf("[mag_cal] constexpr float kMagHardIronBiasBody_uT[3] = {\r\n");
                printf("[mag_cal]     %.2fF, %.2fF, %.2fF,\r\n",
                       static_cast<double>(result.bias_body_uT[0]),
                       static_cast<double>(result.bias_body_uT[1]),
                       static_cast<double>(result.bias_body_uT[2]));
                printf("[mag_cal] };\r\n");
                printf("[mag_cal]\r\n");
                printf("[mag_cal] constexpr float kMagScaleBody[3] = {\r\n");
                printf("[mag_cal]     %.2fF, %.2fF, %.2fF,\r\n",
                       static_cast<double>(result.scale_body[0]),
                       static_cast<double>(result.scale_body[1]),
                       static_cast<double>(result.scale_body[2]));
                printf("[mag_cal] };\r\n");
                printf("[mag_cal] // ===== end copy block =====\r\n");
            } else {
                printf("[mag_cal] failed samples=%lu\r\n"
                       "[mag_cal] reason: insufficient coverage or invalid scale\r\n",
                       static_cast<unsigned long>(result.sample_count));
                ui_state = MagCalUiState::Failed;
            }

            // 诊断统计（无论成功失败都输出）
            printf("[mag_cal] span_body_uT  = { %.2fF, %.2fF, %.2fF };\r\n",
                   static_cast<double>(result.span_body_uT[0]),
                   static_cast<double>(result.span_body_uT[1]),
                   static_cast<double>(result.span_body_uT[2]));
            printf("[mag_cal] bias_body_uT  = { %.2fF, %.2fF, %.2fF };\r\n",
                   static_cast<double>(result.bias_body_uT[0]),
                   static_cast<double>(result.bias_body_uT[1]),
                   static_cast<double>(result.bias_body_uT[2]));
            printf("[mag_cal] scale_body     = { %.2fF, %.2fF, %.2fF };\r\n",
                   static_cast<double>(result.scale_body[0]),
                   static_cast<double>(result.scale_body[1]),
                   static_cast<double>(result.scale_body[2]));
            // raw body-frame norm stats (auxiliary only, not ellipsoid-fit residual)
            printf("[mag_cal] norm_min_uT=%.2f norm_max_uT=%.2f norm_range_ratio=%.3f norm_out_of_range=%lu\r\n",
                   static_cast<double>(result.norm_min_uT),
                   static_cast<double>(result.norm_max_uT),
                   static_cast<double>(result.norm_range_ratio),
                   static_cast<unsigned long>(result.norm_out_of_range_count));

            // calibrated body-frame norm stats (self-validation: raw samples replayed with computed bias/scale)
            if (result.cal_norm_valid) {
                printf("[mag_cal] cal_norm min=%.2f max=%.2f mean=%.2f std=%.2f "
                       "range_ratio=%.3f max_err=%.3f samples=%lu dropped=%lu\r\n",
                       static_cast<double>(result.cal_norm_min_uT),
                       static_cast<double>(result.cal_norm_max_uT),
                       static_cast<double>(result.cal_norm_mean_uT),
                       static_cast<double>(result.cal_norm_std_uT),
                       static_cast<double>(result.cal_norm_range_ratio),
                       static_cast<double>(result.cal_norm_max_error_ratio),
                       static_cast<unsigned long>(result.cal_sample_count),
                       static_cast<unsigned long>(result.cal_sample_dropped_count));
            }

#if IST8310_ENABLE_ELLIPSOID_FIT
            // -- B1 / B2a candidate blocks + summary --
            {
                const auto (*buf)[3] = ist8310_calibration::GetSampleBuffer();
                const size_t n = ist8310_calibration::GetSampleBufferCount();

                bool                  has_efit = false;
                ist8310_calibration::EllipFitResult     efit{};
                bool                  has_ffit = false;
                ist8310_calibration::FullEllipFitResult ffit{};

                // -- A1 PX4-style sphere LM init candidate --
                bool has_sfit = false;
                ist8310_calibration::SphereLmFitResult sfit{};
                if (buf != nullptr && n >= 50u) {
                    sfit = ist8310_calibration::FitSphereLm(buf, n);
                    has_sfit = true;

                    printf("[mag_cal] // ===== PX4-style sphere LM init candidate =====\r\n");
                    printf("[mag_cal] // note: radius+offset only; diag/offdiag fixed to identity/zero; not for runtime config\r\n");
                    printf("[mag_cal] // fit_valid=%u solver_converged=%u iters=%u radius=%.2f cost_px4=%.4f rms=%.3f\r\n",
                           static_cast<unsigned int>(sfit.valid),
                           static_cast<unsigned int>(sfit.solver_converged),
                           static_cast<unsigned int>(sfit.iterations),
                           static_cast<double>(sfit.radius_uT),
                           static_cast<double>(sfit.cost_px4_uT),
                           static_cast<double>(sfit.rms_uT));
                    printf("[mag_cal] // sphere_lm_offset_body_uT = { %.2fF, %.2fF, %.2fF };\r\n",
                           static_cast<double>(sfit.offset_body_uT[0]),
                           static_cast<double>(sfit.offset_body_uT[1]),
                           static_cast<double>(sfit.offset_body_uT[2]));
                    printf("[mag_cal] // ===== end sphere LM init candidate =====\r\n");
                }

                // -- A2 PX4-style ellipsoid LM candidate --
                if (has_sfit && sfit.valid && buf != nullptr && n >= 100u) {
                    const auto e2fit = ist8310_calibration::FitEllipsoidLm(buf, n, sfit);

                    printf("[mag_cal] // ===== PX4-style ellipsoid LM candidate (candidate, review before copying) =====\r\n");
                    printf("[mag_cal] // fit_valid=%u solver_converged=%u iters=%u radius=%.2f cost_px4=%.4f rms=%.3f\r\n",
                           static_cast<unsigned int>(e2fit.valid),
                           static_cast<unsigned int>(e2fit.solver_converged),
                           static_cast<unsigned int>(e2fit.iterations),
                           static_cast<double>(e2fit.radius_uT),
                           static_cast<double>(e2fit.cost_px4_uT),
                           static_cast<double>(e2fit.rms_uT));
                    printf("[mag_cal] constexpr float kMagHardIronBiasBody_uT[3] = {\r\n");
                    printf("[mag_cal]     %.2fF, %.2fF, %.2fF,\r\n",
                           static_cast<double>(e2fit.offset_body_uT[0]),
                           static_cast<double>(e2fit.offset_body_uT[1]),
                           static_cast<double>(e2fit.offset_body_uT[2]));
                    printf("[mag_cal] };\r\n");
                    printf("[mag_cal]\r\n");
                    printf("[mag_cal] constexpr float kMagScaleBody[3] = {\r\n");
                    printf("[mag_cal]     %.2fF, %.2fF, %.2fF,\r\n",
                           static_cast<double>(e2fit.diag[0]),
                           static_cast<double>(e2fit.diag[1]),
                           static_cast<double>(e2fit.diag[2]));
                    printf("[mag_cal] };\r\n");
                    printf("[mag_cal]\r\n");
                    printf("[mag_cal] constexpr float kMagOffDiagScaleBody[3] = {\r\n");
                    printf("[mag_cal]     %.4fF, %.4fF, %.4fF,\r\n",
                           static_cast<double>(e2fit.offdiag[0]),
                           static_cast<double>(e2fit.offdiag[1]),
                           static_cast<double>(e2fit.offdiag[2]));
                    printf("[mag_cal] };\r\n");
                    printf("[mag_cal] // cal_norm min=%.2f max=%.2f mean=%.2f std=%.2f max_err=%.3f\r\n",
                           static_cast<double>(e2fit.cal_norm_min_uT),
                           static_cast<double>(e2fit.cal_norm_max_uT),
                           static_cast<double>(e2fit.cal_norm_mean_uT),
                           static_cast<double>(e2fit.cal_norm_std_uT),
                           static_cast<double>(e2fit.cal_norm_max_err));
                    printf("[mag_cal] // err_percentile p95=%.3f p99=%.3f\r\n",
                           static_cast<double>(e2fit.cal_norm_p95_err),
                           static_cast<double>(e2fit.cal_norm_p99_err));
                    printf("[mag_cal] // max_err_sample index=%lu raw={%.2f,%.2f,%.2f} cal={%.2f,%.2f,%.2f} norm=%.2f abs=%.2f ratio=%.3f\r\n",
                           static_cast<unsigned long>(e2fit.max_err_sample_index),
                           static_cast<double>(e2fit.max_err_raw_body_uT[0]),
                           static_cast<double>(e2fit.max_err_raw_body_uT[1]),
                           static_cast<double>(e2fit.max_err_raw_body_uT[2]),
                           static_cast<double>(e2fit.max_err_cal_body_uT[0]),
                           static_cast<double>(e2fit.max_err_cal_body_uT[1]),
                           static_cast<double>(e2fit.max_err_cal_body_uT[2]),
                           static_cast<double>(e2fit.max_err_cal_norm_uT),
                           static_cast<double>(e2fit.max_err_abs_uT),
                           static_cast<double>(e2fit.cal_norm_max_err));
                    printf("[mag_cal] // coverage octants={%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu} min=%lu max=%lu empty=%lu ratio=%.2f\r\n",
                           static_cast<unsigned long>(e2fit.coverage_octant_count[0]),
                           static_cast<unsigned long>(e2fit.coverage_octant_count[1]),
                           static_cast<unsigned long>(e2fit.coverage_octant_count[2]),
                           static_cast<unsigned long>(e2fit.coverage_octant_count[3]),
                           static_cast<unsigned long>(e2fit.coverage_octant_count[4]),
                           static_cast<unsigned long>(e2fit.coverage_octant_count[5]),
                           static_cast<unsigned long>(e2fit.coverage_octant_count[6]),
                           static_cast<unsigned long>(e2fit.coverage_octant_count[7]),
                           static_cast<unsigned long>(e2fit.coverage_min_octant_count),
                           static_cast<unsigned long>(e2fit.coverage_max_octant_count),
                           static_cast<unsigned long>(e2fit.coverage_empty_octant_count),
                           static_cast<double>(e2fit.coverage_octant_ratio));
                    printf("[mag_cal] // coverage hemi x+=%lu x-=%lu y+=%lu y-=%lu z+=%lu z-=%lu\r\n",
                           static_cast<unsigned long>(e2fit.coverage_hemi_count[0]),
                           static_cast<unsigned long>(e2fit.coverage_hemi_count[1]),
                           static_cast<unsigned long>(e2fit.coverage_hemi_count[2]),
                           static_cast<unsigned long>(e2fit.coverage_hemi_count[3]),
                           static_cast<unsigned long>(e2fit.coverage_hemi_count[4]),
                           static_cast<unsigned long>(e2fit.coverage_hemi_count[5]));
                    printf("[mag_cal] // coverage unit_axis_min={%.2f,%.2f,%.2f} unit_axis_max={%.2f,%.2f,%.2f} max_err_octant=%u\r\n",
                           static_cast<double>(e2fit.coverage_axis_min[0]),
                           static_cast<double>(e2fit.coverage_axis_min[1]),
                           static_cast<double>(e2fit.coverage_axis_min[2]),
                           static_cast<double>(e2fit.coverage_axis_max[0]),
                           static_cast<double>(e2fit.coverage_axis_max[1]),
                           static_cast<double>(e2fit.coverage_axis_max[2]),
                           static_cast<unsigned int>(e2fit.max_err_sample_octant));
                    printf("[mag_cal] // ===== end PX4-style ellipsoid LM candidate =====\r\n");
                }

                // -- B1 fixed-bias 3x3 candidate --
                if (buf != nullptr && n >= 50u) {
                    efit = ist8310_calibration::FitEllipsoidFixedBias(
                        buf, n, result.min_body_uT, result.max_body_uT);
                    has_efit = true;

                    printf("[mag_cal] // ===== fixed-bias 3x3 candidate (ellipsoid fit) =====\r\n");
                    printf("[mag_cal] // fit_valid=%u Q_posdef=%u cond=%.1f\r\n",
                           static_cast<unsigned int>(efit.valid),
                           static_cast<unsigned int>(efit.Q_positive_definite),
                           static_cast<double>(efit.condition_number));
                    printf("[mag_cal] constexpr float kMagHardIronBiasBody_uT[3] = {\r\n");
                    printf("[mag_cal]     %.2fF, %.2fF, %.2fF,\r\n",
                           static_cast<double>(efit.bias_body_uT[0]),
                           static_cast<double>(efit.bias_body_uT[1]),
                           static_cast<double>(efit.bias_body_uT[2]));
                    printf("[mag_cal] };\r\n");
                    printf("[mag_cal]\r\n");
                    printf("[mag_cal] constexpr float kMagSoftIronMatrix[3][3] = {\r\n");
                    for (int row = 0; row < 3; ++row) {
                        printf("[mag_cal]     { % .4fF, % .4fF, % .4fF },\r\n",
                               static_cast<double>(efit.matrix_3x3[row][0]),
                               static_cast<double>(efit.matrix_3x3[row][1]),
                               static_cast<double>(efit.matrix_3x3[row][2]));
                    }
                    printf("[mag_cal] };\r\n");
                    printf("[mag_cal] // cal_norm min=%.2f max=%.2f mean=%.2f std=%.2f max_err=%.3f\r\n",
                           static_cast<double>(efit.cal_norm_min_uT),
                           static_cast<double>(efit.cal_norm_max_uT),
                           static_cast<double>(efit.cal_norm_mean_uT),
                           static_cast<double>(efit.cal_norm_std_uT),
                           static_cast<double>(efit.cal_norm_max_err));
                }

                // -- B2a full-coupled algebraic candidate --
                if (buf != nullptr && n >= 100u) {
                    ffit = ist8310_calibration::FitEllipsoidFullAlgebraic(
                        buf, n, result.min_body_uT, result.max_body_uT);
                    has_ffit = true;

                    printf("[mag_cal] // ===== full-coupled algebraic 3x3 candidate =====\r\n");
                    printf("[mag_cal] // fit_valid=%u solver_converged=%u iters=%u Q_posdef=%u cond=%.1f k=%.6g R0=%.2f R_target=%.2f\r\n",
                           static_cast<unsigned int>(ffit.valid),
                           static_cast<unsigned int>(ffit.solver_converged),
                           static_cast<unsigned int>(ffit.solver_iters),
                           static_cast<unsigned int>(ffit.Q_positive_definite),
                           static_cast<double>(ffit.condition_number),
                           static_cast<double>(ffit.k_raw),
                           static_cast<double>(ffit.R0_uT),
                           static_cast<double>(ffit.R_target_uT));
                    printf("[mag_cal] constexpr float kMagHardIronBiasBody_uT[3] = {\r\n");
                    printf("[mag_cal]     %.2fF, %.2fF, %.2fF,\r\n",
                           static_cast<double>(ffit.bias_body_uT[0]),
                           static_cast<double>(ffit.bias_body_uT[1]),
                           static_cast<double>(ffit.bias_body_uT[2]));
                    printf("[mag_cal] };\r\n");
                    printf("[mag_cal]\r\n");
                    printf("[mag_cal] constexpr float kMagSoftIronMatrix[3][3] = {\r\n");
                    for (int row = 0; row < 3; ++row) {
                        printf("[mag_cal]     { % .4fF, % .4fF, % .4fF },\r\n",
                               static_cast<double>(ffit.matrix_3x3[row][0]),
                               static_cast<double>(ffit.matrix_3x3[row][1]),
                               static_cast<double>(ffit.matrix_3x3[row][2]));
                    }
                    printf("[mag_cal] };\r\n");
                    printf("[mag_cal] // cal_norm min=%.2f max=%.2f mean=%.2f std=%.2f max_err=%.3f\r\n",
                           static_cast<double>(ffit.cal_norm_min_uT),
                           static_cast<double>(ffit.cal_norm_max_uT),
                           static_cast<double>(ffit.cal_norm_mean_uT),
                           static_cast<double>(ffit.cal_norm_std_uT),
                           static_cast<double>(ffit.cal_norm_max_err));
                    printf("[mag_cal] // ===== end full-coupled candidate =====\r\n");
                } else if (buf != nullptr) {
                    printf("[mag_cal] // full-coupled algebraic fit skipped: n=%lu < 100\r\n",
                           static_cast<unsigned long>(n));
                }

                // -- Candidate summary / recommendation --
                if (has_efit || has_ffit) {
                    const bool diag_baseline_ok = result.valid && result.cal_norm_valid;

                    // Diagonal selfcheck
                    bool diag_selfcheck = false;
                    if (diag_baseline_ok) {
                        diag_selfcheck =
                               result.cal_norm_std_uT         < MAG_CAL_SELF_CHECK_MAX_STD_UT
                            && result.cal_norm_mean_uT        > MAG_CAL_SELF_CHECK_MIN_MEAN_UT
                            && result.cal_norm_mean_uT        < MAG_CAL_SELF_CHECK_MAX_MEAN_UT
                            && result.cal_norm_min_uT         > MAG_CAL_SELF_CHECK_MIN_NORM_UT
                            && result.cal_norm_max_uT         < MAG_CAL_SELF_CHECK_MAX_NORM_UT
                            && result.cal_norm_max_error_ratio < MAG_CAL_SELF_CHECK_MAX_ERR_RATIO;
                    }

                    // recommendation for a single candidate
                    auto calc_recommend = [&result, diag_baseline_ok](
                        bool cand_valid,
                        float cand_std, float cand_max_err,
                        float cand_min, float cand_max,
                        bool &std_ok, bool &max_err_ok, bool &max_err_soft,
                        bool &min_ok, bool &max_ok) -> bool
                    {
                        std_ok       = false;
                        max_err_ok   = false;
                        max_err_soft = false;
                        min_ok       = false;
                        max_ok       = false;

                        if (!diag_baseline_ok) { return false; }

                        std_ok       = cand_std    <= result.cal_norm_std_uT * MAG_CAL_RECOMMEND_STD_RATIO;
                        max_err_ok   = cand_max_err <= result.cal_norm_max_error_ratio;
                        max_err_soft = cand_max_err <= result.cal_norm_max_error_ratio + MAG_CAL_MAXERR_SOFT_TOLERANCE;
                        min_ok       = cand_min    >= result.cal_norm_min_uT;
                        max_ok       = cand_max    <= result.cal_norm_max_uT;
                        return cand_valid && std_ok && max_err_ok && min_ok && max_ok;
                    };

                    // B1 recommendation
                    bool b1_rec = false;
                    bool b1_std_ok = false;
                    bool b1_max_err_ok = false;
                    bool b1_max_err_soft = false;
                    bool b1_min_ok = false;
                    bool b1_max_ok = false;
                    if (has_efit) {
                        b1_rec = calc_recommend(efit.valid,
                            efit.cal_norm_std_uT, efit.cal_norm_max_err,
                            efit.cal_norm_min_uT, efit.cal_norm_max_uT,
                            b1_std_ok, b1_max_err_ok, b1_max_err_soft, b1_min_ok, b1_max_ok);
                    }

                    // B2a recommendation
                    bool b2_rec = false;
                    bool b2_std_ok = false;
                    bool b2_max_err_ok = false;
                    bool b2_max_err_soft = false;
                    bool b2_min_ok = false;
                    bool b2_max_ok = false;
                    if (has_ffit) {
                        b2_rec = calc_recommend(ffit.valid,
                            ffit.cal_norm_std_uT, ffit.cal_norm_max_err,
                            ffit.cal_norm_min_uT, ffit.cal_norm_max_uT,
                            b2_std_ok, b2_max_err_ok, b2_max_err_soft, b2_min_ok, b2_max_ok);
                    }

                    // Select recommended candidate
                    const char *rec_label = diag_baseline_ok ? "diagonal" : "none";
                    const char *rec_reason = diag_baseline_ok ? "no_recommended_candidate" : "baseline_invalid";
                    if (b1_rec && b2_rec) {
                        // Both recommended: prefer lower max_err, then tighter range, then lower std
                        const float b1_range = efit.cal_norm_max_uT - efit.cal_norm_min_uT;
                        const float b2_range = ffit.cal_norm_max_uT - ffit.cal_norm_min_uT;
                        const float me_diff  = efit.cal_norm_max_err - ffit.cal_norm_max_err;

                        if (me_diff > MAG_CAL_MAXERR_TIE_TOLERANCE) {
                            rec_label = "full_coupled"; rec_reason = "max_err_better";
                        } else if (me_diff < -MAG_CAL_MAXERR_TIE_TOLERANCE) {
                            rec_label = "fixed_bias"; rec_reason = "max_err_better";
                        } else if (b1_range < b2_range - MAG_CAL_RANGE_TIE_TOLERANCE_UT) {
                            rec_label = "fixed_bias"; rec_reason = "range_tighter";
                        } else if (b2_range < b1_range - MAG_CAL_RANGE_TIE_TOLERANCE_UT) {
                            rec_label = "full_coupled"; rec_reason = "range_tighter";
                        } else if (efit.cal_norm_std_uT < ffit.cal_norm_std_uT) {
                            rec_label = "fixed_bias"; rec_reason = "std_lower";
                        } else if (ffit.cal_norm_std_uT < efit.cal_norm_std_uT) {
                            rec_label = "full_coupled"; rec_reason = "std_lower";
                        } else {
                            rec_label = "ambiguous"; rec_reason = "tie";
                        }
                    } else if (b1_rec) {
                        rec_label = "fixed_bias"; rec_reason = "only_recommended";
                    } else if (b2_rec) {
                        rec_label = "full_coupled"; rec_reason = "only_recommended";
                    }

                    // Print summary
                    printf("[mag_cal] // ===== candidate summary =====\r\n");
                    printf("[mag_cal] // samples=%lu norm_out_of_range=%lu\r\n",
                           static_cast<unsigned long>(result.sample_count),
                           static_cast<unsigned long>(result.norm_out_of_range_count));
                    printf("[mag_cal] // diagonal: selfcheck=%s std=%.2f max_err=%.3f min=%.2f max=%.2f\r\n",
                           !result.valid ? "FAIL" : (!result.cal_norm_valid ? "NO_CAL_NORM" : (diag_selfcheck ? "PASS" : "WARN")),
                           static_cast<double>(result.cal_norm_std_uT),
                           static_cast<double>(result.cal_norm_max_error_ratio),
                           static_cast<double>(result.cal_norm_min_uT),
                           static_cast<double>(result.cal_norm_max_uT));
                    if (has_efit) {
                        printf("[mag_cal] // fixed_bias: valid=%u recommended=%u std=%.2f max_err=%.3f min=%.2f max=%.2f std_ok=%u max_err_ok=%u max_err_soft=%u min_ok=%u max_ok=%u\r\n",
                               static_cast<unsigned int>(efit.valid),
                               static_cast<unsigned int>(b1_rec),
                               static_cast<double>(efit.cal_norm_std_uT),
                               static_cast<double>(efit.cal_norm_max_err),
                               static_cast<double>(efit.cal_norm_min_uT),
                               static_cast<double>(efit.cal_norm_max_uT),
                               static_cast<unsigned int>(b1_std_ok),
                               static_cast<unsigned int>(b1_max_err_ok),
                               static_cast<unsigned int>(b1_max_err_soft),
                               static_cast<unsigned int>(b1_min_ok),
                               static_cast<unsigned int>(b1_max_ok));
                    }
                    if (has_ffit) {
                        printf("[mag_cal] // full_coupled: valid=%u recommended=%u std=%.2f max_err=%.3f min=%.2f max=%.2f std_ok=%u max_err_ok=%u max_err_soft=%u min_ok=%u max_ok=%u\r\n",
                               static_cast<unsigned int>(ffit.valid),
                               static_cast<unsigned int>(b2_rec),
                               static_cast<double>(ffit.cal_norm_std_uT),
                               static_cast<double>(ffit.cal_norm_max_err),
                               static_cast<double>(ffit.cal_norm_min_uT),
                               static_cast<double>(ffit.cal_norm_max_uT),
                               static_cast<unsigned int>(b2_std_ok),
                               static_cast<unsigned int>(b2_max_err_ok),
                               static_cast<unsigned int>(b2_max_err_soft),
                               static_cast<unsigned int>(b2_min_ok),
                               static_cast<unsigned int>(b2_max_ok));
                    }
                    printf("[mag_cal] // recommended=%s reason=%s\r\n", rec_label, rec_reason);
                    if (result.norm_out_of_range_count > 0u) {
                        printf("[mag_cal] // note: raw_norm_out_of_range is auxiliary only; hard-iron bias can make raw norm leave nominal field range\r\n");
                        printf("[mag_cal] // note: use cal_norm/selfcheck/recommended fields for calibration quality\r\n");
                    }
                    printf("[mag_cal] // ===== end candidate summary =====\r\n");
                }
            }
#endif

            key_cnt = 0u;
        }
        break;
    }

    case MagCalUiState::Success:
    case MagCalUiState::Failed:
        if (key_down) {
            ++key_cnt;
            if (key_cnt >= 3u) {
                ui_state = MagCalUiState::Idle;
                key_cnt = 0u;
                printf("[mag_cal] idle\r\n");
            }
        } else {
            key_cnt = 0u;
        }
        break;
    }
}

// ============================================================================
// IST8310 mag debug print task — 硬件验证用低频输出（默认关闭）
// ============================================================================

/**
 * @brief  IST8310 磁力计 debug print task（1 Hz，priority=205，默认关闭）。
 * @note   只读取 ist8310_service::CopyLatest() 缓存，不访问 driver/I2C/Aided_INS。
 *
 *         **启用后仅用于硬件验证**：printf 可能阻塞 cooperative scheduler
 *         数 ms 到十几 ms（浮点格式化 + UART TX），应同时观察 IMU FIFO/INS
 *         是否受影响，不应长期在飞行运行路径中开启。
 */
void MagDebugTask(SchedulerTaskId self_id, SchedulerRunReason reason, SchedulerEventMask events,
                  uint32_t now_ms, uint64_t now_us, void *context)
{
    (void)self_id; (void)reason; (void)events; (void)now_ms; (void)now_us; (void)context;

    if (ist8310_service::IsFault()) {
        static uint32_t s_fault_counter = 0u;
        if ((s_fault_counter++ & 0x0Fu) == 0u) {
            printf("[mag] fault\r\n");
        }
        return;
    }

    if (!ist8310_service::IsStarted()) {
        static uint32_t s_not_started_counter = 0u;
        if ((s_not_started_counter++ & 0x0Fu) == 0u) {
            printf("[mag] not started\r\n");
        }
        return;
    }

    ist8310_service::MagSample sample{};
    if (!ist8310_service::CopyLatest(&sample)) {
        return;
    }

    static uint32_t s_last_mag_counter = 0u;
    if (sample.sample_counter == s_last_mag_counter) {
        return;
    }
    s_last_mag_counter = sample.sample_counter;

    const float cal_norm = std::sqrt(
        sample.mag_uT_body_calibrated[0] * sample.mag_uT_body_calibrated[0] +
        sample.mag_uT_body_calibrated[1] * sample.mag_uT_body_calibrated[1] +
        sample.mag_uT_body_calibrated[2] * sample.mag_uT_body_calibrated[2]);

    printf("[mag] c=%lu body_raw=%d %d %d body_uT=%.1f %.1f %.1f "
           "cal_uT=%.1f %.1f %.1f cal_norm=%.1f cal=%u "
           "sensor_raw=%d %d %d err=%lu ovr=%lu\r\n",
           static_cast<unsigned long>(sample.sample_counter),
           static_cast<int>(sample.raw_body[0]),
           static_cast<int>(sample.raw_body[1]),
           static_cast<int>(sample.raw_body[2]),
           static_cast<double>(sample.mag_uT_body[0]),
           static_cast<double>(sample.mag_uT_body[1]),
           static_cast<double>(sample.mag_uT_body[2]),
           static_cast<double>(sample.mag_uT_body_calibrated[0]),
           static_cast<double>(sample.mag_uT_body_calibrated[1]),
           static_cast<double>(sample.mag_uT_body_calibrated[2]),
           static_cast<double>(cal_norm),
           static_cast<unsigned int>(sample.calibration_applied),
           static_cast<int>(sample.raw_sensor[0]),
           static_cast<int>(sample.raw_sensor[1]),
           static_cast<int>(sample.raw_sensor[2]),
           static_cast<unsigned long>(sample.error_counter),
           static_cast<unsigned long>(sample.overrun_counter));
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
        "mag",                              // IST8310 磁力计非阻塞周期性采样
        MagTask,
        nullptr,
        30u,                                // priority: 低于 imu_drdy(10) 和 ins_consumer(20)
        0u,
        MAG_TASK_PERIOD_MS,
        0u,
        0u,
        1u,                                 // enabled=1
    },
    {
        "mag_cal",                          // IST8310 磁力计 MCU 端校准 task（10 ms）
        MagCalTask,
        nullptr,
        100u,                               // priority: 低于 imu_drdy(10)/ins(20)/mag(30)，高于 debug(200+)
        0u,
        10u,                                // period_ms: 10 ms
        0u,
        0u,
        1u,                                 // enabled=1
    },
    {
        "mag_debug",                        // IST8310 debug print task（1 Hz，默认关闭，仅硬件验证）
        MagDebugTask,
        nullptr,
        205u,                               // priority: debug 区间，低于 mag(30)
        0u,
        MAG_DEBUG_PERIOD_MS,
        0u,
        0u,
        MAG_DEBUG_TASK_ENABLED,
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
 * @brief  应用层 Service 初始化入口。
 * @note   初始化 Aided INS Service 和 IST8310 Service；
 *         本函数不注册 scheduler task；任务注册由 SchedulerAppTasks_RegisterAll() 负责。
 */
extern "C" void App_Init(void)
{
    (void)aided_ins_service::Init();
    (void)ist8310_service::Init(&hi2c1,
                                IST8310_Regs::DATASHEET_DEFAULT_I2C_ADDRESS_7BIT,
                                MAG_RSTN_GPIO_Port,
                                MAG_RSTN_Pin);
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
