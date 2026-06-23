/**
 * @file    AidedInsService.cpp
 * @brief   Aided INS Service 实现
 *
 * @details
 * Service 层职责：
 *   - 拥有并管理全工程唯一的 Aided_INS 算法实例；
 *   - 提供 Init() 作为算法完整初始化入口；
 *   - 提供 Run() 作为 IMU DRDY producer 路径，读取 ICM42688P 最新 sample，
 *     完成有效性检查、timestamp 去重/gap 检测、400 Hz ×2 → 200 Hz 聚合；
 *   - 提供 RunInsConsumerOnce() 作为 INS consumer 路径，消费缓存的 200 Hz IMU，
 *     注入 Aided_INS 并调用 Aided_INS::Run()；
 *   - 提供 GetStats/GetAttitudeTelemetry/IsInsRunning 只读观测接口。
 *
 * 不负责：
 *   - ICM42688P SPI/FIFO/INT/DRDY 驱动细节；
 *   - EKF/INS 数学公式和结构化矩阵计算实现；
 *   - scheduler 调度策略。
 */

#include "AidedInsService.hpp"

#include <new>
#include <cstdint>

#include "Aided_INS.hpp"
#include "ICM42688_Service.hpp"
#include "SystemPort.h"

namespace aided_ins_service
{
    using IMU = Aided_INS_Space::IMU;

    namespace
    {
        // ============================================================================
        // 唯一 Aided_INS 实例与 Service 状态
        // ============================================================================

        // Aided_INS 需要在构造时绑定实例 id。裸机环境不使用动态内存，
        // 因此在静态存储上通过 placement new 构造全工程唯一的 INS 实例。
        // 完整算法初始化仍由 aided_ins_service::Init() 调用 Aided_INS::Init() 完成。
        alignas(Aided_INS) uint8_t aided_ins_storage[sizeof(Aided_INS)]{};
        Aided_INS *aided_ins = nullptr;

        // bound：C++ 对象已构造并绑定实例 id（placement new 完成）
        bool aided_ins_bound = false;

        // producer/consumer 解耦模式。
        // 观测条件：STM32H723 (cortex-m7)，FPU fpv5-d16 硬浮点，RelWithDebInfo 工程配置，
        // runtime printf 及 profiling/debug/telemetry 输出默认关闭。
        // 在上述条件下：ins_consumer 中 Aided_INS::Run() 约 0.93 ms，明显长于
        // IMU DRDY producer/service 路径的数微秒级耗时；因此 INS 不在 DRDY producer
        // 路径同步运行，由独立 ins_consumer task（1 ms polling）异步消费。
        // 上述耗时为历史测量参考，不作为跨编译配置、跨平台或最坏情况实时上界。

        bool      pending_{false};
        IMU       pending_imu_{};
        uint64_t  last_timestamp_us_{0u};
        bool      has_last_timestamp_{false};
        IMU       latest_imu200_{};
        bool      has_imu200_{false};
        Stats     stats_{};

        // ============================================================================
        // 通用小工具函数
        // ============================================================================

        void ResetStats() { stats_ = Stats{}; }

        void RecordServiceRunElapsed(const uint64_t start_us)
        {
            const uint32_t elapsed_us = static_cast<uint32_t>(SystemPort_GetMicros() - start_us);
            stats_.service_run_last_us = elapsed_us;
            if (elapsed_us > stats_.service_run_max_us) { stats_.service_run_max_us = elapsed_us; }
        }

        void ResetAggregationState()
        {
            pending_ = false;
            pending_imu_ = IMU{};
            last_timestamp_us_ = 0u;
            has_last_timestamp_ = false;
            latest_imu200_ = IMU{};
            has_imu200_ = false;
        }

        // ============================================================================
        // 初始化流程
        // ============================================================================

        Aided_INS *BindAidedIns()
        {
            if (!aided_ins_bound) {
                aided_ins = ::new (static_cast<void *>(aided_ins_storage)) Aided_INS(0);
                aided_ins_bound = true;
            }

            return aided_ins;
        }

        // ============================================================================
        // 调试统计辅助
        // ============================================================================

        // 将最近一次 INS profile 快照拷贝到 service stats，供调试统计输出使用
        void CopyInsProfileToStats(const InsProfile &p, Stats &s)
        {
            s.pnd_us = p.process_new_data_us;
            if (p.process_new_data_us > s.pnd_max) { s.pnd_max = p.process_new_data_us; }
            s.prp_us = p.ins_propagation_us;
            if (p.ins_propagation_us > s.prp_max) { s.prp_max = p.ins_propagation_us; }
            s.mec_us = p.ins_mech_us;
            if (p.ins_mech_us > s.mec_max) { s.mec_max = p.ins_mech_us; }
            s.fmx_us = p.phi_f_q_g_us;
            if (p.phi_f_q_g_us > s.fmx_max) { s.fmx_max = p.phi_f_q_g_us; }
            s.ekf_us = p.ekf_predict_us;
            if (p.ekf_predict_us > s.ekf_max) { s.ekf_max = p.ekf_predict_us; }
            s.afb_us = p.acc_feedback_us;
            if (p.acc_feedback_us > s.afb_max) { s.afb_max = p.acc_feedback_us; }
            s.alc_us = p.fmx_alc_us;
            if (p.fmx_alc_us > s.alc_max) { s.alc_max = p.fmx_alc_us; }
            s.fill_us = p.fmx_fill_us;
            if (p.fmx_fill_us > s.fill_max) { s.fill_max = p.fmx_fill_us; }
            s.q1_us = p.fmx_q1_us;
            if (p.fmx_q1_us > s.q1_max) { s.q1_max = p.fmx_q1_us; }
            s.q2_us = p.fmx_q2_us;
            if (p.fmx_q2_us > s.q2_max) { s.q2_max = p.fmx_q2_us; }
            s.acc_try        = p.acc_try;
            s.acc_accept     = p.acc_accept;
            s.acc_fail_small = p.acc_fail_small;
            s.acc_fail_norm  = p.acc_fail_norm;
            s.acc_fail_cos   = p.acc_fail_cos;
            s.acc_feedback   = p.acc_feedback;
            s.f_norm      = p.last_f_norm;
            s.f_gravity   = p.last_gravity;
            s.f_norm_diff = p.last_norm_diff;
            s.f_cos_gn_gb = p.last_cos_gn_gb;
            s.f_f_b[0] = p.last_f_b[0]; s.f_f_b[1] = p.last_f_b[1]; s.f_f_b[2] = p.last_f_b[2];
            s.f_g_b_ByImu[0] = p.last_g_b_ByImu[0]; s.f_g_b_ByImu[1] = p.last_g_b_ByImu[1]; s.f_g_b_ByImu[2] = p.last_g_b_ByImu[2];
            s.ekf_dx_us        = p.ekf_dx_us;
            s.ekf_phi_p_us     = p.ekf_phi_p_us;
            s.ekf_m_phi_t_q_us = p.ekf_m_phi_t_q_us;
            s.acc_prep_us = p.acc_prep_us;
            if (p.acc_prep_us > s.acc_prep_max) { s.acc_prep_max = p.acc_prep_us; }
            s.acc_ekf_us = p.acc_ekf_us;
            if (p.acc_ekf_us > s.acc_ekf_max) { s.acc_ekf_max = p.acc_ekf_us; }
            s.feedback_us = p.feedback_us;
            if (p.feedback_us > s.feedback_max) { s.feedback_max = p.feedback_us; }
            s.acc_phs_us = p.acc_phs_us;
            if (p.acc_phs_us > s.acc_phs_max) { s.acc_phs_max = p.acc_phs_us; }
            s.acc_kdx_us = p.acc_kdx_us;
            if (p.acc_kdx_us > s.acc_kdx_max) { s.acc_kdx_max = p.acc_kdx_us; }
            s.acc_p_khp_us = p.acc_p_khp_us;
            if (p.acc_p_khp_us > s.acc_p_khp_max) { s.acc_p_khp_max = p.acc_p_khp_us; }
            s.euler_r = p.euler_r_deg;
            s.euler_p = p.euler_p_deg;
            s.euler_y = p.euler_y_deg;
            s.g_l_n[0] = p.g_l_n[0]; s.g_l_n[1] = p.g_l_n[1]; s.g_l_n[2] = p.g_l_n[2];
            s.g_b[0] = p.g_b_ByImu[0]; s.g_b[1] = p.g_b_ByImu[1]; s.g_b[2] = p.g_b_ByImu[2];
            s.cbn_f[0] = p.cbn_f_b[0]; s.cbn_f[1] = p.cbn_f_b[1]; s.cbn_f[2] = p.cbn_f_b[2];
            s.cbn_f_g[0] = p.cbn_f_plus_g[0]; s.cbn_f_g[1] = p.cbn_f_plus_g[1]; s.cbn_f_g[2] = p.cbn_f_plus_g[2];
            s.cfn_g_norm = p.cbn_f_plus_g_norm;
            s.cos_fg = p.cos_f_gb;
        }

#if AIDED_INS_ENABLE_COV_HEALTH_CHECK
        void CopyCovHealthToStats(Stats &s)
        {
            if (aided_ins == nullptr) {
                return;
            }

            const auto &h = aided_ins->GetCovHealth();
            s.cov_has_nan_inf       = h.has_nan_inf;
            s.cov_has_neg_diag      = h.has_neg_diag;
            s.cov_max_asymmetry_last = h.max_asymmetry_last;
            s.cov_max_asymmetry_max  = h.max_asymmetry_max;
            s.cov_nan_inf_count      = h.nan_inf_count;
            s.cov_neg_diag_count     = h.neg_diag_count;
            s.cov_check_count        = h.check_count;
        }
#endif
    } // namespace

    // ============================================================================
    // 对外 Service 接口
    // ============================================================================

    // 初始化 service 层：重置聚合状态、统计，并完成 Aided_INS 算法初始化。
    int Init()
    {
        ResetAggregationState();
        ResetStats();

        Aided_INS *ins = BindAidedIns();
        return ins->Init();
    }

    // 返回 service 统计快照（只读），供 ins_debug 等统计输出任务使用。
    Stats GetStats()
    {
        return stats_;
    }

    // 获取当前 INS 姿态遥测快照：只读访问 pvaCur_，不触发 INS 运算。
    // 用于 att_telem task 以低优先级输出四元数或欧拉角。
    AttitudeTelemetry GetAttitudeTelemetry()
    {
        AttitudeTelemetry t{};

        if (aided_ins == nullptr) {
            return t;
        }

        const auto &pva = aided_ins->GetCurrentPVA();
        t.timestamp = aided_ins->GetCurrentTimestamp();
        // NED/FRD 约定下的欧拉角（roll=X, pitch=Y, yaw=Z），rad→deg
        t.roll_deg  = static_cast<float>(pva.att.euler(0) * 57.29577951308232);
        t.pitch_deg = static_cast<float>(pva.att.euler(1) * 57.29577951308232);
        t.yaw_deg   = static_cast<float>(pva.att.euler(2) * 57.29577951308232);
        // 四元数 q_b^n：b 系→n 系（FRD→NED）
        t.qw = static_cast<float>(pva.att.qbn.w());
        t.qx = static_cast<float>(pva.att.qbn.x());
        t.qy = static_cast<float>(pva.att.qbn.y());
        t.qz = static_cast<float>(pva.att.qbn.z());
        return t;
    }

    // 供姿态遥测等调用方判断 INS 是否已完成初始对准；对准完成前不输出姿态。
    bool IsInsRunning()
    {
        return aided_ins != nullptr && aided_ins->IsRunning();
    }

    // consumer 路径：消费缓存的 200 Hz IMU 并运行一次 INS。
    // 由 ins_consumer task（1 ms polling，priority=20）调用。
    // 返回 0=无缓存数据或 INS 未绑定，1=成功执行 INS Run。
    int RunInsConsumerOnce()
    {
        if (!has_imu200_ || aided_ins == nullptr) { return 0; }

        const IMU local_imu = latest_imu200_;
        has_imu200_ = false;

        aided_ins->SetImuData(local_imu);

        const uint64_t t_ins_start = SystemPort_GetMicros();
        aided_ins->Run();
        const uint64_t t_ins_end = SystemPort_GetMicros();

        const uint32_t ins_elapsed = static_cast<uint32_t>(t_ins_end - t_ins_start);
        stats_.ins_run_last_us = ins_elapsed;
        if (ins_elapsed > stats_.ins_run_max_us) { stats_.ins_run_max_us = ins_elapsed; }
        ++stats_.ins_run_calls;

        CopyInsProfileToStats(aided_ins->GetLastProfile(), stats_);

#if AIDED_INS_ENABLE_COV_HEALTH_CHECK
        CopyCovHealthToStats(stats_);
#endif

        return 1;
    }

    // producer 路径：由 imu_drdy task（priority=10）调用。
    // 负责读取 sample、有效性检查、timestamp 去重/gap 检测、400 Hz ×2→200 Hz 聚合、
    // 缓存 latest_imu200_。不调用 Aided_INS::Run()。
    int Run()
    {
        const uint64_t t_total_start = SystemPort_GetMicros();
        ++stats_.run_calls;

        // 1. 读取最新 driver sample。
        ICM42688P::Sample sample{};
        const ICM42688P::Status status = icm42688_service::GetLatest(&sample);

        if (status != ICM42688P::Status::Ok) {
            ++stats_.get_latest_failures;
            pending_ = false;
            RecordServiceRunElapsed(t_total_start);
            return 0;
        }

        // 2. 跳过无效或异常数据，同时清空 pending（避免跨中断聚合）。
        if (!sample.data_valid || sample.delta_time_s <= 0.0f || sample.delta_samples == 0u) {
            ++stats_.invalid_samples;
            pending_ = false;
            RecordServiceRunElapsed(t_total_start);
            return 0;
        }

        // 3. 利用 timestamp_us 去重（重复调用 Service，不清 pending）。
        if (has_last_timestamp_ && sample.timestamp_us == last_timestamp_us_) {
            ++stats_.duplicate_samples;
            RecordServiceRunElapsed(t_total_start);
            return 0;
        }

        // 4. 时间跳变检测：逆序或明显间隔过大时，清空 pending 并以当前 sample 重新起点。
        if (has_last_timestamp_) {
            const uint64_t expected_us = static_cast<uint64_t>(sample.delta_time_s * 1.0e6F);

            if (sample.timestamp_us < last_timestamp_us_) {
                ++stats_.timestamp_errors;
                pending_ = false;
            } else if (expected_us != 0u) {
                const uint64_t max_allowed_gap_us = expected_us * 3u;

                if (sample.timestamp_us - last_timestamp_us_ > max_allowed_gap_us) {
                    ++stats_.timestamp_errors;
                    pending_     = false;
                }
            }
        }

        has_last_timestamp_ = true;
        last_timestamp_us_  = sample.timestamp_us;

        // 5. 转换为 INS 需要的 IMU 格式。
        IMU raw{};
        raw.time       = static_cast<double>(sample.timestamp_us) * 1.0e-6;
        raw.dt         = static_cast<double>(sample.delta_time_s);
        raw.deltaTheta = Eigen::Vector3d(sample.delta_angle_rad[0],
                                          sample.delta_angle_rad[1],
                                          sample.delta_angle_rad[2]);
        raw.deltaVel   = Eigen::Vector3d(sample.delta_velocity_m_s[0],
                                          sample.delta_velocity_m_s[1],
                                          sample.delta_velocity_m_s[2]);

        ++stats_.valid_samples;

        // 6. 两样本聚合（400 Hz ×2 → 200 Hz）。
        if (!pending_) {
            pending_     = true;
            pending_imu_ = raw;
            RecordServiceRunElapsed(t_total_start);
            return 0;
        }

        pending_imu_.deltaTheta += raw.deltaTheta;
        pending_imu_.deltaVel   += raw.deltaVel;
        pending_imu_.dt         += raw.dt;
        pending_imu_.time       = raw.time;

        ++stats_.aggregated_samples;

        // 缓存聚合后的 200 Hz IMU，由独立 ins_consumer task 异步消费。
        if (has_imu200_)
        {
            ++stats_.dropped_imu_samples;
        }
        latest_imu200_ = pending_imu_;
        has_imu200_ = true;
        ++stats_.queued_imu_samples;

        pending_ = false;

        RecordServiceRunElapsed(t_total_start);
        return 1;
    }
} // namespace aided_ins_service
