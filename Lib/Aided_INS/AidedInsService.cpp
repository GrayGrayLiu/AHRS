/**
 * @file    AidedInsService.cpp
 * @brief   Aided INS Service 实现
 *
 * @details
 * 聚合逻辑：
 *   1. 读取 icm42688_service::GetLatest() 获取最新 Sample；
 *   2. 判断 sample 有效性：data_valid、delta_time_s、delta_samples；
 *   3. 利用 timestamp_us 去重（跳过与上一 sample 相同时间戳的 sample）；
 *   4. 每两个有效 400 Hz sample 聚合成一个 200 Hz IMU delta；
 *   5. 聚合完成后缓存 200 Hz IMU（latest_imu200_）；
 *      默认不在 IMU DRDY producer 路径同步运行 INS；
 *      由独立 ins_consumer task 通过 RunInsConsumerOnce() 消费缓存，
 *      注入 Aided_INS 并调用 Aided_INS::Run()。
 *      kRunInsInDrdyForProfiling=true 时仍在 producer 路径同步调用（仅 profiling）。
 */

#include "AidedInsService.hpp"
#include "Aided_INS.hpp"
#include "ICM42688_Service.hpp"
#include "SystemPort.h"

#include <cstdint>

namespace aided_ins_service
{
    using IMU = Aided_INS_Space::IMU;

    namespace
    {
        // producer/consumer 解耦模式。
        // 历史观测：ins_consumer 中 Aided_INS::Run() 约 0.93 ms，明显长于 IMU DRDY producer/service
        // 路径的数微秒级耗时；因此 INS 不应在 DRDY producer 路径同步运行（会阻塞后续 400 Hz sample），
        // 当前默认由独立 ins_consumer task（1 ms polling）异步消费。
        constexpr bool kRunInsInDrdyForProfiling = false;

        bool      pending_{false};
        IMU       pending_imu_{};
        uint64_t  last_timestamp_us_{0u};
        bool      has_last_timestamp_{false};
        IMU       latest_imu200_{};
        bool      has_imu200_{false};
        Stats     stats_{};

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

        Aided_INS &InsInstance()
        {
            static Aided_INS aided_ins{0};
            return aided_ins;
        }

        // 抽取重复的 profile→stats 拷贝逻辑，供同步 profiling 分支和 consumer 共用
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
            s.acc_p_phkt_us = p.acc_p_phkt_us;
            if (p.acc_p_phkt_us > s.acc_p_phkt_max) { s.acc_p_phkt_max = p.acc_p_phkt_us; }
            s.acc_p_ksk_us = p.acc_p_ksk_us;
            if (p.acc_p_ksk_us > s.acc_p_ksk_max) { s.acc_p_ksk_max = p.acc_p_ksk_us; }
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
    } // namespace

    int Init()
    {
        ResetAggregationState();
        ResetStats();
        return InsInstance().Init();
    }

    Stats GetStats()
    {
        return stats_;
    }

    AttitudeTelemetry GetAttitudeTelemetry()
    {
        AttitudeTelemetry t{};
        const auto &pva = InsInstance().GetCurrentPVA();
        t.timestamp = InsInstance().GetCurrentTimestamp();
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

    bool IsInsRunning()
    {
        return InsInstance().IsRunning();
    }

    int RunInsConsumerOnce()
    {
        if (!has_imu200_) { return 0; }

        const IMU local_imu = latest_imu200_;
        has_imu200_ = false;

        InsInstance().SetImuData(local_imu);

        const uint64_t t_ins_start = SystemPort_GetMicros();
        InsInstance().Run();
        const uint64_t t_ins_end = SystemPort_GetMicros();

        const uint32_t ins_elapsed = static_cast<uint32_t>(t_ins_end - t_ins_start);
        stats_.ins_run_last_us = ins_elapsed;
        if (ins_elapsed > stats_.ins_run_max_us) { stats_.ins_run_max_us = ins_elapsed; }
        ++stats_.ins_run_calls;

        CopyInsProfileToStats(InsInstance().GetLastProfile(), stats_);

#if AIDED_INS_ENABLE_COV_HEALTH_CHECK
        {
            const auto &h = InsInstance().GetCovHealth();
            stats_.cov_has_nan_inf       = h.has_nan_inf;
            stats_.cov_has_neg_diag      = h.has_neg_diag;
            stats_.cov_max_asymmetry_last = h.max_asymmetry_last;
            stats_.cov_max_asymmetry_max  = h.max_asymmetry_max;
            stats_.cov_nan_inf_count      = h.nan_inf_count;
            stats_.cov_neg_diag_count     = h.neg_diag_count;
            stats_.cov_check_count        = h.check_count;
        }
#endif

        return 1;
    }

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

        // 7. 临时诊断开关：选择聚合后的行为。
        if (kRunInsInDrdyForProfiling)
        {
            // 临时：重新调用 Aided_INS::Run() 以测量 hard-float 下 INS 耗时。
            InsInstance().SetImuData(pending_imu_);

            const uint64_t t_ins_start = SystemPort_GetMicros();
            InsInstance().Run();
            const uint64_t t_ins_end = SystemPort_GetMicros();

            const uint32_t ins_elapsed = static_cast<uint32_t>(t_ins_end - t_ins_start);
            stats_.ins_run_last_us = ins_elapsed;

            if (ins_elapsed > stats_.ins_run_max_us)
            {
                stats_.ins_run_max_us = ins_elapsed;
            }

            ++stats_.ins_run_calls;

            CopyInsProfileToStats(InsInstance().GetLastProfile(), stats_);

#if AIDED_INS_ENABLE_COV_HEALTH_CHECK
            {
                const auto &h = InsInstance().GetCovHealth();
                stats_.cov_has_nan_inf       = h.has_nan_inf;
                stats_.cov_has_neg_diag      = h.has_neg_diag;
                stats_.cov_max_asymmetry_last = h.max_asymmetry_last;
                stats_.cov_max_asymmetry_max  = h.max_asymmetry_max;
                stats_.cov_nan_inf_count      = h.nan_inf_count;
                stats_.cov_neg_diag_count     = h.neg_diag_count;
                stats_.cov_check_count        = h.check_count;
            }
#endif
        }
        else
        {
            // 保留 producer-only 缓存逻辑。
            if (has_imu200_)
            {
                ++stats_.dropped_imu_samples;
            }
            latest_imu200_ = pending_imu_;
            has_imu200_ = true;
            ++stats_.queued_imu_samples;
        }

        pending_ = false;

        RecordServiceRunElapsed(t_total_start);
        return 1;
    }
} // namespace aided_ins_service
