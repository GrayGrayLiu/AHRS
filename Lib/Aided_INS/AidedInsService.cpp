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
 *   5. 聚合完成后缓存 200 Hz IMU；当前诊断阶段暂不调用 Aided_INS::Run()。
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
        constexpr bool kRunInsInDrdyForProfiling = true;  // 临时诊断开关：true=聚合后调用 INS Run 测耗时

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

            // [PROFILE] 读取分段耗时并更新 stats last/max
            const InsProfile &p = InsInstance().GetLastProfile();
            stats_.pnd_us = p.process_new_data_us;
            if (p.process_new_data_us > stats_.pnd_max) { stats_.pnd_max = p.process_new_data_us; }
            stats_.prp_us = p.ins_propagation_us;
            if (p.ins_propagation_us > stats_.prp_max) { stats_.prp_max = p.ins_propagation_us; }
            stats_.mec_us = p.ins_mech_us;
            if (p.ins_mech_us > stats_.mec_max) { stats_.mec_max = p.ins_mech_us; }
            stats_.fmx_us = p.phi_f_q_g_us;
            if (p.phi_f_q_g_us > stats_.fmx_max) { stats_.fmx_max = p.phi_f_q_g_us; }
            stats_.ekf_us = p.ekf_predict_us;
            if (p.ekf_predict_us > stats_.ekf_max) { stats_.ekf_max = p.ekf_predict_us; }
            stats_.afb_us = p.acc_feedback_us;
            if (p.acc_feedback_us > stats_.afb_max) { stats_.afb_max = p.acc_feedback_us; }
            // [PROFILE] Phase-2: fmx 子段
            stats_.alc_us = p.fmx_alc_us;
            if (p.fmx_alc_us > stats_.alc_max) { stats_.alc_max = p.fmx_alc_us; }
            stats_.fill_us = p.fmx_fill_us;
            if (p.fmx_fill_us > stats_.fill_max) { stats_.fill_max = p.fmx_fill_us; }
            stats_.q1_us = p.fmx_q1_us;
            if (p.fmx_q1_us > stats_.q1_max) { stats_.q1_max = p.fmx_q1_us; }
            stats_.q2_us = p.fmx_q2_us;
            if (p.fmx_q2_us > stats_.q2_max) { stats_.q2_max = p.fmx_q2_us; }
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
            ++stats_.ins_run_disabled;
        }

        pending_ = false;

        RecordServiceRunElapsed(t_total_start);
        return 1;
    }
} // namespace aided_ins_service
