/**
 * @file    AidedInsService.hpp
 * @brief   Aided INS Service 最小骨架接口
 *
 * @details
 * 本 Service 负责连接 ICM42688P driver 和 Aided_INS 算法类：
 *   1. 从 icm42688_service 读取最新 Sample；
 *   2. 将 Sample 转换为 Aided_INS_Space::IMU；
 *   3. 每两个连续 400 Hz IMU delta 聚合成一个 200 Hz IMU delta；
 *   4. 聚合完成后注入 Aided_INS 并调用 Run()。
 *
 * 当前 Service 已由 IMU DRDY 路径调用；本文件仅提供初始化、运行和统计读取接口。
 */

#pragma once

#include <cstdint>

namespace aided_ins_service
{
    /**
     * @brief  运行统计（用于 debugger 或低频 debug task 观察 IMU→INS 数据链路）。
     */
    struct Stats
    {
        uint32_t run_calls{0};              // Run() 被调用次数
        uint32_t get_latest_failures{0};     // GetLatest 非 Ok 次数
        uint32_t invalid_samples{0};         // data_valid/dt/delta_samples 异常次数
        uint32_t duplicate_samples{0};       // timestamp 重复次数
        uint32_t timestamp_errors{0};        // 逆序或间隔异常次数
        uint32_t valid_samples{0};           // 有效 sample 次数
        uint32_t aggregated_samples{0};      // 完成 400 Hz×2→200 Hz 聚合次数
        uint32_t ins_run_calls{0};           // Aided_INS::Run() 调用次数
        uint32_t ins_run_last_us{0};         // Aided_INS::Run() 最近一次执行时间，us
        uint32_t ins_run_max_us{0};          // Aided_INS::Run() 单次最大执行时间，us
        uint32_t service_run_last_us{0};     // Run() 最近一次总耗时，us
        uint32_t service_run_max_us{0};      // Run() 单次最大总耗时，us
        uint32_t queued_imu_samples{0};      // 完成 200 Hz 聚合并缓存次数
        uint32_t dropped_imu_samples{0};     // 缓存覆盖次数（消费者未及时取走）
        uint32_t ins_run_disabled{0};        // 聚合完成但跳过 INS Run 的次数

        // [PROFILE] 临时分段耗时统计
        uint32_t pnd_us{0};      // ProcessNewData last
        uint32_t pnd_max{0};     // ProcessNewData max
        uint32_t prp_us{0};      // InsPropagation last
        uint32_t prp_max{0};     // InsPropagation max
        uint32_t mec_us{0};      // INS_Mech last
        uint32_t mec_max{0};     // INS_Mech max
        uint32_t fmx_us{0};      // Phi/F/Q/G last
        uint32_t fmx_max{0};     // Phi/F/Q/G max
        uint32_t ekf_us{0};      // EKFPredict last
        uint32_t ekf_max{0};     // EKFPredict max
        uint32_t afb_us{0};      // AccUpdate+StateFeedback last
        uint32_t afb_max{0};     // AccUpdate+StateFeedback max
        // [PROFILE] Phase-2: fmx 子段统计
        uint32_t alc_us{0};      // Matrix create+resize+init last
        uint32_t alc_max{0};     // Matrix create+resize+init max
        uint32_t fill_us{0};     // 地理参数+F/G 填充+Phi 更新 last
        uint32_t fill_max{0};    // 地理参数+F/G 填充+Phi 更新 max
        uint32_t q1_us{0};       // 第一次 Q = Phi*G*q_*G'*Phi' last
        uint32_t q1_max{0};      // 第一次 Q max
        uint32_t q2_us{0};       // G 更新 + 第二次 Q last
        uint32_t q2_max{0};      // G 更新 + 第二次 Q max
        // [ACC_DBG] 临时 AccUpdate 诊断
        uint32_t acc_try{0};
        uint32_t acc_accept{0};
        uint32_t acc_fail_small{0};
        uint32_t acc_fail_norm{0};
        uint32_t acc_fail_cos{0};
        uint32_t acc_feedback{0};
        float    f_norm{0.0f};
        float    f_gravity{0.0f};
        float    f_norm_diff{0.0f};
        float    f_cos_gn_gb{0.0f};
        float    f_f_b[3]{0.0f, 0.0f, 0.0f};
        float    f_g_b_ByImu[3]{0.0f, 0.0f, 0.0f};
        // [EKF_DBG] EKFPredict 分段计时
        uint32_t ekf_dx_us{0};
        uint32_t ekf_phi_p_us{0};
        uint32_t ekf_m_phi_t_q_us{0};
        // [PROFILE] afb 拆分
        uint32_t acc_prep_us{0};
        uint32_t acc_prep_max{0};
        uint32_t acc_ekf_us{0};
        uint32_t acc_ekf_max{0};
        uint32_t feedback_us{0};
        uint32_t feedback_max{0};
        // [PROFILE] EkfUpdateAcc3 内部分段
        uint32_t acc_phs_us{0};
        uint32_t acc_phs_max{0};
        uint32_t acc_kdx_us{0};
        uint32_t acc_kdx_max{0};
        uint32_t acc_p_khp_us{0};
        uint32_t acc_p_khp_max{0};
        uint32_t acc_p_phkt_us{0};
        uint32_t acc_p_phkt_max{0};
        uint32_t acc_p_ksk_us{0};
        uint32_t acc_p_ksk_max{0};
        // [ATT_DBG] 姿态/重力/比力一致性诊断
        float euler_r{0.0f};
        float euler_p{0.0f};
        float euler_y{0.0f};
        float g_l_n[3]{0.0f, 0.0f, 0.0f};
        float g_b[3]{0.0f, 0.0f, 0.0f};
        float cbn_f[3]{0.0f, 0.0f, 0.0f};
        float cbn_f_g[3]{0.0f, 0.0f, 0.0f};
        float cfn_g_norm{0.0f};
        float cos_fg{0.0f};
        // [COV_HLTH] 协方差健康状态镜像（always present，macro off 时保持零值）
        bool     cov_has_nan_inf{false};
        bool     cov_has_neg_diag{false};
        float    cov_max_asymmetry_last{0.0f};
        float    cov_max_asymmetry_max{0.0f};
        uint32_t cov_nan_inf_count{0};
        uint32_t cov_neg_diag_count{0};
        uint32_t cov_check_count{0};
    };

    /**
     * @brief  初始化 Service（创建 Aided_INS 实例并调用 Init()）。
     * @retval 0 成功，-1 失败。
     */
    int Init();

    /**
     * @brief  消费一次最新的 ICM42688P Sample 并推进聚合逻辑。
     * @note   每次 ICM42688P driver 更新 latest 后调用本函数。
     *         内部每两个 400 Hz 有效 sample 聚合成一个 200 Hz INS 输入。
     */
    int Run();

    /**
     * @brief  消费最新聚合完成的 200 Hz IMU 并执行一次 INS 更新。
     * @note   独立于 producer 的 consumer 路径；不访问 ICM42688P 驱动。
     * @retval 0 无可用数据，1 成功执行了 Run()。
     */
    int RunInsConsumerOnce();

    /**
     * @brief  返回运行统计快照（只读，如需清零则调用 Init()）。
     */
    Stats GetStats();

    /**
     * @brief  姿态遥测数据快照（供低优先级 telemetry task 使用）。
     * @details 坐标系为 INS 内部 NED/FRD 约定，未做上位机映射。
     */
    struct AttitudeTelemetry
    {
        double timestamp{0.0};
        float  roll_deg{0.0f};
        float  pitch_deg{0.0f};
        float  yaw_deg{0.0f};
        float  qw{0.0f};
        float  qx{0.0f};
        float  qy{0.0f};
        float  qz{0.0f};
    };

    AttitudeTelemetry GetAttitudeTelemetry();

    /** @brief INS 是否已进入 Running 状态（供 telemetry task 判断是否可输出） */
    bool IsInsRunning();
} // namespace aided_ins_service
