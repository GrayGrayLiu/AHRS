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
     * @brief  返回运行统计快照（只读，如需清零则调用 Init()）。
     */
    Stats GetStats();
} // namespace aided_ins_service
