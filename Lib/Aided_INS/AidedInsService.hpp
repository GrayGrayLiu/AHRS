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
 * 当前为最小骨架，尚未接入 scheduler。后续在 imu_drdy 路径中接入。
 */

#pragma once

namespace aided_ins_service
{
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
} // namespace aided_ins_service
