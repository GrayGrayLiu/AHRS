/**
 * @file    IST8310_Calibration.hpp
 * @brief   IST8310 磁力计 MCU 端校准算法模块
 *
 * @details
 * 本模块只负责校准算法：接收 body-frame uT sample、跟踪 min/max、
 * 计算 hard-iron bias 和三轴 scale。
 *
 * 不负责：按键、printf、LED、scheduler、Flash 存储。
 */

#pragma once

#include <cstdint>

namespace ist8310_calibration
{

struct MagCalResult
{
    float bias_body_uT[3]{0.0F, 0.0F, 0.0F};    // hard-iron 偏置 (body-frame uT)
    float scale_body[3]{1.0F, 1.0F, 1.0F};      // 三轴 scale (理想 ≈1.0)

    float min_body_uT[3]{0.0F, 0.0F, 0.0F};     // 收集期间每轴最小值
    float max_body_uT[3]{0.0F, 0.0F, 0.0F};     // 收集期间每轴最大值
    float span_body_uT[3]{0.0F, 0.0F, 0.0F};    // max - min
    float radius_body_uT[3]{0.0F, 0.0F, 0.0F};  // span / 2
    float radius_avg_uT{0.0F};                   // 三轴平均半径

    float    quality_score{0.0F};                // 0~1，越高越好

    // raw body-frame 磁场模长辅助统计（仅用于观察采样覆盖和异常情况，
    // 不等价于椭球拟合残差，也不等价于校准后球面误差）
    float    norm_min_uT{0.0F};                   // 收集期间磁场模长最小值
    float    norm_max_uT{0.0F};                   // 收集期间磁场模长最大值
    float    norm_range_ratio{0.0F};              // 模长变化率 = (max-min) / mean
    uint32_t norm_out_of_range_count{0u};         // 模长超出 [10,100] uT 的样本数（仅统计，不拒绝）

    // 校准后磁场模长自验证统计（应用本轮 bias/scale 回放样本）
    // 与 raw norm 统计互补：raw norm 反映采样分布，cal norm 反映校准后球面一致性
    float    cal_norm_min_uT{0.0F};
    float    cal_norm_max_uT{0.0F};
    float    cal_norm_mean_uT{0.0F};
    float    cal_norm_std_uT{0.0F};
    float    cal_norm_range_ratio{0.0F};
    float    cal_norm_max_error_ratio{0.0F};
    bool     cal_norm_valid{false};
    uint32_t cal_sample_count{0u};
    uint32_t cal_sample_dropped_count{0u};

    uint32_t sample_count{0u};                   // 有效 sample 数
    bool     valid{false};                       // 结果是否通过质量门限
};

/**
 * @brief 重置所有内部状态（重新开始收集）
 */
void Reset();

/**
 * @brief 输入一个 body-frame uT sample，更新 min/max
 * @param mag_uT_body[3]  body-frame 三轴微特斯拉值
 */
void FeedSample(const float mag_uT_body[3]);

/**
 * @brief 结束收集，计算 bias/scale 并返回结果
 */
MagCalResult Finish();

/**
 * @brief 返回已收集的 sample 数
 */
uint32_t GetSampleCount();

} // namespace ist8310_calibration
