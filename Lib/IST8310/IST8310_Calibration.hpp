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

#include <cstddef>
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
 * @brief fixed-bias 3×3 quadratic fit 结果
 *
 * @note  不是完整 coupled ellipsoid fit。bias 由 min/max 预估计，
 *        仅对 centered data 拟合 3×3 对称二次型 Q，再通过 SPD sqrt 构造 M。
 */
struct EllipFitResult
{
    float bias_body_uT[3]{};
    float matrix_3x3[3][3]{};       // M, mag_cal = M * (x - bias)

    float cal_norm_min_uT{};
    float cal_norm_max_uT{};
    float cal_norm_mean_uT{};
    float cal_norm_std_uT{};
    float cal_norm_max_err{};

    float condition_number{};       // λ_max / λ_min（过大说明数据接近共面）
    bool  Q_positive_definite{false};
    bool  valid{false};             // 拟合是否通过质量门限
};

/**
 * @brief 固定偏置 3×3 quadratic fit（应在 Finish() 后、同一批样本上调用）
 *
 * @param sample_buffer  样本数组 [count][3] body-frame 未校准 uT
 * @param count          样本数
 * @param min_uT[3]      每轴最小值（用于 bias 初值）
 * @param max_uT[3]      每轴最大值（用于 bias 初值）
 * @return 拟合结果
 */
EllipFitResult FitEllipsoidFixedBias(
    const float sample_buffer[][3],
    size_t count,
    const float min_uT[3],
    const float max_uT[3]);

/**
 * @brief 10-parameter algebraic full coupled ellipsoid candidate 结果
 *
 * @note  不是几何距离最优，不是最终方案。齐次最小特征向量解仅保证代数最优，
 *        输出前需校验 Q positive definite + condition number + cal_norm。
 */
struct FullEllipFitResult
{
    float bias_body_uT[3]{};
    float matrix_3x3[3][3]{};       // M, mag_cal = M * (x - bias)

    float cal_norm_min_uT{};
    float cal_norm_max_uT{};
    float cal_norm_mean_uT{};
    float cal_norm_std_uT{};
    float cal_norm_max_err{};

    float condition_number{};
    float k_raw{};                  // b^T A b - c（归一化空间 scale factor）
    float R0_uT{};                  // 归一化半径
    float R_target_uT{};            // 校正目标半径

    uint8_t solver_iters{};
    bool solver_converged{false};
    bool Q_positive_definite{false};
    bool valid{false};
};

/**
 * @brief 10-parameter algebraic full coupled ellipsoid candidate fit
 *
 * @note  使用归一化 + 齐次最小特征向量法估计 b 和 Q。
 *        不保证 Q positive definite，调用方须检查 FullEllipFitResult.valid。
 *        只作为候选参数块，不替换当前 diagonal 基线。
 *
 * @param sample_buffer  样本数组 [count][3] body-frame 未校准 uT
 * @param count          样本数
 * @param min_uT[3]      每轴最小值
 * @param max_uT[3]      每轴最大值
 * @return 拟合结果
 */
FullEllipFitResult FitEllipsoidFullAlgebraic(
    const float sample_buffer[][3],
    size_t count,
    const float min_uT[3],
    const float max_uT[3]);

/**
 * @brief PX4-style sphere LM fit 结果（初始化阶段，只优化 radius + offset）
 *
 * @note  对应 PX4 lm_mag_fit(..., full_ellipsoid=false)。
 *        本阶段 diag={1,1,1}, offdiag={0,0,0}, 只优化 radius + offset[3]。
 *        不输出 scale/offdiag, full ellipsoid LM 不在本轮范围。
 *        AHRS 使用 uT，PX4 使用 Gauss（1 G = 100 uT）。
 */
struct SphereLmFitResult
{
    float offset_body_uT[3]{};
    float radius_uT{};
    float cost_px4_uT{};    // sqrt(sum_sq) / N (PX4-style normalized cost, used for convergence), in uT
    float rms_uT{};         // sqrt(sum_sq / N) (true RMS residual, diagnostic only), in uT
    uint8_t iterations{};
    bool solver_converged{false};
    bool valid{false};
};

/**
 * @brief PX4-style sphere LM fit（只优化 radius + offset）
 *
 * @param sample_buffer  样本数组 [count][3] body-frame 未校准 uT
 * @param count          样本数
 * @return 拟合结果
 */
SphereLmFitResult FitSphereLm(
    const float sample_buffer[][3],
    size_t count);

/**
 * @brief PX4-style full ellipsoid LM refinement 结果
 *
 * @note  对应 PX4 lm_mag_fit(..., full_ellipsoid=true)。
 *        A1 sphere LM 提供 radius + offset 初值。
 *        本阶段固定 radius，优化 offset[3] + diag[3] + offdiag[3]。
 *        diag/offdiag 直接组成 runtime correction matrix M，不是 ellipsoid Q 的中间量。
 *        AHRS 使用 uT，PX4 使用 Gauss。
 */
struct EllipsoidLmFitResult
{
    float offset_body_uT[3]{};
    float diag[3]{1.0F, 1.0F, 1.0F};
    float offdiag[3]{};
    float radius_uT{};
    float cost_px4_uT{};
    float rms_uT{};
    uint8_t iterations{};
    bool solver_converged{false};
    bool valid{false};

    float cal_norm_min_uT{};
    float cal_norm_max_uT{};
    float cal_norm_mean_uT{};
    float cal_norm_std_uT{};
    float cal_norm_max_err{};

    // ── 诊断字段（A2 self-validation 辅助） ──
    float cal_norm_p95_err{};       // 相对误差 95th 百分位: abs(norm - mean) / mean
    float cal_norm_p99_err{};       // 相对误差 99th 百分位
    uint32_t max_err_sample_index{};
    float max_err_raw_body_uT[3]{};
    float max_err_cal_body_uT[3]{};
    float max_err_cal_norm_uT{};
    float max_err_abs_uT{};         // abs(cal_norm - cal_norm_mean)
};

/**
 * @brief PX4-style full ellipsoid LM refinement（固定 radius，优化 offset + diag + offdiag）
 *
 * @param sample_buffer  样本数组 [count][3] body-frame 未校准 uT
 * @param count          样本数
 * @param sphere         A1 FitSphereLm 结果（作为初值）
 * @return 拟合结果
 */
EllipsoidLmFitResult FitEllipsoidLm(
    const float sample_buffer[][3],
    size_t count,
    const SphereLmFitResult &sphere);

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

/**
 * @brief 返回内部样本缓存指针；样本有效数量由 GetSampleBufferCount() 判断
 */
const float (*GetSampleBuffer())[3];
size_t GetSampleBufferCount();

} // namespace ist8310_calibration
