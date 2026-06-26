/**
 * @file    IST8310_Calibration.cpp
 * @brief   IST8310 磁力计 MCU 端校准算法实现
 */

#include "IST8310_Calibration.hpp"

#include <cfloat>
#include <cmath>

namespace ist8310_calibration
{

namespace
{

enum
{
    MIN_SAMPLES       = 200u,    // 最少有效 sample 数
};

constexpr float MIN_SPAN_UT        = 10.0F;   // 每轴最小 span，uT
constexpr float MIN_RADIUS_AVG_UT  = 15.0F;   // 最小平均半径，uT
constexpr float SCALE_MIN          = 0.5F;    // 允许的最小 scale
constexpr float SCALE_MAX          = 2.0F;    // 允许的最大 scale

// ── 内部状态 ──
bool     active_{false};
float    min_uT_[3]{0.0F, 0.0F, 0.0F};
float    max_uT_[3]{0.0F, 0.0F, 0.0F};
uint32_t sample_count_{0u};

// 磁场模长统计（仅追踪，不拒绝样本）
float    norm_min_{FLT_MAX};
float    norm_max_{-FLT_MAX};
uint32_t norm_out_of_range_count_{0u};

// 样本缓存：用于 Finish() 中计算校准后磁场模长统计（body-frame 未校准 uT）
static constexpr size_t MAX_CAL_SAMPLES = 2000u;
float cal_sample_buffer_[MAX_CAL_SAMPLES][3]{};
size_t cal_sample_count_{0u};
size_t cal_sample_dropped_{0u};

void InitMinMax()
{
    for (int i = 0; i < 3; ++i) {
        min_uT_[i] =  FLT_MAX;
        max_uT_[i] = -FLT_MAX;
    }
}

} // namespace

void Reset()
{
    InitMinMax();
    sample_count_ = 0u;
    norm_min_ = FLT_MAX;
    norm_max_ = -FLT_MAX;
    norm_out_of_range_count_ = 0u;
    cal_sample_count_ = 0u;
    cal_sample_dropped_ = 0u;
    active_ = true;
}

void FeedSample(const float mag_uT_body[3])
{
    if (!active_) { return; }

    // 磁场模长统计（仅追踪，不拒绝样本；后续可据此评估数据质量和异常情况）
    const float norm = std::sqrt(mag_uT_body[0] * mag_uT_body[0] +
                                  mag_uT_body[1] * mag_uT_body[1] +
                                  mag_uT_body[2] * mag_uT_body[2]);
    if (norm < norm_min_) { norm_min_ = norm; }
    if (norm > norm_max_) { norm_max_ = norm; }
    if (norm < 10.0F || norm > 100.0F) {
        ++norm_out_of_range_count_;
    }

    for (int i = 0; i < 3; ++i) {
        if (mag_uT_body[i] < min_uT_[i]) { min_uT_[i] = mag_uT_body[i]; }
        if (mag_uT_body[i] > max_uT_[i]) { max_uT_[i] = mag_uT_body[i]; }
    }

    ++sample_count_;

    // 保存未校准 body-frame 样本，供 Finish() 计算校准后模长统计
    if (cal_sample_count_ < MAX_CAL_SAMPLES) {
        cal_sample_buffer_[cal_sample_count_][0] = mag_uT_body[0];
        cal_sample_buffer_[cal_sample_count_][1] = mag_uT_body[1];
        cal_sample_buffer_[cal_sample_count_][2] = mag_uT_body[2];
        ++cal_sample_count_;
    } else {
        ++cal_sample_dropped_;
    }
}

MagCalResult Finish()
{
    active_ = false;

    MagCalResult r{};
    r.sample_count = sample_count_;

    if (sample_count_ == 0u) {
        r.valid = false;
        return r;
    }

    // 拷贝原始 min/max
    for (int i = 0; i < 3; ++i) {
        r.min_body_uT[i] = min_uT_[i];
        r.max_body_uT[i] = max_uT_[i];
    }

    // 计算 span / radius / bias
    float radius_sum = 0.0F;

    for (int i = 0; i < 3; ++i) {
        r.span_body_uT[i]   = r.max_body_uT[i] - r.min_body_uT[i];
        r.radius_body_uT[i] = r.span_body_uT[i] * 0.5F;
        r.bias_body_uT[i]   = (r.max_body_uT[i] + r.min_body_uT[i]) * 0.5F;
        radius_sum += r.radius_body_uT[i];
    }

    r.radius_avg_uT = radius_sum / 3.0F;

    // 计算三轴 scale
    if (r.radius_avg_uT > 0.0F) {
        for (int i = 0; i < 3; ++i) {
            if (r.radius_body_uT[i] > 0.0F) {
                r.scale_body[i] = r.radius_avg_uT / r.radius_body_uT[i];
            } else {
                r.scale_body[i] = 1.0F;   // 除零保护
            }
        }
    } else {
        for (int i = 0; i < 3; ++i) { r.scale_body[i] = 1.0F; }
    }

    // 质量门限
    bool pass = true;

    // 最少 sample 数
    if (sample_count_ < MIN_SAMPLES) { pass = false; }

    // 每轴最小 span
    for (int i = 0; i < 3; ++i) {
        if (r.span_body_uT[i] < MIN_SPAN_UT) { pass = false; }
    }

    // 最小平均半径
    if (r.radius_avg_uT < MIN_RADIUS_AVG_UT) { pass = false; }

    // scale 范围
    for (int i = 0; i < 3; ++i) {
        if (r.scale_body[i] < SCALE_MIN || r.scale_body[i] > SCALE_MAX) { pass = false; }
    }

    // 磁场模长辅助统计（raw body-frame norm 仅作观察，不等价于椭球拟合残差或校准后球面误差）
    r.norm_min_uT = norm_min_;
    r.norm_max_uT = norm_max_;
    if (norm_max_ > 0.0F && norm_min_ < FLT_MAX) {
        const float norm_mean = (norm_max_ + norm_min_) * 0.5F;
        if (norm_mean > 0.0F) {
            r.norm_range_ratio = (norm_max_ - norm_min_) / norm_mean;
        }
    }
    r.norm_out_of_range_count = norm_out_of_range_count_;

    // quality score: min(span) / max(span)
    float span_min = r.span_body_uT[0];
    float span_max = r.span_body_uT[0];

    for (int i = 1; i < 3; ++i) {
        if (r.span_body_uT[i] < span_min) { span_min = r.span_body_uT[i]; }
        if (r.span_body_uT[i] > span_max) { span_max = r.span_body_uT[i]; }
    }

    if (span_max > 0.0F) {
        r.quality_score = span_min / span_max;
    }

    // ── 校准后磁场模长自验证：基于本轮 bias/scale 回放所有保存的样本 ──
    if (cal_sample_count_ > 0u) {
        float  cal_norm_min = FLT_MAX;
        float  cal_norm_max = -FLT_MAX;
        double cal_norm_sum = 0.0;
        double cal_norm_sum_sq = 0.0;

        for (size_t i = 0u; i < cal_sample_count_; ++i) {
            const float cx = (cal_sample_buffer_[i][0] - r.bias_body_uT[0]) * r.scale_body[0];
            const float cy = (cal_sample_buffer_[i][1] - r.bias_body_uT[1]) * r.scale_body[1];
            const float cz = (cal_sample_buffer_[i][2] - r.bias_body_uT[2]) * r.scale_body[2];
            const float cn = std::sqrt(cx * cx + cy * cy + cz * cz);

            if (cn < cal_norm_min) { cal_norm_min = cn; }
            if (cn > cal_norm_max) { cal_norm_max = cn; }
            cal_norm_sum    += static_cast<double>(cn);
            cal_norm_sum_sq += static_cast<double>(cn) * static_cast<double>(cn);
        }

        r.cal_norm_min_uT  = cal_norm_min;
        r.cal_norm_max_uT  = cal_norm_max;
        r.cal_norm_mean_uT = static_cast<float>(cal_norm_sum / static_cast<double>(cal_sample_count_));

        const double mean_d = static_cast<double>(r.cal_norm_mean_uT);
        const double var   = (cal_norm_sum_sq / static_cast<double>(cal_sample_count_)) - mean_d * mean_d;
        r.cal_norm_std_uT  = (var > 0.0) ? static_cast<float>(std::sqrt(var)) : 0.0F;

        if (r.cal_norm_mean_uT > 0.0F) {
            r.cal_norm_range_ratio = (cal_norm_max - cal_norm_min) / r.cal_norm_mean_uT;
            const float max_dev = (cal_norm_max - r.cal_norm_mean_uT) > (r.cal_norm_mean_uT - cal_norm_min)
                                ? (cal_norm_max - r.cal_norm_mean_uT)
                                : (r.cal_norm_mean_uT - cal_norm_min);
            r.cal_norm_max_error_ratio = max_dev / r.cal_norm_mean_uT;
        }

        r.cal_norm_valid          = true;
        r.cal_sample_count        = static_cast<uint32_t>(cal_sample_count_);
        r.cal_sample_dropped_count = static_cast<uint32_t>(cal_sample_dropped_);
    }

    r.valid = pass;
    return r;
}

uint32_t GetSampleCount()
{
    return sample_count_;
}

} // namespace ist8310_calibration
