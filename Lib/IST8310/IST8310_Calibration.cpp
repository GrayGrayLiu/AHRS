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

// ── 椭球拟合质量门限（B1 / B2a 共用）──
constexpr float ELLIP_FIT_MAX_STD_UT    = 5.0F;
constexpr float ELLIP_FIT_MIN_MEAN_UT   = 30.0F;
constexpr float ELLIP_FIT_MAX_MEAN_UT   = 60.0F;
constexpr float ELLIP_FIT_MIN_NORM_UT   = 30.0F;
constexpr float ELLIP_FIT_MAX_NORM_UT   = 60.0F;
constexpr float ELLIP_FIT_MAX_ERR_RATIO = 0.30F;
constexpr float ELLIP_FIT_MAX_COND      = 500.0F;

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

// ============================================================================
// Fixed-bias 3×3 quadratic fit 线性代数 helper
// ============================================================================

// 6×6 对称正定线性系统求解（Cholesky）
// 返回 true 表示成功，false 表示矩阵不正定
bool Solve6x6SPD(const double A[6][6], const double b[6], double x[6])
{
    double L[6][6] = {};

    // Cholesky: A = L * L^T
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j <= i; ++j) {
            double sum = A[i][j];
            for (int k = 0; k < j; ++k) { sum -= L[i][k] * L[j][k]; }
            if (i == j) {
                if (sum <= 0.0) { return false; }  // 非正定，拟合失败
                L[i][j] = std::sqrt(sum);
            } else {
                L[i][j] = sum / L[j][j];
            }
        }
    }

    // Forward: L * y = b
    double y[6];
    for (int i = 0; i < 6; ++i) {
        double sum = b[i];
        for (int j = 0; j < i; ++j) { sum -= L[i][j] * y[j]; }
        y[i] = sum / L[i][i];
    }

    // Backward: L^T * x = y
    for (int i = 5; i >= 0; --i) {
        double sum = y[i];
        for (int j = i + 1; j < 6; ++j) { sum -= L[j][i] * x[j]; }
        x[i] = sum / L[i][i];
    }

    return true;
}

// 3×3 对称矩阵 Jacobi 特征值分解
// A_in: 输入对称 3×3（不修改）
// V:    输出特征向量矩阵（列向量），V * diag(lambda) * V^T = A_in
// lambda: 输出特征值
// 返回 true 若收敛，false 若固定 sweep 数后未收敛
bool Jacobi3x3(const double A_in[3][3], double V[3][3], double lambda[3])
{
    double A[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) { A[i][j] = A_in[i][j]; }
    }

    // V 初始化为单位阵
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) { V[i][j] = 0.0; }
        V[i][i] = 1.0;
    }

    constexpr int    MAX_SWEEPS = 15;
    constexpr double EPS        = 1.0e-14;
    bool converged = false;

    for (int sweep = 0; sweep < MAX_SWEEPS; ++sweep) {
        double max_off = 0.0;
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                const double apq = std::abs(A[p][q]);
                if (apq > max_off) { max_off = apq; }
                if (apq < EPS) { continue; }

                const double theta = 0.5 * std::atan2(2.0 * A[p][q], A[p][p] - A[q][q]);
                const double c = std::cos(theta);
                const double s = std::sin(theta);
                const double cc = c * c;
                const double ss = s * s;
                const double cs = c * s;

                const double app = A[p][p];
                const double aqq = A[q][q];

                // A = J^T * A * J
                A[p][p] = cc * app + 2.0 * cs * A[p][q] + ss * aqq;
                A[q][q] = ss * app - 2.0 * cs * A[p][q] + cc * aqq;
                A[p][q] = 0.0;
                A[q][p] = 0.0;

                for (int r = 0; r < 3; ++r) {
                    if (r == p || r == q) { continue; }
                    const double arp = A[r][p];
                    const double arq = A[r][q];
                    A[r][p] = A[p][r] =  c * arp + s * arq;
                    A[r][q] = A[q][r] = -s * arp + c * arq;
                }

                // V = V * J
                for (int r = 0; r < 3; ++r) {
                    const double vrp = V[r][p];
                    const double vrq = V[r][q];
                    V[r][p] =  c * vrp + s * vrq;
                    V[r][q] = -s * vrp + c * vrq;
                }
            }
        }
        if (max_off < EPS) { converged = true; break; }
    }

    lambda[0] = A[0][0];
    lambda[1] = A[1][1];
    lambda[2] = A[2][2];

    return converged;
}

// Swap eigenvalue lambda[a]/lambda[b] AND corresponding eigenvector columns
void SwapEigenPair(double V[3][3], double lambda[3], const int a, const int b)
{
    const double t_lambda = lambda[a];
    lambda[a] = lambda[b];
    lambda[b] = t_lambda;

    for (int r = 0; r < 3; ++r) {
        const double t_v = V[r][a];
        V[r][a] = V[r][b];
        V[r][b] = t_v;
    }
}

// 3×3 SPD 主平方根: Q_sqrt = V * sqrt(Λ) * V^T
void SqrtSPD3x3(const double V[3][3], const double lambda[3], double Q_sqrt[3][3])
{
    // sqrt_Λ = diag(sqrt(lambda))
    // temp[i][k] = V[i][k] * sqrt(lambda[k])
    // Q_sqrt[i][j] = Σ_k temp[i][k] * V[j][k]
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += V[i][k] * std::sqrt(lambda[k]) * V[j][k];
            }
            Q_sqrt[i][j] = sum;
        }
    }
}

// ============================================================================
// Full coupled 10-parameter algebraic fit helpers
// ============================================================================

// 10×10 symmetric positive definite linear system solve (Cholesky)
bool Solve10x10SPD(const double A[10][10], const double b[10], double x[10])
{
    double L[10][10] = {};

    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j <= i; ++j) {
            double sum = A[i][j];
            for (int k = 0; k < j; ++k) { sum -= L[i][k] * L[j][k]; }
            if (i == j) {
                if (sum <= 0.0) { return false; }
                L[i][j] = std::sqrt(sum);
            } else {
                L[i][j] = sum / L[j][j];
            }
        }
    }

    double y[10];
    for (int i = 0; i < 10; ++i) {
        double sum = b[i];
        for (int j = 0; j < i; ++j) { sum -= L[i][j] * y[j]; }
        y[i] = sum / L[i][i];
    }

    for (int i = 9; i >= 0; --i) {
        double sum = y[i];
        for (int j = i + 1; j < 10; ++j) { sum -= L[j][i] * x[j]; }
        x[i] = sum / L[i][i];
    }

    return true;
}

// 3×3 symmetric matrix inverse (cofactor method)
// A = [[a,b,c],[b,d,e],[c,e,f]]  →  A^{-1}
// Returns false if singular (|det| < eps)
bool Inverse3x3(const double A[3][3], double A_inv[3][3])
{
    const double a = A[0][0], b = A[0][1], c = A[0][2];
    const double d = A[1][1], e = A[1][2], f = A[2][2];

    // Cofactors of symmetric A
    const double C00 = d * f - e * e;
    const double C11 = a * f - c * c;
    const double C22 = a * d - b * b;
    const double C01 = c * e - b * f;
    const double C02 = b * e - c * d;
    const double C12 = b * c - a * e;

    const double det = a * C00 + b * C01 + c * C02;
    if (std::abs(det) < 1.0e-15) { return false; }

    const double inv_det = 1.0 / det;
    A_inv[0][0] = C00 * inv_det;
    A_inv[0][1] = C01 * inv_det;
    A_inv[0][2] = C02 * inv_det;
    A_inv[1][0] = A_inv[0][1];
    A_inv[1][1] = C11 * inv_det;
    A_inv[1][2] = C12 * inv_det;
    A_inv[2][0] = A_inv[0][2];
    A_inv[2][1] = A_inv[1][2];
    A_inv[2][2] = C22 * inv_det;

    return true;
}

// Inverse power iteration: find eigenvector of S corresponding to smallest eigenvalue
// S:     10×10 symmetric positive semidefinite (modified in-place with regularization)
// theta: output parameter vector (eigenvector)
// iters: output iteration count
// converged: output convergence flag
// Returns false if Cholesky fails during iteration
bool InversePowerSmallestEigen10x10(
    const double S_in[10][10], double theta[10],
    uint8_t &iters, bool &converged)
{
    // Regularize
    double trace = 0.0;
    for (int i = 0; i < 10; ++i) { trace += S_in[i][i]; }
    const double reg = 1.0e-8 * trace / 10.0;

    double S_reg[10][10];
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 10; ++j) { S_reg[i][j] = S_in[i][j]; }
        S_reg[i][i] += reg;
    }

    double v[10];
    for (int i = 0; i < 10; ++i) { v[i] = 1.0; }

    constexpr int    MAX_ITERS = 15;
    constexpr double TOL       = 1.0e-8;

    converged = false;
    iters = 0;

    for (int iter = 0; iter < MAX_ITERS; ++iter) {
        double w[10];
        if (!Solve10x10SPD(S_reg, v, w)) { return false; }

        // Normalize (guard against zero or degenerate w)
        double norm_w = 0.0;
        for (int i = 0; i < 10; ++i) { norm_w += w[i] * w[i]; }
        if (norm_w <= 0.0) { return false; }
        norm_w = std::sqrt(norm_w);
        for (int i = 0; i < 10; ++i) { w[i] /= norm_w; }

        // Check eigenvector change (handle sign ambiguity: v and -v are same direction)
        double diff_same = 0.0;
        double diff_flip = 0.0;
        for (int i = 0; i < 10; ++i) {
            const double d_same = w[i] - v[i];
            const double d_flip = w[i] + v[i];
            diff_same += d_same * d_same;
            diff_flip += d_flip * d_flip;
        }
        const double diff = (diff_same < diff_flip) ? std::sqrt(diff_same) : std::sqrt(diff_flip);

        for (int i = 0; i < 10; ++i) { v[i] = w[i]; }

        iters = static_cast<uint8_t>(iter + 1);
        if (diff < TOL) { converged = true; break; }
    }

    for (int i = 0; i < 10; ++i) { theta[i] = v[i]; }
    return true;
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

const float (*GetSampleBuffer())[3]
{
    return cal_sample_buffer_;
}

size_t GetSampleBufferCount()
{
    return cal_sample_count_;
}

EllipFitResult FitEllipsoidFixedBias(
    const float sample_buffer[][3],
    const size_t count,
    const float min_uT[3],
    const float max_uT[3])
{
    EllipFitResult r{};

    if (count < 50u) { return r; }  // 样本不足，不拟合

    // ── Stage 1: fixed bias from min/max ──
    for (int i = 0; i < 3; ++i) {
        r.bias_body_uT[i] = (max_uT[i] + min_uT[i]) * 0.5F;
    }

    // ── Stage 2: accumulate 6×6 normal equations for centered Q ──
    double A[6][6] = {};
    double rhs[6]  = {};

    for (size_t k = 0u; k < count; ++k) {
        const double y0 = static_cast<double>(sample_buffer[k][0]) - static_cast<double>(r.bias_body_uT[0]);
        const double y1 = static_cast<double>(sample_buffer[k][1]) - static_cast<double>(r.bias_body_uT[1]);
        const double y2 = static_cast<double>(sample_buffer[k][2]) - static_cast<double>(r.bias_body_uT[2]);

        const double f[6] = {
            y0 * y0,
            y1 * y1,
            y2 * y2,
            2.0 * y0 * y1,
            2.0 * y0 * y2,
            2.0 * y1 * y2,
        };

        for (int i = 0; i < 6; ++i) {
            rhs[i] += f[i];
            for (int j = 0; j < 6; ++j) {
                A[i][j] += f[i] * f[j];
            }
        }
    }

    // ── Stage 3: solve A * θ = rhs ──
    double theta[6];
    if (!Solve6x6SPD(A, rhs, theta)) {
        return r;  // 矩阵不正定，拟合失败
    }

    // ── Stage 4: construct symmetric Q ──
    double Q[3][3] = {
        { theta[0], theta[3], theta[4] },
        { theta[3], theta[1], theta[5] },
        { theta[4], theta[5], theta[2] },
    };

    // ── Stage 5: Jacobi eigen decomposition Q = V Λ V^T ──
    double V[3][3];
    double lambda[3];
    if (!Jacobi3x3(Q, V, lambda)) {
        return r;  // Jacobi 不收敛，拟合失败
    }

    // ── Stage 6: positive definiteness + condition number ──
    // 按降序排列 (eigenvalue, eigenvector) pair（Jacobi 不保证排序）
    if (lambda[0] < lambda[1]) { SwapEigenPair(V, lambda, 0, 1); }
    if (lambda[0] < lambda[2]) { SwapEigenPair(V, lambda, 0, 2); }
    if (lambda[1] < lambda[2]) { SwapEigenPair(V, lambda, 1, 2); }

    const double lambda_min = lambda[2];
    const double lambda_max = lambda[0];

    r.Q_positive_definite = (lambda_min > 0.0);
    if (!r.Q_positive_definite) { return r; }

    r.condition_number = static_cast<float>(lambda_max / lambda_min);

    // ── Stage 7: SPD square root Q_sqrt = V sqrt(Λ) V^T ──
    double Q_sqrt[3][3];
    SqrtSPD3x3(V, lambda, Q_sqrt);

    // ── Stage 8: target radius R = mean(||y_k||) ──
    double R = 0.0;
    for (size_t k = 0u; k < count; ++k) {
        const double y0 = static_cast<double>(sample_buffer[k][0]) - static_cast<double>(r.bias_body_uT[0]);
        const double y1 = static_cast<double>(sample_buffer[k][1]) - static_cast<double>(r.bias_body_uT[1]);
        const double y2 = static_cast<double>(sample_buffer[k][2]) - static_cast<double>(r.bias_body_uT[2]);
        R += std::sqrt(y0 * y0 + y1 * y1 + y2 * y2);
    }
    R /= static_cast<double>(count);

    // ── Stage 9: correction matrix M = R * Q_sqrt ──
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            r.matrix_3x3[i][j] = static_cast<float>(R * Q_sqrt[i][j]);
        }
    }

    // ── Stage 10: self-validation ──
    float  cal_min  = FLT_MAX;
    float  cal_max  = -FLT_MAX;
    double cal_sum  = 0.0;
    double cal_sum2 = 0.0;

    for (size_t k = 0u; k < count; ++k) {
        const float  y[3] = {
            sample_buffer[k][0] - r.bias_body_uT[0],
            sample_buffer[k][1] - r.bias_body_uT[1],
            sample_buffer[k][2] - r.bias_body_uT[2],
        };
        float cal[3] = {};
        for (int i = 0; i < 3; ++i) {
            cal[i] = r.matrix_3x3[i][0] * y[0]
                   + r.matrix_3x3[i][1] * y[1]
                   + r.matrix_3x3[i][2] * y[2];
        }
        const float cn = std::sqrt(cal[0] * cal[0] + cal[1] * cal[1] + cal[2] * cal[2]);

        if (cn < cal_min) { cal_min = cn; }
        if (cn > cal_max) { cal_max = cn; }
        cal_sum  += static_cast<double>(cn);
        cal_sum2 += static_cast<double>(cn) * static_cast<double>(cn);
    }

    r.cal_norm_min_uT  = cal_min;
    r.cal_norm_max_uT  = cal_max;
    r.cal_norm_mean_uT = static_cast<float>(cal_sum / static_cast<double>(count));
    {
        const double m  = static_cast<double>(r.cal_norm_mean_uT);
        const double var = cal_sum2 / static_cast<double>(count) - m * m;
        r.cal_norm_std_uT = (var > 0.0) ? static_cast<float>(std::sqrt(var)) : 0.0F;
    }
    if (r.cal_norm_mean_uT > 0.0F) {
        const float max_dev = (cal_max - r.cal_norm_mean_uT) > (r.cal_norm_mean_uT - cal_min)
                            ? (cal_max - r.cal_norm_mean_uT)
                            : (r.cal_norm_mean_uT - cal_min);
        r.cal_norm_max_err = max_dev / r.cal_norm_mean_uT;
    }

    // ── Stage 11: acceptance ──
    r.valid = r.Q_positive_definite
           && r.condition_number < ELLIP_FIT_MAX_COND
           && r.cal_norm_std_uT  < ELLIP_FIT_MAX_STD_UT
           && r.cal_norm_mean_uT > ELLIP_FIT_MIN_MEAN_UT
           && r.cal_norm_mean_uT < ELLIP_FIT_MAX_MEAN_UT
           && r.cal_norm_min_uT  > ELLIP_FIT_MIN_NORM_UT
           && r.cal_norm_max_uT  < ELLIP_FIT_MAX_NORM_UT
           && r.cal_norm_max_err < ELLIP_FIT_MAX_ERR_RATIO;

    return r;
}

FullEllipFitResult FitEllipsoidFullAlgebraic(
    const float sample_buffer[][3],
    const size_t count,
    const float min_uT[3],
    const float max_uT[3])
{
    FullEllipFitResult r{};

    if (count < 100u) { return r; }

    // ── Normalization parameters ──
    float b0[3];
    float R0 = 0.0F;
    for (int i = 0; i < 3; ++i) {
        b0[i] = (max_uT[i] + min_uT[i]) * 0.5F;
        R0   += (max_uT[i] - min_uT[i]) * 0.5F;
    }
    R0 /= 3.0F;
    r.R0_uT = R0;
    if (R0 < 1.0F) { return r; }

    // ── Accumulate 10×10 S = Σ f_i f_i^T ──
    double S[10][10] = {};
    for (size_t k = 0u; k < count; ++k) {
        const double z0 = (static_cast<double>(sample_buffer[k][0]) - static_cast<double>(b0[0])) / static_cast<double>(R0);
        const double z1 = (static_cast<double>(sample_buffer[k][1]) - static_cast<double>(b0[1])) / static_cast<double>(R0);
        const double z2 = (static_cast<double>(sample_buffer[k][2]) - static_cast<double>(b0[2])) / static_cast<double>(R0);

        const double f[10] = {
            z0 * z0,
            z1 * z1,
            z2 * z2,
            2.0 * z0 * z1,
            2.0 * z0 * z2,
            2.0 * z1 * z2,
            z0,
            z1,
            z2,
            1.0,
        };

        for (int i = 0; i < 10; ++i) {
            for (int j = 0; j < 10; ++j) {
                S[i][j] += f[i] * f[j];
            }
        }
    }

    // ── Inverse power iteration: smallest eigenvector of S ──
    double theta[10];
    bool   conv;
    uint8_t iters;
    if (!InversePowerSmallestEigen10x10(S, theta, iters, conv)) {
        r.solver_iters     = iters;
        r.solver_converged = conv;
        return r;
    }
    r.solver_iters     = iters;
    r.solver_converged = conv;
    if (!conv) { return r; }  // 未收敛的 θ 不可用

    // ── Extract A, d, c from θ ──
    double A[3][3] = {
        { theta[0], theta[3], theta[4] },
        { theta[3], theta[1], theta[5] },
        { theta[4], theta[5], theta[2] },
    };
    const double d[3] = { theta[6], theta[7], theta[8] };
    const double c_z  = theta[9];

    // ── Center: b_z = -0.5 * A^{-1} * d ──
    double A_inv[3][3];
    if (!Inverse3x3(A, A_inv)) { return r; }

    double b_z[3] = {};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            b_z[i] += A_inv[i][j] * d[j];
        }
        b_z[i] *= -0.5;
    }

    // ── Scale normalization + sign handling ──
    double k_raw = 0.0;
    {
        double Ab[3] = {};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) { Ab[i] += A[i][j] * b_z[j]; }
        }
        for (int i = 0; i < 3; ++i) { k_raw += b_z[i] * Ab[i]; }
    }
    k_raw -= c_z;

    if (std::abs(k_raw) < 1.0e-12) { r.k_raw = static_cast<float>(k_raw); return r; }

    // b_z 和 k_raw 已确定；后续只需 Q_z = A / k_raw。
    // k_raw < 0 表示 θ 整体符号取反；仅翻转 A 与 k_raw 使 Q_z 保持等价且 k_raw 为正。
    if (k_raw < 0.0) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) { A[i][j] = -A[i][j]; }
        }
        k_raw = -k_raw;
    }
    r.k_raw = static_cast<float>(k_raw);

    // Q_z = A / k_raw
    double Q_z[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) { Q_z[i][j] = A[i][j] / k_raw; }
    }

    // ── Positive definiteness + condition number ──
    {
        double V[3][3], lambda[3];
        if (!Jacobi3x3(Q_z, V, lambda)) { return r; }

        if (lambda[0] < lambda[1]) { SwapEigenPair(V, lambda, 0, 1); }
        if (lambda[0] < lambda[2]) { SwapEigenPair(V, lambda, 0, 2); }
        if (lambda[1] < lambda[2]) { SwapEigenPair(V, lambda, 1, 2); }

        r.Q_positive_definite = (lambda[2] > 0.0);
        if (!r.Q_positive_definite) { return r; }

        r.condition_number = static_cast<float>(lambda[0] / lambda[2]);
        if (r.condition_number > 500.0F) { return r; }

        // ── SPD sqrt: Q_sqrt = V sqrt(Λ) V^T ──
        double Q_sqrt[3][3];
        SqrtSPD3x3(V, lambda, Q_sqrt);

        // ── De-normalize: b_raw, Q_raw ──
        for (int i = 0; i < 3; ++i) {
            r.bias_body_uT[i] = b0[i] + R0 * static_cast<float>(b_z[i]);
        }

        // Q_raw = Q_z / R0², applied via M = R_target * (Q_sqrt / R0)
        // since Q_sqrt is sqrt(Q_z), Q_raw_sqrt = Q_sqrt / R0
        const double inv_R0 = 1.0 / static_cast<double>(R0);

        // ── R_target: mean(||x_i - b_raw||) ──
        double R_target = 0.0;
        for (size_t k = 0u; k < count; ++k) {
            const double y0 = static_cast<double>(sample_buffer[k][0]) - static_cast<double>(r.bias_body_uT[0]);
            const double y1 = static_cast<double>(sample_buffer[k][1]) - static_cast<double>(r.bias_body_uT[1]);
            const double y2 = static_cast<double>(sample_buffer[k][2]) - static_cast<double>(r.bias_body_uT[2]);
            R_target += std::sqrt(y0 * y0 + y1 * y1 + y2 * y2);
        }
        R_target /= static_cast<double>(count);
        r.R_target_uT = static_cast<float>(R_target);

        // M = R_target * (Q_sqrt / R0) = (R_target / R0) * Q_sqrt
        const double scale = R_target * inv_R0;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                r.matrix_3x3[i][j] = static_cast<float>(scale * Q_sqrt[i][j]);
            }
        }

        // ── Self-validation ──
        float  cal_min  = FLT_MAX;
        float  cal_max  = -FLT_MAX;
        double cal_sum  = 0.0;
        double cal_sum2 = 0.0;

        for (size_t k = 0u; k < count; ++k) {
            const float y[3] = {
                sample_buffer[k][0] - r.bias_body_uT[0],
                sample_buffer[k][1] - r.bias_body_uT[1],
                sample_buffer[k][2] - r.bias_body_uT[2],
            };
            float cal[3] = {};
            for (int i = 0; i < 3; ++i) {
                cal[i] = r.matrix_3x3[i][0] * y[0]
                       + r.matrix_3x3[i][1] * y[1]
                       + r.matrix_3x3[i][2] * y[2];
            }
            const float cn = std::sqrt(cal[0] * cal[0] + cal[1] * cal[1] + cal[2] * cal[2]);

            if (cn < cal_min) { cal_min = cn; }
            if (cn > cal_max) { cal_max = cn; }
            cal_sum  += static_cast<double>(cn);
            cal_sum2 += static_cast<double>(cn) * static_cast<double>(cn);
        }

        r.cal_norm_min_uT  = cal_min;
        r.cal_norm_max_uT  = cal_max;
        r.cal_norm_mean_uT = static_cast<float>(cal_sum / static_cast<double>(count));
        {
            const double m   = static_cast<double>(r.cal_norm_mean_uT);
            const double var = cal_sum2 / static_cast<double>(count) - m * m;
            r.cal_norm_std_uT = (var > 0.0) ? static_cast<float>(std::sqrt(var)) : 0.0F;
        }
        if (r.cal_norm_mean_uT > 0.0F) {
            const float max_dev = (cal_max - r.cal_norm_mean_uT) > (r.cal_norm_mean_uT - cal_min)
                                ? (cal_max - r.cal_norm_mean_uT)
                                : (r.cal_norm_mean_uT - cal_min);
            r.cal_norm_max_err = max_dev / r.cal_norm_mean_uT;
        }

        // ── Acceptance ──
        r.valid = r.solver_converged
               && r.Q_positive_definite
               && r.condition_number < ELLIP_FIT_MAX_COND
               && r.cal_norm_std_uT  < ELLIP_FIT_MAX_STD_UT
               && r.cal_norm_mean_uT > ELLIP_FIT_MIN_MEAN_UT
               && r.cal_norm_mean_uT < ELLIP_FIT_MAX_MEAN_UT
               && r.cal_norm_min_uT  > ELLIP_FIT_MIN_NORM_UT
               && r.cal_norm_max_uT  < ELLIP_FIT_MAX_NORM_UT
               && r.cal_norm_max_err < ELLIP_FIT_MAX_ERR_RATIO;
    }

    return r;
}

} // namespace ist8310_calibration
