/**
 * @file    IST8310_Calibration_Config.hpp
 * @brief   IST8310 磁力计校准参数配置（手动填入，重新编译生效）
 *
 * @details
 * 参数由 MCU 端校准 [mag_cal] printf 输出，复制填入后重新编译烧录。
 * 不涉及 Flash 存储，不涉及文件系统。
 */

#pragma once

#ifndef IST8310_ENABLE_MAG_CALIBRATION
#define IST8310_ENABLE_MAG_CALIBRATION 1
#endif

namespace ist8310_calibration_config
{

// 当前校准参数（6-side calibration, recommended=diagonal）
//   formula:
//     centered = mag_body - bias_body
//     mag_cal = M_body * centered
//     M_body: symmetric 3x3, diag=scale_body, offdiag=kMagOffDiagScaleBody
//   当前采用 recommended=diagonal，因此 offdiag={0,0,0}，实际退化为 diag(scale_body) * centered
//   坐标系: body-frame, 映射: body_x=sensor_y, body_y=-sensor_x, body_z=-sensor_z
constexpr float kMagHardIronBiasBody_uT[3] = {
    19.55F, 9.85F, -6.21F,
};

constexpr float kMagScaleBody[3] = {
    1.04F, 1.00F, 0.97F,
};

// recommended=diagonal，offdiag 清零
constexpr float kMagOffDiagScaleBody[3] = {
    0.0000F, 0.0000F, 0.0000F,
};

// 第三次 MCU 端校准结果（2026-06-26，已弃用）
//   samples=1006, quality=0.95, radius_avg_uT=45.45
//   span_body_uT={90.91F, 88.79F, 93.03F}
//   raw norm: min=10.32 max=104.61 range_ratio=1.641 out_of_range=36
// constexpr float kMagHardIronBiasBody_uT[3] = { 48.48F, 14.39F, 14.70F };
// constexpr float kMagScaleBody[3]            = {  1.00F,  1.02F,   0.98F };

// 第一版 MCU 端校准结果（2026-06-25，已弃用）
//   samples=1004, quality=0.92, radius_avg_uT=43.69
//   span_body_uT={91.52F, 83.94F, 86.67F}
// constexpr float kMagHardIronBiasBody_uT[3] = { 11.82F, 15.00F, -15.45F };
// constexpr float kMagScaleBody[3]            = {  0.95F,  1.04F,   1.01F };

} // namespace ist8310_calibration_config
