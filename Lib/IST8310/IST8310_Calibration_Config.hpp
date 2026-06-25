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

// 第一版 MCU 端校准结果（2026-06-25）
constexpr float kMagHardIronBiasBody_uT[3] = {
    11.82F, 15.00F, -15.45F,
};

constexpr float kMagScaleBody[3] = {
    0.95F, 1.04F, 1.01F,
};

} // namespace ist8310_calibration_config
