/******************************************************************************
 * @file    INS_Mechanization.hpp
 * @brief   惯导机械编排算法
 *
 * @details
 * 
 *
 * @author  Gray
 * @email   grayme12345@gmail.com
 * @date    2026/4/16
 *
 * @copyright
 * Copyright (c) 2026 Gray
 *
 * This software is provided "as is" without any warranties.
 *
 ******************************************************************************/

#ifndef AHRS_INS_MECHANIZATION_HPP
#define AHRS_INS_MECHANIZATION_HPP

#include "Aided_INS_Types.hpp"

using Aided_INS_Space::PVA;
using Aided_INS_Space::IMU;

namespace INS_Mechanization
{
    /**
     * @brief INS机械编排算法, 利用IMU数据进行速度、位置和姿态更新
     *        INS Mechanization, update velocity, position and attitude using imudata
     * @param [in]     pvaPre 上一时刻状态
     *                        the previous imu state
     * @param [in,out] pvaCur 输出当前时刻状态
     *                        output the current imu state
     * @param [in]     imuPre, imuCur imu data
     */
    void INS_Mech(const PVA &pvaPre, PVA &pvaCur, const IMU &imuPre, const IMU &imuCur);
}

#endif //AHRS_INS_MECHANIZATION_HPP
