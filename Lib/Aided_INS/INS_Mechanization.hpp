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

using namespace Aided_INS_Space;

class INS_Mechanization
{
public:
    /**
     * @brief INS机械编排算法, 利用IMU数据进行速度、位置和姿态更新
     *        INS Mechanization, update velocity, position and attitude using imudata
     * @param [in]     PVA_Pre 上一时刻状态
     *                        the last imustate
     * @param [in,out] PVA_Cur 输出当前时刻状态
     *                        output the current imustate
     * @param [in]     IMU_Pre, IMU_Cur imudata
     * */
    static void INS_Mech(const PVA &PVA_Pre, PVA &PVA_Cur, const IMU &IMU_Pre, const IMU &IMU_Cur);

private:
    /**
     * @breif 位置更新
     *        position update
     * */
    static void PosUpdate(const PVA &PVA_Pre, PVA &PVA_Cur, const IMU &IMU_Pre, const IMU &IMU_Cur);

    /**
     * @breif 速度更新
     *        velocity update
     * */
    static void VelUpdate(const PVA &PVA_Pre, PVA &PVA_Cur, const IMU &IMU_Pre, const IMU &IMU_Cur);

    /**
     * @breif 姿态更新
     *        attitude update
     * */
    static void AttUpdate(const PVA &PVA_Pre, PVA &PVA_Cur, const IMU &IMU_Pre, const IMU &IMU_Cur);
};

#endif //AHRS_INS_MECHANIZATION_HPP
