/******************************************************************************
 * @file    Aided_INS.hpp
 * @brief
 *
 * @details
 *
 *
 * @author  Gray
 * @email   grayme12345@gmail.com
 * @date    2026/4/7
 *
 * @copyright
 * Copyright (c) 2026 Gray
 *
 * This software is provided "as is" without any warranties.
 *
 ******************************************************************************/

#ifndef AHRS_AIDED_INS_H
#define AHRS_AIDED_INS_H

#include <cstdint>
#include <cstdio>
#include <Eigen/Dense>
#include "Aided_INS_Types.hpp"

using Aided_INS_Space::NavState;
using Aided_INS_Space::IMU;
using Aided_INS_Space::GNSS;
using Aided_INS_Space::PVA;
using Aided_INS_Space::ImuError;

using Eigen::MatrixXd;

class Aided_INS
{
public:
    explicit Aided_INS(uint8_t id);

    /**
     * @brief 组合惯性导航系统初始化
     * @param
     * @retval
     */
    int Init();

    /**
      * @brief 运行导航算法
      * @param
      * @retval
      */
    int Run();

private:
    /**
     * @brief 初始对准
     * @param
     * @retval
     */
    int InitialAlignment();

    Aided_INS_Space::Config LoadConfig();

    /**
     * @brief 初始化系统状态和协方差
     *        initialize state and state covariance
     * @param config
     */
    void Initialize(const Aided_INS_Space::Config &config);

    /**
     * @brief 当前IMU误差补偿到IMU数据中
     *        componsate imu error to the imudata
     * @param [in,out] imu 需要补偿的IMU数据
     *                     imudata to be compensated
     */
    void ImuCompensate(IMU &imu) const;

// /**
//  * @brief 判断是否需要更新,以及更新哪一时刻系统状态
//  *        determine if we should do upate and which navstate to update
//  * @param [in] imutime1   上一IMU状态时间
//  *                        the last state time
//  * @param [in] imutime2   当前IMU状态时间
//  *                        the current state time
//  * @param [in] updatetime 状态更新的时间
//  *                        time to update state
//  * @return 0: 不需要更新
//  *            donot need update
//  *         1: 需要更新上一IMU状态
//  *            update the last navstate
//  *         2: 需要更新当前IMU状态
//  *            update the current navstate
//  *         3: 需要将IMU进行内插到状态更新时间
//  *            need interpolate imudata to updatetime
//  */
// int isToUpdate(double imutime1, double imutime2, double updatetime) const;

    /**
     * @brief 进行INS状态更新(IMU机械编排算法), 并计算IMU状态转移矩阵和噪声阵
     *        do INS state update(INS mechanization), and compute state transition matrix and noise matrix
     * @param [in,out] imuPre 前一时刻IMU数据
     *                        imudata at the previous epoch
     * @param [in,out] imuCur 当前时刻IMU数据
     *                        imudata at the current epoch
     */
    void InsPropagation(const IMU &imuPre, IMU &imuCur);

    /**
     * @brief Kalman 预测,
     *        Kalman Filter Predict process
     * @param [in,out] Phi 状态转移矩阵
     *                     state transition matrix
     * @param [in,out] Q  传播噪声矩阵
     *                     propagation noise matrix
     */
    void EKFPredict(const MatrixXd &Phi, const MatrixXd &Q);


// /**
//  * @brief 使用GNSS位置观测更新系统状态
//  *        update state using gnss position
//  * @param [in,out] gnssdata
//  */
// void gnssUpdate(GNSS &gnssdata);

    /**
     * @brief Kalman 更新
     *        Kalman Filter Update process
     * @param [in] dz 观测新息
     *                measurement innovation
     * @param [in] H  观测矩阵
     *                measurement matrix
     * @param [in] R  观测噪声阵
     *                measurement noise matrix
     */
    void EkfUpdate(MatrixXd &dz, MatrixXd &H, MatrixXd &R);

    /**
     * @brief 反馈误差状态到当前状态
     *        feedback error state to the current state
     */
    void StateFeedback();


    Aided_INS_Space::Config config_;

    double timestamp_;

    // 更新时间对齐误差，IMU状态和观测信息误差小于它则认为两者对齐
    const double TIME_ALIGN_ERR_ = 0.001;

    // IMU和GNSS原始数据
    IMU imuPre_;
    IMU imuCur_;
    GNSS gnssData_;

    // IMU状态（位置、速度、姿态和IMU误差）
    PVA pvaCur_;
    PVA pvaPre_;
    ImuError imuError_;

    // Kalman滤波相关
    Eigen::MatrixXd P_;
    Eigen::MatrixXd q_;
    Eigen::MatrixXd dx_;

    const int RANK_       = 21;
    const int NOISE_RANK_ = 18;

    // 状态ID和噪声ID
    enum StateID { P_ID = 0, V_ID = 3, PHI_ID = 6, GB_ID = 9, AB_ID = 12, GS_ID = 15, AS_ID = 18 };
    enum NoiseID { VRW_ID = 0, ARW_ID = 3, GBSTD_ID = 6, ABSTD_ID = 9, GSSTD_ID = 12, ASSTD_ID = 15 };
};

#endif //AHRS_AIDED_INS_H
