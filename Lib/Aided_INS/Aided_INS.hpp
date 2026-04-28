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
using Aided_INS_Space::Mag;

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

    /**
     * @brief 获取IMU数据
     */
    bool GetImuData();

    /**
     * @brief 获取磁力计数据
     */
    bool GetMagData();

    /**
     * @brief
     * @return
     */
    static Aided_INS_Space::Config LoadConfig();

    /**
     * @brief 初始化系统状态和协方差
     *        initialize state and state covariance
     * @param config
     */
    void Initialize(const Aided_INS_Space::Config &config);

    /**
    * @brief 内插增量形式的IMU数据到指定时刻
    *        interpolate incremental imudata to given timestamp
    * @param [in]     imuPre      前一时刻IMU数据
    *                           the previous imudata
    * @param [in,out] imuCur      当前时刻IMU数据
    *                           the current imudata
    * @param [in]     timestamp 给定内插到的时刻
    *                           given interpolate timestamp
    * @param [in,out] middle    输出内插时刻的IMU数据
    *                           output imudata at given timestamp
    */
    static void imuInterpolate(const IMU &imuPre, IMU &imuCur, const double timestamp, IMU &middle)
    {

        if (imuPre.time > timestamp || imuCur.time < timestamp)
            return;

        const double lamda = (timestamp - imuPre.time) / (imuCur.time - imuPre.time);

        middle.time       = timestamp;
        middle.deltaTheta = imuCur.deltaTheta * lamda;
        middle.deltaVel   = imuCur.deltaVel * lamda;
        middle.dt         = timestamp - imuPre.time;

        imuCur.deltaTheta = imuCur.deltaTheta - middle.deltaTheta;
        imuCur.deltaVel   = imuCur.deltaVel - middle.deltaVel;
        imuCur.dt         = imuCur.dt - middle.dt;
    }

    /**
    * @brief 当前IMU误差补偿到IMU数据中
    *        componsate imu error to the imudata
    * @param [in,out] imu 需要补偿的IMU数据
    *                     imudata to be compensated
    */
    void ImuCompensate(IMU &imu) const;

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

    /**
     * @brief 加速度计观测方程
     * @param imuData IMU的采样数据
     * @param pvaCur 当前时刻IMU的导航解
     * @param config 初始配置
     */
    bool AccUpdate(const IMU& imuData, const PVA& pvaCur, const Aided_INS_Space::Config& config);

    /**
     * @brief 磁力计观测方程
     * @param magData 磁力计的采样数据
     * @param pvaCur 当前时刻IMU的导航解
     * @param config 初始配置
     */
    void MagUpdate(const Mag &magData, const PVA &pvaCur, const Aided_INS_Space::Config &config);

    /**
     * @brief 使用GNSS位置观测更新系统状态
     *        update state using gnss position
     * @param [in,out] gnssData
     */
    void GnssUpdate(GNSS &gnssData);

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
    void EkfUpdate(const MatrixXd &dz, MatrixXd &H, const MatrixXd &R);

    /**
     * @brief 反馈误差状态到当前状态
     *        feedback error state to the current state
     */
    void StateFeedback();

    enum class KfUpdateType: uint8_t
    {
        None,  //不需要KF更新
        Prev,  //需要KF更新上一IMU状态
        Curr,  //需要KF更新当前IMU状态
        Middle //需要将IMU进行内插到观测量更新时间
    };

    /**
     * @brief 判断是否需要KF更新
     * @param imuTime1 上一IMU状态时间
     * @param imuTime2 当前IMU状态时间
     * @param updateTime 观测量更新的时间
     * @return 需要KF的更新类型
     */
    [[nodiscard]] KfUpdateType IsToUpdate(double imuTime1, double imuTime2, double updateTime) const;

    /**
     * @brief 处理新数据
     */
    void ProcessNewData();


    Aided_INS_Space::Config config_;

    double timestamp_{0.0};

    // 更新时间对齐误差，IMU状态和观测信息误差小于它则认为两者对齐
    const double TIME_ALIGN_ERR_ = 0.001;

    // IMU、磁力计和GNSS原始数据
    IMU imuPre_;
    IMU imuCur_;
    Mag magData_;
    GNSS gnssData_;

    // IMU状态（位置、速度、姿态和IMU误差）
    PVA pvaCur_;
    PVA pvaPre_;
    ImuError imuError_;

    // Kalman滤波相关
    MatrixXd P_;  //协方差矩阵
    MatrixXd q_;  //系统噪声矩阵
    MatrixXd dx_; //误差状态向量

    const int RANK_       = 21;
    const int NOISE_RANK_ = 18;

    // 状态ID和噪声ID
    enum StateID { P_ID = 0, V_ID = 3, PHI_ID = 6, GB_ID = 9, AB_ID = 12, GS_ID = 15, AS_ID = 18 };
    enum NoiseID { VRW_ID = 0, ARW_ID = 3, GBSTD_ID = 6, ABSTD_ID = 9, GSSTD_ID = 12, ASSTD_ID = 15 };

    //传感器驱动类
    // ImuDriver& imu_;
    // MagDriver& mag_;
    // GNSSDriver& gnss_;
};

#endif //AHRS_AIDED_INS_H
