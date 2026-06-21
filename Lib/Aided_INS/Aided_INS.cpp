/******************************************************************************
 * @file    Aided_INS.cpp
 * @brief   输入系统的一些参数
 *
 * @details
 * 包括初始状态、初始状态标准差、IMU噪声建模参数、天线杆臂。
 * 导航坐标系（n系）采用北东地-NED坐标系，机体坐标系（b系）采用前右下-FRD坐标系。
 * IMU与b系固连。
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

#include "Aided_INS.hpp"
#include "Configuration.hpp"
#include "AngleUtilities.hpp"
#include "EarthUtilities.hpp"
#include "RotationUtilities.hpp"
#include "INS_Mechanization.hpp"
#include "SystemPort.h"  // [PROFILE]
#include "stm32h7xx.h"

#include <cassert>
#include <cmath>

using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;

using Earth::Gravity;
using Earth::RM_And_RN;
using Earth::WIE;
using Earth::DR_Inv;
using Earth::DR;
using Earth::w_ie_n;

using Rotation::Euler2DCM;
using Rotation::Euler2Quaternion;
using Rotation::SkewSymmetric;
using Rotation::RotVec2Quaternion;
using Rotation::Quaternion2DCM;
using Rotation::DCM2Euler;

using Angle::Deg2Rad;

using Aided_INS_Space::Config;

Aided_INS::Aided_INS(const uint8_t id)
    : id_(id)
{
    config_ = LoadConfig();
    Initialize();
}

int Aided_INS::Init()
{
    config_ = LoadConfig();
    Initialize();
    status_ = InsStatus::Unaligned;

#if 1  // 验证 BuildGQGt 等价性（通过后改为 #if 0 关闭）
    VerifyStructuredQ();
#endif

    return 0;
}

int Aided_INS::Run()
{
    switch (status_)
    {
        case InsStatus::Unaligned:
        {
            alignStartTime_ = HAL_GetTick();

            alignGyroSum_.setZero();
            alignAccSum_.setZero();
            alignMagSum_.setZero();
            alignCount_    = 0;
            alignMagCount_ = 0;

            status_ = InsStatus::Aligning;
            return 0;
        }

        case InsStatus::Aligning:
        {
            return InitialAlignment();
        }

        case InsStatus::Running:
        {
            const bool imuReady = GetImuData();
            (void)GetMagData();

            if (imuReady)
            {
                ProcessNewData();
                return 1;
            }

            return 0;
        }

        default:
            return -1;
    }
}

int Aided_INS::InitialAlignment()
{
    const bool imuReady = GetImuData();
    const bool magReady = GetMagData();

    if (!imuReady)
        return 0;

    // 当前瞬时值
    const Vector3d gyro = imuCur_.deltaTheta / imuCur_.dt;
    const Vector3d acc  = imuCur_.deltaVel   / imuCur_.dt;

    constexpr float ALIGN_ANGLE_SPEED_DEG_S = 1.0;
    if (gyro.norm() > Deg2Rad(ALIGN_ANGLE_SPEED_DEG_S)) //载体角速度过大，退出初始对准
    {
        status_ = InsStatus::Unaligned;
        return -1;
    }

    constexpr float ALIGN_ACCELERATION_MPS2 = 0.2; //载体运动角速度过大，退出初始对准
    if (std::fabs(acc.norm() - Gravity(config_.initState.pos)) > ALIGN_ACCELERATION_MPS2)
    {
        status_ = InsStatus::Unaligned;
        return -1;
    }

    // 累积
    alignGyroSum_ += gyro;
    alignAccSum_  += acc;
    alignCount_++;

    if (magReady)
    {
        alignMagSum_ += magData_.mag;
        alignMagCount_++;
    }

    const uint32_t elapsed = HAL_GetTick() - alignStartTime_;

    constexpr uint32_t ALIGN_TIME_MS = 3000;
    // 对准时间未到，继续累计
    if (elapsed < ALIGN_TIME_MS)
        return 0;

    // 防止除0
    if (alignCount_ == 0)
        return -1;

    // 平均
    const Vector3d gyroMean = alignGyroSum_ / static_cast<double>(alignCount_);
    const Vector3d accMean  = alignAccSum_  / static_cast<double>(alignCount_);

    const double ax = accMean(0);
    const double ay = accMean(1);
    const double az = accMean(2);

    // roll / pitch
    const double roll  = std::atan2(-ay, -az);
    const double pitch = std::atan2(ax, std::sqrt(ay * ay + az * az));

    // yaw：有磁力计时使用倾斜补偿磁航向，否则使用配置初值。
    double yaw = config_.initState.euler[2];

    if (alignMagCount_ > 0u)
    {
        const Vector3d magMean = alignMagSum_ / static_cast<double>(alignMagCount_);

        const double mx = magMean(0);
        const double my = magMean(1);
        const double mz = magMean(2);

        const double mx_h = mx * std::cos(pitch) + my * std::sin(roll) * std::sin(pitch) + mz * std::cos(roll) * std::sin(pitch);
        const double my_h = my * std::cos(roll) - mz * std::sin(roll);

        yaw = std::atan2(-my_h, mx_h);
    }

    // 初始化姿态
    pvaCur_.att.euler << roll, pitch, yaw;
    pvaCur_.att.Cbn = Euler2DCM(pvaCur_.att.euler);
    pvaCur_.att.qbn = Euler2Quaternion(pvaCur_.att.euler);

    pvaPre_ = pvaCur_;
    imuPre_ = imuCur_;

    // 估计初始陀螺零偏，扣除地球自转角速度
    imuError_.gyrBias = gyroMean - pvaCur_.att.Cbn.transpose() * w_ie_n(config_.initState.pos[0]);

    // 清零误差状态
    dx_.setZero();

    //清除磁力计更新标志位，避免刚进入运行时重复用旧磁力计。
    magData_.isUpdate = false;

    // 进入运行
    status_ = InsStatus::Running;

    return 1;
}

void Aided_INS::SetImuData(const Aided_INS_Space::IMU &imu)
{
    imuCur_ = imu;
    imuReady_ = true;
}

bool Aided_INS::GetImuData()
{
    if (!imuReady_)
    {
        return false;
    }

    imuReady_ = false;
    return true;
}

bool Aided_INS::GetMagData()
{
    // if ()
    // {
    //     magData_.isUpdate = true;
    //     return true;
    // }

    return false;
}

Config Aided_INS::LoadConfig()
{
    using namespace Aided_INS_Config;
    using Angle::D2R;

    Config config{};

    constexpr double DEG_TO_RAD   = D2R;
    constexpr double HOUR_TO_SEC  = 3600.0;
    constexpr double MGAL_TO_MPS2 = 1.0e-5; // 1 mGal = 1e-5 m/s^2
    constexpr double PPM_TO_SCALE = 1.0e-6; // ppm：Parts Per Million，百万分之一

    /********************************************初始状态********************************************/
    config.initState.pos << StateInit::initPosLatitude  * DEG_TO_RAD,
                            StateInit::initPosLongitude * DEG_TO_RAD,
                            StateInit::initPosAltitude;

    config.initState.vel << StateInit::initVelNorth,
                            StateInit::initVelEast,
                            StateInit::initVelDown;

    config.initState.euler << StateInit::initAttRoll  * DEG_TO_RAD,
                              StateInit::initAttPitch * DEG_TO_RAD,
                              StateInit::initAttYaw   * DEG_TO_RAD;

    config.initState.gyrBias << StateInit::initGyrBiasX * DEG_TO_RAD / HOUR_TO_SEC,
                                StateInit::initGyrBiasY * DEG_TO_RAD / HOUR_TO_SEC,
                                StateInit::initGyrBiasZ * DEG_TO_RAD / HOUR_TO_SEC;

    config.initState.accBias << StateInit::initAccBiasX * MGAL_TO_MPS2,
                                StateInit::initAccBiasY * MGAL_TO_MPS2,
                                StateInit::initAccBiasZ * MGAL_TO_MPS2;

    config.initState.gyrScale << StateInit::initGyrScaleX * PPM_TO_SCALE,
                                 StateInit::initGyrScaleY * PPM_TO_SCALE,
                                 StateInit::initGyrScaleZ * PPM_TO_SCALE;

    config.initState.accScale << StateInit::initAccScaleX * PPM_TO_SCALE,
                                 StateInit::initAccScaleY * PPM_TO_SCALE,
                                 StateInit::initAccScaleZ * PPM_TO_SCALE;
    /**********************************************************************************************/

    /*****************************************初始状态标准差*****************************************/
    config.initStateStd.pos << ErrStateStdInit::initPosStdLatitude,
                               ErrStateStdInit::initPosStdLongitude,
                               ErrStateStdInit::initPosStdAltitude;

    config.initStateStd.vel << ErrStateStdInit::initVelStdNorth,
                               ErrStateStdInit::initVelStdEast,
                               ErrStateStdInit::initVelStdDown;

    config.initStateStd.euler << ErrStateStdInit::initAttStdRoll  * DEG_TO_RAD,
                                 ErrStateStdInit::initAttStdPitch * DEG_TO_RAD,
                                 ErrStateStdInit::initAttStdYaw   * DEG_TO_RAD;

    config.initStateStd.gyrBias << ErrStateStdInit::initGyrBiasStdX * DEG_TO_RAD / HOUR_TO_SEC,
                                   ErrStateStdInit::initGyrBiasStdY * DEG_TO_RAD / HOUR_TO_SEC,
                                   ErrStateStdInit::initGyrBiasStdZ * DEG_TO_RAD / HOUR_TO_SEC;

    config.initStateStd.accBias << ErrStateStdInit::initAccBiasStdX * MGAL_TO_MPS2,
                                   ErrStateStdInit::initAccBiasStdY * MGAL_TO_MPS2,
                                   ErrStateStdInit::initAccBiasStdZ * MGAL_TO_MPS2;

    config.initStateStd.gyrScale << ErrStateStdInit::initGyrScaleStdX * PPM_TO_SCALE,
                                    ErrStateStdInit::initGyrScaleStdY * PPM_TO_SCALE,
                                    ErrStateStdInit::initGyrScaleStdZ * PPM_TO_SCALE;

    config.initStateStd.accScale << ErrStateStdInit::initAccScaleStdX * PPM_TO_SCALE,
                                    ErrStateStdInit::initAccScaleStdY * PPM_TO_SCALE,
                                    ErrStateStdInit::initAccScaleStdZ * PPM_TO_SCALE;
    /**********************************************************************************************/

    /*******************************************IMU噪声参数******************************************/
    config.imuNoise.gyrArw << IMU_Noise::gyrArwX * DEG_TO_RAD / std::sqrt(HOUR_TO_SEC),
                              IMU_Noise::gyrArwY * DEG_TO_RAD / std::sqrt(HOUR_TO_SEC),
                              IMU_Noise::gyrArwZ * DEG_TO_RAD / std::sqrt(HOUR_TO_SEC);

    config.imuNoise.accVrw << IMU_Noise::accVrwX / std::sqrt(HOUR_TO_SEC),
                              IMU_Noise::accVrwY / std::sqrt(HOUR_TO_SEC),
                              IMU_Noise::accVrwZ / std::sqrt(HOUR_TO_SEC);

    config.imuNoise.gyrBiasStd << IMU_Noise::gyrBiasStdX * DEG_TO_RAD / HOUR_TO_SEC,
                                  IMU_Noise::gyrBiasStdY * DEG_TO_RAD / HOUR_TO_SEC,
                                  IMU_Noise::gyrBiasStdZ * DEG_TO_RAD / HOUR_TO_SEC;

    config.imuNoise.accBiasStd << IMU_Noise::accBiasStdX * MGAL_TO_MPS2,
                                  IMU_Noise::accBiasStdY * MGAL_TO_MPS2,
                                  IMU_Noise::accBiasStdZ * MGAL_TO_MPS2;

    config.imuNoise.gyrScaleStd << IMU_Noise::gyrScaleStdX * PPM_TO_SCALE,
                                   IMU_Noise::gyrScaleStdY * PPM_TO_SCALE,
                                   IMU_Noise::gyrScaleStdZ * PPM_TO_SCALE;

    config.imuNoise.accScaleStd << IMU_Noise::accScaleStdX * PPM_TO_SCALE,
                                   IMU_Noise::accScaleStdY * PPM_TO_SCALE,
                                   IMU_Noise::accScaleStdZ * PPM_TO_SCALE;

    config.imuNoise.corr_time = IMU_Noise::imuCorrTime * HOUR_TO_SEC;
    /**********************************************************************************************/

    /*****************************************磁力计噪声参数******************************************/
    config.magMeasureYawStd = MagNoise::magMeasureYawStd * DEG_TO_RAD;
    /**********************************************************************************************/

    /**********************************天线杆臂，IMU坐标系前右下方向************************************/
    config.antennaLever << GNSS_AntennaLever::gnssAntennaLeverX,
                           GNSS_AntennaLever::gnssAntennaLeverY,
                           GNSS_AntennaLever::gnssAntennaLeverZ;
    /**********************************************************************************************/

    return config;
}

void Aided_INS::Initialize()
{
    timestamp_ = 0;

    //设置协方差矩阵Cov，系统噪声阵q和系统误差状态矩阵dx（固定尺寸，不需要 resize）
    P_.setZero();
    q_.setZero();
    dx_.setZero();

    // 初始化系统噪声阵q
    auto imuNoise = config_.imuNoise;
    q_.block(VRW_ID, VRW_ID, 3, 3) = imuNoise.accVrw.cwiseProduct(imuNoise.accVrw).asDiagonal();
    q_.block(ARW_ID, ARW_ID, 3, 3) = imuNoise.gyrArw.cwiseProduct(imuNoise.gyrArw).asDiagonal();
    q_.block(GBSTD_ID, GBSTD_ID, 3, 3) =
        2 / imuNoise.corr_time * imuNoise.gyrBiasStd.cwiseProduct(imuNoise.gyrBiasStd).asDiagonal();
    q_.block(ABSTD_ID, ABSTD_ID, 3, 3) =
        2 / imuNoise.corr_time * imuNoise.accBiasStd.cwiseProduct(imuNoise.accBiasStd).asDiagonal();
    q_.block(GSSTD_ID, GSSTD_ID, 3, 3) =
        2 / imuNoise.corr_time * imuNoise.gyrScaleStd.cwiseProduct(imuNoise.gyrScaleStd).asDiagonal();
    q_.block(ASSTD_ID, ASSTD_ID, 3, 3) =
        2 / imuNoise.corr_time * imuNoise.accScaleStd.cwiseProduct(imuNoise.accScaleStd).asDiagonal();

    NavState &initState    = config_.initState;
    NavState &initStateStd = config_.initStateStd;

    //初始化位置、速度、姿态
    pvaCur_.pos       = initState.pos;
    pvaCur_.vel       = initState.vel;
    pvaCur_.att.euler = initState.euler;
    pvaCur_.att.Cbn   = Euler2DCM(pvaCur_.att.euler);
    pvaCur_.att.qbn   = Euler2Quaternion(pvaCur_.att.euler);

    //初始化IMU误差
    imuError_.gyrBias  = initState.gyrBias;
    imuError_.accBias   = initState.accBias;
    imuError_.gyrScale = initState.gyrScale;
    imuError_.accScale  = initState.accScale;

    //给上一时刻状态赋同样的初值
    pvaPre_ = pvaCur_;

    //初始化协方差
    P_.block(P_ID, P_ID, 3, 3)     = initStateStd.pos.cwiseProduct(initStateStd.pos).asDiagonal();
    P_.block(V_ID, V_ID, 3, 3)     = initStateStd.vel.cwiseProduct(initStateStd.vel).asDiagonal();
    P_.block(PHI_ID, PHI_ID, 3, 3) = initStateStd.euler.cwiseProduct(initStateStd.euler).asDiagonal();
    P_.block(GB_ID, GB_ID, 3, 3)   = initStateStd.gyrBias.cwiseProduct(initStateStd.gyrBias).asDiagonal();
    P_.block(AB_ID, AB_ID, 3, 3)   = initStateStd.accBias.cwiseProduct(initStateStd.accBias).asDiagonal();
    P_.block(GS_ID, GS_ID, 3, 3)   = initStateStd.gyrScale.cwiseProduct(initStateStd.gyrScale).asDiagonal();
    P_.block(AS_ID, AS_ID, 3, 3)   = initStateStd.accScale.cwiseProduct(initStateStd.accScale).asDiagonal();
}

void Aided_INS::ImuCompensate(IMU &imu) const
{
    //补偿IMU零偏
    imu.deltaTheta -= imuError_.gyrBias * imu.dt;
    imu.deltaVel   -= imuError_.accBias * imu.dt;

    //补偿IMU比例因子
    const Vector3d gyrScale = Vector3d::Ones() + imuError_.gyrScale;
    const Vector3d accScale = Vector3d::Ones() + imuError_.accScale;
    imu.deltaTheta = imu.deltaTheta.cwiseProduct(gyrScale.cwiseInverse());
    imu.deltaVel   = imu.deltaVel.cwiseProduct(accScale.cwiseInverse());
}

// ============================================================================
// VerifyStructuredQ — 调用 BuildGQGtFromNoise，确保与正式路径共用同一实现
// ============================================================================

#if 1  // 验证：确认等价后改为 #if 0 关闭

void Aided_INS::VerifyStructuredQ()
{
    // 构造非零 q_test：6 个 3×3 块各填不同非零值，覆盖对角和非对角
    NoiseMatrix q_test;
    q_test.setZero();

    for (int b = 0; b < 6; ++b)
    {
        const int id = b * 3; // VRW=0, ARW=3, GBSTD=6, ABSTD=9, GSSTD=12, ASSTD=15
        Eigen::Matrix3d blk;
        blk << 1.0 + b,  0.1,        0.2,
               0.1,       2.0 + b,    0.3,
               0.2,       0.3,        3.0 + b;
        q_test.template block<3, 3>(id, id) = blk;
    }

    // 构造非平凡 Cbn（30° roll 旋转）和 Cbn_cur（-20° pitch）
    const double angle_pre = 0.5235987756;   // 30°
    const double angle_cur = -0.3490658504;  // -20°
    const Matrix3d Cbn_pre = (Eigen::AngleAxisd(angle_pre, Vector3d::UnitX())).toRotationMatrix();
    const Matrix3d Cbn_cur = (Eigen::AngleAxisd(angle_cur, Vector3d::UnitY())).toRotationMatrix();

    // -------------------------------
    // 验证 1: Qbase 结构化 vs 稠密
    // -------------------------------
    NoiseDriveMatrix G_test;
    G_test.setZero();
    G_test.template block<3, 3>(V_ID,   VRW_ID)   = Cbn_pre;
    G_test.template block<3, 3>(PHI_ID, ARW_ID)   = Cbn_pre;
    G_test.template block<3, 3>(GB_ID,  GBSTD_ID) = Matrix3d::Identity();
    G_test.template block<3, 3>(AB_ID,  ABSTD_ID) = Matrix3d::Identity();
    G_test.template block<3, 3>(GS_ID,  GSSTD_ID) = Matrix3d::Identity();
    G_test.template block<3, 3>(AS_ID,  ASSTD_ID) = Matrix3d::Identity();

    const StateMatrix Qbase_ref = G_test * q_test * G_test.transpose();

    StateMatrix Qbase_opt;
    BuildGQGtFromNoise(Cbn_pre, q_test, Qbase_opt);  // 调用正式实现

    const double qbase_err = (Qbase_ref - Qbase_opt).cwiseAbs().maxCoeff();

    // -------------------------------
    // 验证 2: Q1 = Phi * Qbase * Phiᵀ（使用 BuildPhiQbasePhiT）
    // -------------------------------

    // 构造 MakeTestBlock 辅助 lambda：根据 seed 生成不同的 3×3 非平凡矩阵
    auto MakeTestBlock = [](double s) -> Eigen::Matrix3d {
        Eigen::Matrix3d m;
        m << s,       s*0.1,   s*0.2,
             s*0.3,   s,       s*0.4,
             s*0.5,   s*0.6,   s;
        return m;
    };

    // Phi_test：覆盖所有允许的非零 3×3 块
    StateMatrix Phi_test = StateMatrix::Identity();
    // 对角线块（用不同 seed 覆盖 I 的默认值）
    Phi_test.template block<3, 3>(P_ID,   P_ID)   = MakeTestBlock(1.0);
    Phi_test.template block<3, 3>(V_ID,   V_ID)   = MakeTestBlock(1.1);
    Phi_test.template block<3, 3>(PHI_ID, PHI_ID) = MakeTestBlock(1.2);
    Phi_test.template block<3, 3>(GB_ID,  GB_ID)  = MakeTestBlock(1.3);
    Phi_test.template block<3, 3>(AB_ID,  AB_ID)  = MakeTestBlock(1.4);
    Phi_test.template block<3, 3>(GS_ID,  GS_ID)  = MakeTestBlock(1.5);
    Phi_test.template block<3, 3>(AS_ID,  AS_ID)  = MakeTestBlock(1.6);
    // 非对角块（基于当前 F/Phi 固有块稀疏结构）
    Phi_test.template block<3, 3>(P_ID,   V_ID)   = MakeTestBlock(0.02);
    Phi_test.template block<3, 3>(V_ID,   P_ID)   = MakeTestBlock(0.03);
    Phi_test.template block<3, 3>(V_ID,   PHI_ID) = MakeTestBlock(0.04);
    Phi_test.template block<3, 3>(V_ID,   AB_ID)  = MakeTestBlock(0.05);
    Phi_test.template block<3, 3>(V_ID,   AS_ID)  = MakeTestBlock(0.06);
    Phi_test.template block<3, 3>(PHI_ID, P_ID)   = MakeTestBlock(0.07);
    Phi_test.template block<3, 3>(PHI_ID, V_ID)   = MakeTestBlock(0.08);
    Phi_test.template block<3, 3>(PHI_ID, GB_ID)  = MakeTestBlock(0.09);
    Phi_test.template block<3, 3>(PHI_ID, GS_ID)  = MakeTestBlock(0.10);

    // 稠密参考
    const StateMatrix Q1_ref = Phi_test * (G_test * q_test * G_test.transpose()) * Phi_test.transpose();

    // 结构化计算（调用正式运行路径使用的 BuildPhiQbasePhiT）
    StateMatrix Q1_opt;
    BuildPhiQbasePhiT(Phi_test, Qbase_opt, Q1_opt);
    const double q1_err = (Q1_ref - Q1_opt).cwiseAbs().maxCoeff();

    // -------------------------------
    // 验证 3: Q2 = (Q1 + Qbase_cur) * dt * 0.5
    // -------------------------------
    NoiseDriveMatrix G_cur_test;
    G_cur_test.setZero();
    G_cur_test.template block<3, 3>(V_ID,   VRW_ID)   = Cbn_cur;
    G_cur_test.template block<3, 3>(PHI_ID, ARW_ID)   = Cbn_cur;
    G_cur_test.template block<3, 3>(GB_ID,  GBSTD_ID) = Matrix3d::Identity();
    G_cur_test.template block<3, 3>(AB_ID,  ABSTD_ID) = Matrix3d::Identity();
    G_cur_test.template block<3, 3>(GS_ID,  GSSTD_ID) = Matrix3d::Identity();
    G_cur_test.template block<3, 3>(AS_ID,  ASSTD_ID) = Matrix3d::Identity();

    StateMatrix Qbase_cur_opt;
    BuildGQGtFromNoise(Cbn_cur, q_test, Qbase_cur_opt);  // 共用同一实现

    constexpr double dt = 0.005;  // 200 Hz
    const StateMatrix Q2_ref = (Q1_ref + G_cur_test * q_test * G_cur_test.transpose()) * dt * 0.5;
    const StateMatrix Q2_opt = (Q1_opt + Qbase_cur_opt) * dt * 0.5;
    const double q2_err = (Q2_ref - Q2_opt).cwiseAbs().maxCoeff();

    // -------------------------------
    // 验证 4: Stage 3a — M = Phi * P 结构化 vs 稠密
    // -------------------------------

    // 构造非平凡对称满矩阵 P_test
    StateMatrix P_test;
    P_test.setZero();
    for (int i = 0; i < kStateRank; ++i)
    {
        for (int j = i; j < kStateRank; ++j)
        {
            const double val = 1.0 + 0.1 * static_cast<double>(i + j);
            P_test(i, j) = val;
            P_test(j, i) = val;
        }
    }

    // 稠密参考
    const StateMatrix M_ref = Phi_test * P_test;

    // 结构化：BuildPhiTimesP
    StateMatrix M_opt;
    BuildPhiTimesP(Phi_test, P_test, M_opt);
    const double m_err = (M_ref - M_opt).cwiseAbs().maxCoeff();

    // 进一步：完整 EKFPredict 后的 P（M * Phiᵀ + Q 第二步暂保留稠密）
    // 用 Q1_opt 作为非零 Q_test（Q1_opt 已在前面验证为正确）
    const StateMatrix P_ref = Phi_test * P_test * Phi_test.transpose() + Q1_opt;
    const StateMatrix P_opt = M_opt * Phi_test.transpose() + Q1_opt;
    const double p_err = (P_ref - P_opt).cwiseAbs().maxCoeff();

    // -------------------------------
    // 输出
    // -------------------------------
    printf("[Q_VERIFY] qbase_err=%.6e  q1_err=%.6e  q2_err=%.6e\r\n",
           qbase_err, q1_err, q2_err);

    printf("[EKF_VERIFY] m_err=%.6e  p_err=%.6e\r\n", m_err, p_err);

    constexpr double kQVerifyThreshold = 1.0e-12;  // 块累加顺序不同，允许略大舍入
    bool q_fail = (qbase_err > kQVerifyThreshold || q1_err > kQVerifyThreshold || q2_err > kQVerifyThreshold);
    bool ekf_fail = (m_err > kQVerifyThreshold || p_err > kQVerifyThreshold);

    if (q_fail || ekf_fail)
    {
        printf("[VERIFY] *** WARNING: structured differs from dense reference ***\r\n");
    }
    else
    {
        printf("[VERIFY] all checks pass (err < 1e-12)\r\n");
    }
}

#endif // 验证开关

void Aided_INS::InsPropagation(const IMU &imuPre, IMU &imuCur)
{
    // [PROFILE] InsPropagation 总耗时起点
    const uint64_t t_prop_start = SystemPort_GetMicros();

    //对当前IMU数据(imuCur)补偿误差, 上一IMU数据(imupre)已经补偿过了
    ImuCompensate(imuCur);

    // [PROFILE] INS_Mech 起点
    const uint64_t t_before_mech = SystemPort_GetMicros();
    //IMU状态更新(机械编排算法)
    INS_Mechanization::INS_Mech(pvaPre_, pvaCur_, imuPre, imuCur);
    // [PROFILE] INS_Mech 终点，Phi/F/Q/G 构造起点
    const uint64_t t_after_mech = SystemPort_GetMicros();

    //系统噪声传播，姿态误差采用phi角误差模型（固定尺寸 scratch buffer，避免 malloc）
    StateMatrix &Phi = Phi_scratch_;
    StateMatrix &F   = F_scratch_;
    StateMatrix &Q   = Q_scratch_;

    //初始化Phi阵(状态转移矩阵)，F阵，Q阵(传播噪声阵)
    Phi.setIdentity();
    F.setZero();
    Q.setZero();
    // [PROFILE] 矩阵创建+初始化终点，alc 子段
    const uint64_t t_after_alc = SystemPort_GetMicros();

    //使用上一历元状态计算状态转移矩阵F(k-1)
    //计算地理参数，地球自转角速度投影到n系，n系相对于e系转动角速度投影到n系，重力加速度，子午圈半径，卯酉圈半径，加速度（fb），转动角速度
    const Vector3d w_ie_n = Earth::w_ie_n(pvaPre_.pos[0]);
    const Vector3d w_en_n = Earth::w_en_n(pvaPre_.pos, pvaPre_.vel);
    const double gravity = Gravity(pvaPre_.pos);
    const Vector2d RM_RN = RM_And_RN(pvaPre_.pos[0]);
    const double RM_h = RM_RN[0] + pvaPre_.pos[2];
    const double RN_h = RM_RN[1] + pvaPre_.pos[2];
    const Vector3d f_b = imuCur.deltaVel / imuCur.dt;
    const Vector3d w_ib_b = imuCur.deltaTheta / imuCur.dt;

    //位置误差
    Matrix3d temp;
    temp.setZero();
    temp(0, 0) =  -pvaPre_.vel[2] / RM_h;
    temp(0, 2) =   pvaPre_.vel[0] / RM_h;
    temp(1, 0) =   pvaPre_.vel[1] * tan(pvaPre_.pos[0]) / RN_h;
    temp(1, 1) = -(pvaPre_.vel[2] + pvaPre_.vel[0] * tan(pvaPre_.pos[0])) / RN_h;
    temp(1, 2) =   pvaPre_.vel[1] / RN_h;
    F.block(P_ID, P_ID, 3, 3) = temp;
    F.block(P_ID, V_ID, 3, 3) = Matrix3d::Identity();

    //速度误差
    temp.setZero();
    temp(0, 0) = -2.0 * pvaPre_.vel[1] * WIE * cos(pvaPre_.pos[0]) / RM_h -
                         pow(pvaPre_.vel[1], 2) / (RM_h * RN_h * pow(cos(pvaPre_.pos[0]), 2));
    temp(0, 2) = pvaPre_.vel[0] * pvaPre_.vel[2] / (RM_h * RM_h) - pow(pvaPre_.vel[1], 2) * tan(pvaPre_.pos[0]) / (RN_h * RN_h);
    temp(1, 0) = 2 * WIE * (pvaPre_.vel[0] * cos(pvaPre_.pos[0]) - pvaPre_.vel[2] * sin(pvaPre_.pos[0])) / RM_h +
                         pvaPre_.vel[0] * pvaPre_.vel[1] / (RM_h * RN_h * pow(cos(pvaPre_.pos[0]), 2));
    temp(1, 2) = (pvaPre_.vel[1] * pvaPre_.vel[2] + pvaPre_.vel[0] * pvaPre_.vel[1] * tan(pvaPre_.pos[0])) / (RN_h * RN_h);
    temp(2, 0) = 2 * WIE * pvaPre_.vel[1] * sin(pvaPre_.pos[0]) / RM_h;
    temp(2, 2) = -pow(pvaPre_.vel[1], 2) / (RN_h * RN_h) - pow(pvaPre_.vel[0], 2) / (RM_h * RM_h) +
                         2 * gravity / (sqrt(RM_RN[0] * RM_RN[1]) + pvaPre_.pos[2]);
    F.block(V_ID, P_ID, 3, 3) = temp;
    temp.setZero();
    temp(0, 0) = pvaPre_.vel[2] / RM_h;
    temp(0, 1) = -2 * (WIE * sin(pvaPre_.pos[0]) + pvaPre_.vel[1] * tan(pvaPre_.pos[0]) / RN_h);
    temp(0, 2) = pvaPre_.vel[0] / RM_h;
    temp(1, 0) = 2 * WIE * sin(pvaPre_.pos[0]) + pvaPre_.vel[1] * tan(pvaPre_.pos[0]) / RN_h;
    temp(1, 1) = (pvaPre_.vel[2] + pvaPre_.vel[0] * tan(pvaPre_.pos[0])) / RN_h;
    temp(1, 2) = 2 * WIE * cos(pvaPre_.pos[0]) + pvaPre_.vel[1] / RN_h;
    temp(2, 0) = -2 * pvaPre_.vel[0] / RM_h;
    temp(2, 1) = -2 * (WIE * cos(pvaPre_.pos[0]) + pvaPre_.vel[1] / RN_h);
    F.block(V_ID, V_ID, 3, 3)   = temp;
    F.block(V_ID, PHI_ID, 3, 3) = SkewSymmetric(pvaPre_.att.Cbn * f_b);
    F.block(V_ID, AB_ID, 3, 3)  = pvaPre_.att.Cbn;
    F.block(V_ID, AS_ID, 3, 3)  = pvaPre_.att.Cbn * (f_b.asDiagonal());

    //姿态误差
    temp.setZero();
    temp(0, 0) = -WIE * sin(pvaPre_.pos[0]) / RM_h;
    temp(0, 2) =  pvaPre_.vel[1] / (RN_h * RN_h);
    temp(1, 2) = -pvaPre_.vel[0] / (RM_h * RM_h);
    temp(2, 0) = -WIE * cos(pvaPre_.pos[0]) / RM_h - pvaPre_.vel[1] / (RM_h * RN_h * pow(cos(pvaPre_.pos[0]), 2));
    temp(2, 2) = -pvaPre_.vel[1] * tan(pvaPre_.pos[0]) / (RN_h * RN_h);
    F.block(PHI_ID, P_ID, 3, 3) = temp;
    temp.setZero();
    temp(0, 1) =  1 / RN_h;
    temp(1, 0) = -1 / RM_h;
    temp(2, 1) = -tan(pvaPre_.pos[0]) / RN_h;
    F.block(PHI_ID, V_ID, 3, 3)   = temp;
    F.block(PHI_ID, PHI_ID, 3, 3) = -SkewSymmetric(w_ie_n + w_en_n);
    F.block(PHI_ID, GB_ID, 3, 3)  = -pvaPre_.att.Cbn;
    F.block(PHI_ID, GS_ID, 3, 3)  = -pvaPre_.att.Cbn * (w_ib_b.asDiagonal());

    //IMU零偏误差和比例因子误差，建模成一阶高斯-马尔科夫过程
    F.block(GB_ID, GB_ID, 3, 3) = -1 / config_.imuNoise.corr_time * Matrix3d::Identity();
    F.block(AB_ID, AB_ID, 3, 3) = -1 / config_.imuNoise.corr_time * Matrix3d::Identity();
    F.block(GS_ID, GS_ID, 3, 3) = -1 / config_.imuNoise.corr_time * Matrix3d::Identity();
    F.block(AS_ID, AS_ID, 3, 3) = -1 / config_.imuNoise.corr_time * Matrix3d::Identity();

    //状态转移矩阵
    Phi.setIdentity();
    Phi = Phi + F * imuCur.dt;
    // [PROFILE] F 填充+Phi 更新终点，fill 子段
    const uint64_t t_after_fill = SystemPort_GetMicros();

    //计算系统传播噪声矩阵Q(k)（结构化 BuildGQGt + BuildPhiQbasePhiT）
    StateMatrix &QbasePre = Qbase_pre_scratch_;
    StateMatrix &QbaseCur = Qbase_cur_scratch_;
    BuildGQGt(pvaPre_.att.Cbn, QbasePre);
    BuildPhiQbasePhiT(Phi, QbasePre, Q);
    // [PROFILE] 第一次 Q 终点，q1 子段（含 BuildGQGt(pre) + BuildPhiQbasePhiT）
    const uint64_t t_after_q1 = SystemPort_GetMicros();

    BuildGQGt(pvaCur_.att.Cbn, QbaseCur);
    Q = (Q + QbaseCur) * imuCur.dt * 0.5;

    // [PROFILE] Phi/F/Q/G 构造终点，EKFPredict 起点
    const uint64_t t_before_ekf = SystemPort_GetMicros();
    //EKF预测传播系统误差状态和系统协方差
    EKFPredict(Phi, Q);
    // [PROFILE] EKFPredict 终点
    const uint64_t t_after_ekf = SystemPort_GetMicros();

    // [PROFILE] 写入 InsPropagation 分段耗时
    profile_.ins_propagation_us = static_cast<uint32_t>(t_after_ekf - t_prop_start);
    profile_.ins_mech_us        = static_cast<uint32_t>(t_after_mech - t_before_mech);
    profile_.phi_f_q_g_us       = static_cast<uint32_t>(t_before_ekf - t_after_mech);
    profile_.ekf_predict_us     = static_cast<uint32_t>(t_after_ekf - t_before_ekf);
    // [PROFILE] Phase-2: fmx 子段
    profile_.fmx_alc_us  = static_cast<uint32_t>(t_after_alc  - t_after_mech);
    profile_.fmx_fill_us = static_cast<uint32_t>(t_after_fill - t_after_alc);
    profile_.fmx_q1_us   = static_cast<uint32_t>(t_after_q1   - t_after_fill);
    profile_.fmx_q2_us   = static_cast<uint32_t>(t_before_ekf - t_after_q1);
}

void Aided_INS::EKFPredict(const StateMatrix &Phi, const StateMatrix &Q)
{
    assert(Phi.rows() == P_.rows());
    assert(Q.rows() == P_.rows());

    /**
     * 经典教材公式（Kalman 预测步）：
     *     dx = Phi * dx
     *     P  = Phi * P * Phi^T + Q
     *
     * Stage 3a 实现（保留第二阶段 M*Phi^T 为稠密乘法）：
     *     M  = Phi * P        // BuildPhiTimesP：利用 Phi 的 3×3 块稀疏结构
     *     P  = M * Phi^T + Q  // 第二阶段暂保留 Eigen 稠密表达式
     *
     * BuildPhiTimesP() 基于 Phi 的固有块稀疏结构（16 个非零 3×3 块），
     * 不假设 P 稀疏，不改变数学结果。
     */
    dx_ = Phi * dx_;

    StateMatrix &M = M_scratch_;
    BuildPhiTimesP(Phi, P_, M);

    P_ = M * Phi.transpose() + Q;
}

bool Aided_INS::AccUpdate(const IMU& imuData, const PVA& pvaCur, const Config& config)
{
    const Vector3d f_b = imuData.deltaVel / imuData.dt;
    const double gravity = Gravity(pvaCur.pos); //当地重力加速度
    const Vector3d g_l_n{0, 0, gravity}; //当地重力加速度投影在n系
    const Vector3d g_b_ByImu = pvaCur.att.Cbn.transpose() * g_l_n; //IMU测量到的重力加速度（假设没有运动加速度）。g_b_ByImu = Cnb * g_l_n

    if (f_b.norm() > 1e-6f && g_b_ByImu.norm() > 1e-6f)
    {
        const double cos_gn_gb = f_b.dot(g_b_ByImu) / ( f_b.norm() * g_b_ByImu.norm() ); //重力测量值与理论值的夹角，如果不是1，说明测量值与理论值方向不重合

        if ( std::fabs(f_b.norm() - gravity) < 0.8 && cos_gn_gb < -0.95) // 比力方向与重力方向相反；运动加速度过大则不用加速度计更新姿态
        {
            //构造输入加速度计观测方程的测量误差
            const Vector3d f_b_ByImu = -g_b_ByImu; //IMU测量到的重力加速度（假设没有运动加速度）产生的比力
            const MeasurementVector<3> dz = f_b_ByImu - f_b;

            //构造加速度计观测矩阵
            MeasurementMatrix<3> H_acc;
            H_acc.setZero();
            H_acc.template block<3, 3>(0, PHI_ID) = SkewSymmetric(-f_b_ByImu);
            H_acc.template block<3, 3>(0, AB_ID)  = -Matrix3d::Identity();
            H_acc.template block<3, 3>(0, AS_ID)  = (-f_b_ByImu).asDiagonal();

            //加速度计观测噪声矩阵
            const MeasurementNoise<3> R_acc = (config.imuNoise.accVrw.cwiseProduct(config.imuNoise.accVrw)  / imuData.dt).asDiagonal();

            // EKF更新协方差和误差状态
            EkfUpdate<3>(dz, H_acc, R_acc);

            return true;
        }
    }

    return false;
}

void Aided_INS::MagUpdate(const Mag &magData, const PVA &pvaCur, const Config &config)
{
    /**
     *认为roll、pitch是准确的，认为pvaCur的估计结果只存在yaw误差，所以先将磁力计测量值转到n系，
     *此时，hx_n、hy_n理论上就是地磁的水平分量。如果无磁偏角、yaw误差，hy_n应该为0，所以
     *hy_n、hx_n形成的夹角就是磁偏角与yaw误差的总和。
     */
    Vector3d h_n = pvaCur.att.Cbn * magData.mag;

    //构造输入磁力计观测方程的测量误差（先不修正磁偏角）
    MeasurementVector<1> dz;
    dz(0, 0) = -atan2(-h_n(1), h_n(0)); //磁力计测得的航向误差 = 偏航角误差 + 磁偏角

    //构造磁力计观测矩阵
    MeasurementMatrix<1> H_mag;
    H_mag.setZero();
    H_mag(0, PHI_ID+2) = 1;

    //磁力计观测噪声矩阵
    MeasurementNoise<1> R_mag;
    R_mag(0, 0) = config.magMeasureYawStd * config.magMeasureYawStd;

    // EKF更新协方差和误差状态
    EkfUpdate<1>(dz, H_mag, R_mag);

    //磁力计更新之后设置为不可用
    magData_.isUpdate = false;
}

void Aided_INS::GnssUpdate(GNSS &gnssData)
{
    //IMU位置转到GNSS天线相位中心位置
    const Matrix3d Dr_inv = DR_Inv(pvaCur_.pos);
    const Matrix3d Dr = DR(pvaCur_.pos);
    const Vector3d antennaPosByImu = pvaCur_.pos + Dr_inv * pvaCur_.att.Cbn * config_.antennaLever;

    //GNSS位置测量新息
    const MeasurementVector<3> dz = Dr * (antennaPosByImu - gnssData.blh);

    //构造GNSS位置观测矩阵
    MeasurementMatrix<3> H_gnssPos;
    H_gnssPos.setZero();
    H_gnssPos.template block<3, 3>(0, P_ID)   = Matrix3d::Identity();
    H_gnssPos.template block<3, 3>(0, PHI_ID) = SkewSymmetric(pvaCur_.att.Cbn * config_.antennaLever);

    //位置观测噪声阵
    const MeasurementNoise<3> R_gnssPos = gnssData.std.cwiseProduct(gnssData.std).asDiagonal();

    //EKF更新协方差和误差状态
    EkfUpdate<3>(dz, H_gnssPos, R_gnssPos);

    //GNSS更新之后设置为不可用
    gnssData.isUpdate = false;
}

template<int MeasDim>
void Aided_INS::EkfUpdate(const MeasurementVector<MeasDim> &dz, const MeasurementMatrix<MeasDim> &H,
                           const MeasurementNoise<MeasDim> &R)
{
    assert(H.cols()  == kStateRank);
    assert(dz.rows() == MeasDim);
    assert(R.rows() == MeasDim);
    assert(dz.cols() == 1);

    //计算Kalman增益：K(state×meas) = P * H^T * inv(H * P * H^T + R)
    const Eigen::Matrix<double, MeasDim, MeasDim> temp = H * P_ * H.transpose() + R;
    const Eigen::Matrix<double, kStateRank, MeasDim> K = P_ * H.transpose() * temp.inverse();

    //更新系统误差状态和协方差（I_scratch_ 复用，避免 21×21 栈分配）
    StateMatrix &I = I_scratch_;
    I.setIdentity();
    I -= K * H;
    //如果每次更新后都进行状态反馈，则更新前dx_一直为0，下式可以简化为：dx_ = K * dz;
    const MeasurementVector<MeasDim> innovation = dz - H * dx_;
    dx_ += K * innovation;

    // P_ 出现在右边，不能对 P_ 用 noalias；用局部 StateMatrix 做新协方差再写回
    StateMatrix P_new;
    P_new.noalias() = I * P_ * I.transpose();
    P_new.noalias() += K * R * K.transpose();
    P_ = P_new;
}

// 显式实例化：当前用到的测量维度
template void Aided_INS::EkfUpdate<1>(const MeasurementVector<1>&, const MeasurementMatrix<1>&,
                                       const MeasurementNoise<1>&);
template void Aided_INS::EkfUpdate<3>(const MeasurementVector<3>&, const MeasurementMatrix<3>&,
                                       const MeasurementNoise<3>&);

void Aided_INS::StateFeedback()
{
    //位置误差反馈
    Vector3d vecTemp = dx_.block(P_ID, 0, 3, 1); //delta_r位置误差
    pvaCur_.pos -= DR_Inv(pvaCur_.pos) * vecTemp;

    //速度误差反馈
    vecTemp = dx_.block(V_ID, 0, 3, 1); //delta_v速度误差
    pvaCur_.vel -= vecTemp;

    //姿态误差反馈
    vecTemp               = dx_.block(PHI_ID, 0, 3, 1); //phi姿态失准角小角度下可近似为等效旋转矢量phi_n(k)_p(k)，p系为估计得到的导航系，n系为真导航系
    const Quaterniond qpn = RotVec2Quaternion(vecTemp); //q_p(k)_n(k)
    pvaCur_.att.qbn       = (qpn * pvaCur_.att.qbn).normalized(); //q_b(k)_n(k) = q_p(k)_n(k) * q_b(k)_p(k)
    pvaCur_.att.Cbn       = Quaternion2DCM(pvaCur_.att.qbn);
    pvaCur_.att.euler     = DCM2Euler(pvaCur_.att.Cbn);

    //IMU零偏误差反馈
    vecTemp = dx_.block(GB_ID, 0, 3, 1);
    imuError_.gyrBias += vecTemp;
    vecTemp = dx_.block(AB_ID, 0, 3, 1);
    imuError_.accBias += vecTemp;

    //IMU比例因子误差反馈
    vecTemp = dx_.block(GS_ID, 0, 3, 1);
    imuError_.gyrScale += vecTemp;
    vecTemp = dx_.block(AS_ID, 0, 3, 1);
    imuError_.accScale += vecTemp;

    // 误差状态反馈到系统状态后,将误差状态清零
    dx_.setZero();
}

Aided_INS::KfUpdateType Aided_INS::IsToUpdate(const double imuTime1, const double imuTime2,
                                              const double updateTime) const
{
    if (std::fabs(imuTime1 - updateTime) < TIME_ALIGN_ERR_)
        return KfUpdateType::Prev; //更新时间靠近imuTime1

    if (std::fabs(imuTime2 - updateTime) <= TIME_ALIGN_ERR_)
        return KfUpdateType::Curr; //更新时间靠近imuTime2

    if (imuTime1 < updateTime && updateTime < imuTime2)
        return KfUpdateType::Middle; //更新时间在imuTime1和imTime2之间, 但不靠近任何一个

    return KfUpdateType::None; //更新时间不在imuTime1和imuTime2之间，且不靠近任何一个
}

void Aided_INS::ProcessNewData()
{
    // [PROFILE] ProcessNewData 总耗时起点
    const uint64_t t_pnd_start = SystemPort_GetMicros();

    //当前IMU时间作为系统当前状态时间
    timestamp_ = imuCur_.time;

    //如果磁力计有效，则将更新时间设置为磁力计采样时间
    const double updateTime = magData_.isUpdate ? magData_.time : -1;

    //判断是否需要进行KF磁力计更新
    const KfUpdateType res = IsToUpdate(imuPre_.time, imuCur_.time, updateTime);

    if ( res == KfUpdateType::None)
    {
        //只传播导航状态、加速度计更新
        InsPropagation(imuPre_, imuCur_);

        // [PROFILE] AccUpdate + StateFeedback 合计起点
        const uint64_t t_after_prop = SystemPort_GetMicros();

        if (AccUpdate(imuCur_, pvaCur_, config_))
        {
            StateFeedback();
        }

        // [PROFILE] 写入 acc_feedback_us
        profile_.acc_feedback_us = static_cast<uint32_t>(SystemPort_GetMicros() - t_after_prop);
    }
    else if (res == KfUpdateType::Prev)
    {
        //磁力计数据靠近上一历元，先对上一历元进行磁力计更新
        MagUpdate(magData_, pvaCur_, config_);
        StateFeedback();

        pvaPre_ = pvaCur_;
        InsPropagation(imuPre_, imuCur_);
        if (AccUpdate(imuCur_, pvaCur_, config_))
        {
            StateFeedback();
        }
    }
    else if (res == KfUpdateType::Curr)
    {
        //磁力计数据靠近当前历元，先对当前IMU进行状态传播
        InsPropagation(imuPre_, imuCur_);
        if (AccUpdate(imuCur_, pvaCur_, config_))
        {
            StateFeedback();
        }
        MagUpdate(magData_, pvaCur_, config_);
        StateFeedback();
    }
    else
    {
        //磁力计数据在两个IMU数据之间(不靠近任何一个), 将当前IMU内插到磁力计采样时刻
        IMU imuMiddle;
        imuInterpolate(imuPre_, imuCur_, updateTime, imuMiddle);
        // NOTE：内插之后采样间隔会变化，严格上不满足INSMech的假设，但影响较小暂时忽略

        //对前一半IMU进行状态传播
        InsPropagation(imuPre_, imuMiddle);
        if (AccUpdate(imuMiddle, pvaCur_, config_))
        {
            StateFeedback();
        }

        //进行磁力计更新，并反馈系统状态
        MagUpdate(magData_, pvaCur_, config_);
        StateFeedback();

        //对后一半IMU进行状态传播
        pvaPre_ = pvaCur_;
        InsPropagation(imuMiddle, imuCur_);
        if (AccUpdate(imuCur_, pvaCur_, config_))
        {
            StateFeedback();
        }
    }

    //更新上一时刻的状态和IMU数据
    pvaPre_ = pvaCur_;
    imuPre_ = imuCur_;

    // [PROFILE] ProcessNewData 总耗时终点
    profile_.process_new_data_us = static_cast<uint32_t>(SystemPort_GetMicros() - t_pnd_start);
}

// ============================================================================
// 结构化传播辅助函数
// ============================================================================
//
// 下列 helper 是对教材/论文中 Kalman 传播公式的等价块结构实现。
//
// 原始公式：
//   Qbase = G * q * G^T
//   Q     = (Phi * Qbase_pre * Phi^T + Qbase_cur) * dt / 2
//   P     = Phi * P * Phi^T + Q
//
// 为降低 MCU 上 21×18 / 21×21 稠密矩阵乘法开销，当前实现没有在运行时完整构造
// G，也没有直接调用所有稠密乘法，而是利用 G、Phi、Qbase 的固定 3×3 块结构。
// 这些 helper 只改变计算组织形式，不改变数学模型。
//
// P 仍按满矩阵处理；目前只结构化计算 Phi * P，第二步 M * Phi^T 仍保留稠密计算。
// ============================================================================

// --- BuildGQGt / BuildGQGtFromNoise -----------------------------------------
//
// 原始公式：Qbase = G * q * G^T
//
// G 的固定非零块（kNoiseRank=18, kStateRank=21）：
//   G(V,   VRW)   = Cbn          G(PHI, ARW)   = Cbn
//   G(GB,  GBSTD) = I            G(AB,  ABSTD) = I
//   G(GS,  GSSTD) = I            G(AS,  ASSTD) = I
//   其余全为零。
//
// q 本身是块对角矩阵（6 个 3×3 对角块）。
// 因为 G 的非零块和 q 的块对角结构，G*q*G^T 仅以下 6 个主对角状态块非零：
//   Qbase(V,V)     = Cbn * q(VRW,VRW) * Cbn^T
//   Qbase(PHI,PHI) = Cbn * q(ARW,ARW) * Cbn^T
//   Qbase(GB,GB)   = q(GBSTD,GBSTD)    (及 AB, GS, AS 同理)
//
// 其余 21×21 元素全为零。因此可以直接填充这 6 个块，跳过完整 G 的构造和稠密乘法。
// ----------------------------------------------------------------------------

void Aided_INS::BuildGQGtFromNoise(const Matrix3d &Cbn, const NoiseMatrix &q, StateMatrix &Qbase)
{
    Qbase.setZero();

    Qbase.template block<3, 3>(V_ID, V_ID) =
        Cbn * q.template block<3, 3>(VRW_ID, VRW_ID) * Cbn.transpose();

    Qbase.template block<3, 3>(PHI_ID, PHI_ID) =
        Cbn * q.template block<3, 3>(ARW_ID, ARW_ID) * Cbn.transpose();

    Qbase.template block<3, 3>(GB_ID, GB_ID) = q.template block<3, 3>(GBSTD_ID, GBSTD_ID);
    Qbase.template block<3, 3>(AB_ID, AB_ID) = q.template block<3, 3>(ABSTD_ID, ABSTD_ID);
    Qbase.template block<3, 3>(GS_ID, GS_ID) = q.template block<3, 3>(GSSTD_ID, GSSTD_ID);
    Qbase.template block<3, 3>(AS_ID, AS_ID) = q.template block<3, 3>(ASSTD_ID, ASSTD_ID);
}

void Aided_INS::BuildGQGt(const Matrix3d &Cbn, StateMatrix &Qbase) const
{
    BuildGQGtFromNoise(Cbn, q_, Qbase);
}

// --- AccumulatePhiQbaseColumn / BuildPhiQbasePhiT ----------------------------
//
// 原始公式：Q = Phi * Qbase * Phi^T
//
// 因为 Qbase 仅有 6 个主对角 3×3 块非零（见 BuildGQGtFromNoise），推导如下：
//   Q(i,j) = Σ_k Σ_l Phi(i,k) * Qbase(k,l) * Phi(j,l)^T
// 由于 Qbase(k,l) 仅在 k=l 且 k∈{V,PHI,GB,AB,GS,AS} 时非零，退化为：
//   Q(i,j) = Σ_{k∈非零列} Phi(i,k) * Qbase(k,k) * Phi(j,k)^T
//
// 这意味着结构化的 Q 是按列分块累加的：对于每个 k，将贡献加到所有 (i,j) 对上。
// 注意：不能只填 Q 的主对角块——Phi 的非对角块会把噪声传播到交叉协方差，
// 所以必须遍历 Phi(*,k) 非零的 i 行和 j 行。
// ----------------------------------------------------------------------------

void Aided_INS::AccumulatePhiQbaseColumn(const StateMatrix &Phi, const StateMatrix &Qbase,
                                          const int k_id, const int *rows, const int row_count,
                                          StateMatrix &Qout)
{
    const Eigen::Matrix3d Qkk = Qbase.template block<3, 3>(k_id, k_id);

    for (int a = 0; a < row_count; ++a)
    {
        const int i_id = rows[a];

        Eigen::Matrix3d tmp;
        tmp.noalias() = Phi.template block<3, 3>(i_id, k_id) * Qkk;

        for (int b = 0; b < row_count; ++b)
        {
            const int j_id = rows[b];

            Qout.template block<3, 3>(i_id, j_id).noalias() +=
                tmp * Phi.template block<3, 3>(j_id, k_id).transpose();
        }
    }
}

void Aided_INS::BuildPhiQbasePhiT(const StateMatrix &Phi, const StateMatrix &Qbase, StateMatrix &Qout)
{
    Qout.setZero();

    const int rows_v[]   = {P_ID,   V_ID,   PHI_ID};
    const int rows_phi[] = {V_ID,   PHI_ID};
    const int rows_gb[]  = {PHI_ID, GB_ID};
    const int rows_ab[]  = {V_ID,   AB_ID};
    const int rows_gs[]  = {PHI_ID, GS_ID};
    const int rows_as[]  = {V_ID,   AS_ID};

    AccumulatePhiQbaseColumn(Phi, Qbase, V_ID,   rows_v,   3, Qout);
    AccumulatePhiQbaseColumn(Phi, Qbase, PHI_ID, rows_phi, 2, Qout);
    AccumulatePhiQbaseColumn(Phi, Qbase, GB_ID,  rows_gb,  2, Qout);
    AccumulatePhiQbaseColumn(Phi, Qbase, AB_ID,  rows_ab,  2, Qout);
    AccumulatePhiQbaseColumn(Phi, Qbase, GS_ID,  rows_gs,  2, Qout);
    AccumulatePhiQbaseColumn(Phi, Qbase, AS_ID,  rows_as,  2, Qout);
}

// --- AccumulatePhiPRow / BuildPhiTimesP -------------------------------------
//
// 原始公式（EKFPredict 第一步）：M = Phi * P
//
// 利用 Phi 的固定 3×3 块稀疏结构（仅 16 个非零 3×3 块，详见 InsPropagation 中 F 填充逻辑）。
// 对每个状态行块 i：
//   M(i,:) = Σ_{k∈非零列} Phi(i,k) * P(k,:)
//
// 注意：P 是协方差矩阵，一般不能假设稀疏；本函数不假设 P 稀疏，
// 只是跳过 Phi 中确定为零的列块，不改变数学结果。
// ----------------------------------------------------------------------------

void Aided_INS::AccumulatePhiPRow(const StateMatrix &Phi, const StateMatrix &P,
                                   const int row_id, const int *cols, const int col_count,
                                   StateMatrix &Mout)
{
    for (int c = 0; c < col_count; ++c)
    {
        const int k_id = cols[c];
        Mout.template block<3, 21>(row_id, 0).noalias() +=
            Phi.template block<3, 3>(row_id, k_id) * P.template block<3, 21>(k_id, 0);
    }
}

void Aided_INS::BuildPhiTimesP(const StateMatrix &Phi, const StateMatrix &P, StateMatrix &Mout)
{
    Mout.setZero();

    const int cols_P[]   = {P_ID,   V_ID};
    const int cols_V[]   = {P_ID,   V_ID,   PHI_ID, AB_ID, AS_ID};
    const int cols_PHI[] = {P_ID,   V_ID,   PHI_ID, GB_ID, GS_ID};
    const int cols_GB[]  = {GB_ID};
    const int cols_AB[]  = {AB_ID};
    const int cols_GS[]  = {GS_ID};
    const int cols_AS[]  = {AS_ID};

    AccumulatePhiPRow(Phi, P, P_ID,   cols_P,   2, Mout);
    AccumulatePhiPRow(Phi, P, V_ID,   cols_V,   5, Mout);
    AccumulatePhiPRow(Phi, P, PHI_ID, cols_PHI, 5, Mout);
    AccumulatePhiPRow(Phi, P, GB_ID,  cols_GB,  1, Mout);
    AccumulatePhiPRow(Phi, P, AB_ID,  cols_AB,  1, Mout);
    AccumulatePhiPRow(Phi, P, GS_ID,  cols_GS,  1, Mout);
    AccumulatePhiPRow(Phi, P, AS_ID,  cols_AS,  1, Mout);
}
