/******************************************************************************
 * @file    Configuration.h
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

using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;

using Earth::Gravity;
using Earth::RM_And_RN;
using Earth::WIE;
using Earth::DR_Inv;

using Rotation::Euler2DCM;
using Rotation::Euler2Quaternion;
using Rotation::SkewSymmetric;
using Rotation::RotVec2Quaternion;
using Rotation::Quaternion2DCM;
using Rotation::DCM2Euler;

Aided_INS::Aided_INS(const uint8_t id)
{
    const Aided_INS_Space::Config &config = LoadConfig();
    Initialize(config);
}

int Aided_INS::Run()
{
    return 0;
}

int Aided_INS::InitialAlignment()
{
    return 0;
}

Aided_INS_Space::Config Aided_INS::LoadConfig()
{
    Aided_INS_Space::Config config = {};

    return config;
}

void Aided_INS::Initialize(const Aided_INS_Space::Config &config)
{
    this->config_ = config;
    timestamp_ = 0;

    //设置协方差矩阵Cov，系统噪声阵q和系统误差状态矩阵dx大小
    P_.resize(RANK_, RANK_);
    q_.resize(NOISE_RANK_, NOISE_RANK_);
    dx_.resize(RANK_, 1);
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

void Aided_INS::InsPropagation(const IMU &imuPre, IMU &imuCur)
{
    //对当前IMU数据(imuCur)补偿误差, 上一IMU数据(imupre)已经补偿过了
    ImuCompensate(imuCur);

    //IMU状态更新(机械编排算法)
    INS_Mechanization::INS_Mech(pvaPre_, pvaCur_, imuPre, imuCur);

    //系统噪声传播，姿态误差采用phi角误差模型
    MatrixXd Phi, F, Q, G;

    //初始化Phi阵(状态转移矩阵)，F阵，Q阵(传播噪声阵)，G阵(噪声驱动阵)
    Phi.resizeLike(P_);
    F.resizeLike(P_);
    Q.resizeLike(P_);
    G.resize(RANK_, NOISE_RANK_);
    Phi.setIdentity();
    F.setZero();
    Q.setZero();
    G.setZero();

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

    // 位置误差
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

    //系统噪声驱动矩阵G(k-1)
    G.block(V_ID, VRW_ID, 3, 3)    = pvaPre_.att.Cbn;
    G.block(PHI_ID, ARW_ID, 3, 3)  = pvaPre_.att.Cbn;
    G.block(GB_ID, GBSTD_ID, 3, 3) = Matrix3d::Identity();
    G.block(AB_ID, ABSTD_ID, 3, 3) = Matrix3d::Identity();
    G.block(GS_ID, GSSTD_ID, 3, 3) = Matrix3d::Identity();
    G.block(AS_ID, ASSTD_ID, 3, 3) = Matrix3d::Identity();

    //状态转移矩阵
    Phi.setIdentity();
    Phi = Phi + F * imuCur.dt;

    //计算系统传播噪声矩阵Q(k)
    // Q = G * q_ * G.transpose() * imuCur.dt;
    // Q = (Phi * Q * Phi.transpose() + Q) * 0.5;
    Q = Phi * G * q_ * G.transpose() * Phi.transpose();
    //用IMU预测的姿态近似得到G(k)
    G.block(V_ID, VRW_ID, 3, 3)    = pvaCur_.att.Cbn;
    G.block(PHI_ID, ARW_ID, 3, 3)  = pvaCur_.att.Cbn;
    Q = (Q + G * q_ * G.transpose()) * imuCur.dt * 0.5;

    //EKF预测传播系统误差状态和系统协方差
    EKFPredict(Phi, Q);
}

void Aided_INS::EKFPredict(const MatrixXd &Phi, const MatrixXd &Q)
{
    assert(Phi.rows() == P_.rows());
    assert(Q.rows() == P_.rows());

    //传播系统误差状态和系统协方差
    dx_ = Phi * dx_;
    P_  = Phi * P_ * Phi.transpose() + Q;
}

void Aided_INS::EkfUpdate(MatrixXd &dz, MatrixXd &H, MatrixXd &R)
{
    assert(H.cols()  == P_.rows());
    assert(dz.rows() == H.rows());
    assert(dz.rows() == R.rows());
    assert(dz.cols() == 1);

    //计算Kalman增益
    const auto temp = H * P_ * H.transpose() + R;
    MatrixXd K = P_ * H.transpose() * temp.inverse();

    //更新系统误差状态和协方差
    MatrixXd I;
    I.resizeLike(P_);
    I.setIdentity();
    I = I - K * H;
    //如果每次更新后都进行状态反馈，则更新前dx_一直为0，下式可以简化为：dx_ = K * dz;
    dx_  = dx_ + K * (dz - H * dx_);
    P_ = I * P_ * I.transpose() + K * R * K.transpose();
}

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
    pvaCur_.att.qbn       = qpn * pvaCur_.att.qbn; //q_b(k)_n(k) = q_p(k)_n(k) * q_b(k)_p(k)
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
