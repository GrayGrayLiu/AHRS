/******************************************************************************
 * @file    INS_Mechanization.cpp
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

#include "INS_Mechanization.hpp"
#include "RotationUtilities.hpp"
#include "EarthUtilities.hpp"

using namespace Earth;
using namespace Rotation;

void INS_Mechanization::INS_Mech(const PVA &PVA_Pre, PVA &PVA_Cur, const IMU &IMU_Pre, const IMU &IMU_Cur)
{
    //依次进行速度更新、位置更新、姿态更新, 不可调换顺序
    VelUpdate(PVA_Pre, PVA_Cur, IMU_Pre, IMU_Cur);
    PosUpdate(PVA_Pre, PVA_Cur, IMU_Pre, IMU_Cur);
    AttUpdate(PVA_Pre, PVA_Cur, IMU_Pre, IMU_Cur);
}

void INS_Mechanization::VelUpdate(const PVA &PVA_Pre, PVA &PVA_Cur, const IMU &IMU_Pre, const IMU &IMU_Cur)
{
    //计算地理参数，地球自转角速度投影到n系, n系相对于e系转动角速度投影到n系
    Vector3d w_ie_n = Earth::w_ie_n(PVA_Pre.pos[0]);
    Vector3d w_en_n = Earth::w_en_n(PVA_Pre.pos, PVA_Pre.vel);

    //b系比力积分项
    const Vector3d dv_f_b = IMU_Cur.deltaVel +
                            IMU_Cur.deltaTheta.cross(IMU_Cur.deltaVel) * 0.5 +          //旋转效应
                            IMU_Pre.deltaTheta.cross(IMU_Cur.deltaVel) * (1.0 / 12.0) +
                            IMU_Pre.deltaVel.cross(IMU_Cur.deltaTheta) * (1.0 / 12.0);  //双子样划桨效应(基于相同采样间隔推导)

    //比力积分项投影到n系。此处理论上应该用(w_ie_n + w_en_n)在中间时刻（k-1/2）的角速度，但目前因为尚无法获得k时刻的w_en_n，且这一项产生的误差小，所以使用k-1 时刻的量近似
    Vector3d phi_nk1_nk = (w_ie_n + w_en_n) * IMU_Cur.dt; //等效旋转矢量phi_n(k-1)_n(k)
    Matrix3d C_nk1_nk = Matrix3d::Identity() - SkewSymmetric(phi_nk1_nk) * 0.5; //C_n(k-1)_n(k)
    Vector3d dv_f_n = C_nk1_nk * PVA_Pre.att.Cbn * dv_f_b;

    //计算重力/哥式积分项
    double gravity = Gravity(PVA_Pre.pos);
    Vector3d g_l_n{0, 0, gravity}; //当地重力加速度在n系的投影
    Vector3d dv_g_n = (g_l_n - (2.0 * w_ie_n + w_en_n).cross(PVA_Pre.vel)) * IMU_Cur.dt;

    // 得到中间时刻（k-1/2）速度
    Vector3d velMiddle = PVA_Pre.vel + (dv_f_n + dv_g_n) * 0.5;

    // 外推得到中间时刻（k-1/2）位置
    const Vector3d phi_nk1_nk12 = (w_ie_n + w_en_n) * IMU_Cur.dt * 0.5; //phi_n(k-1)_n(k-1/2)。 w_ie_n + w_en_n近似为常数，对等效旋转矢量做内插
    const Quaterniond q_nk12_nk1 = RotVec2Quaternion(phi_nk1_nk12); //q_n(k-1/2)_n(k-1)。
    const Vector3d phi_ek12_ek1{0, 0, -WIE * IMU_Cur.dt * 0.5}; //phi_e(k-1/2)_e(k-1)，e系转动等效旋转矢量，因为WIE为常数，所以时间可以平移。
    const Quaterniond q_ek1_ek12 = RotVec2Quaternion(phi_ek12_ek1); //q_e(k-1)_e(k-1/2)
    const Quaterniond q_nk1_ek1 = qne(PVA_Pre.pos); //q_n(k-1)_e(k-1)
    const Quaterniond q_nk12_ek12 = (q_ek1_ek12 * q_nk1_ek1 * q_nk12_nk1).normalized(); //q_n(k-1/2)_e(k-1/2) = q_e(k-1)_e(k-1/2) * q_n(k-1)_e(k-1) * q_n(k-1/2)_n(k-1)
    Vector3d posMiddle;
    posMiddle[2] = PVA_Pre.pos[2] - velMiddle[2] * IMU_Cur.dt / 2;
    posMiddle    = blh(q_nk12_ek12, posMiddle[2]);

    // 重新计算中间时刻（k-1/2）的w_ie_n, w_en_n
    w_ie_n = Earth::w_ie_n(posMiddle[0]);
    w_en_n = Earth::w_en_n(posMiddle, velMiddle);

    // 重新计算n系下平均比力积分项
    phi_nk1_nk = (w_ie_n + w_en_n) * IMU_Cur.dt; //等效旋转矢量phi_n(k-1)_n(k)
    C_nk1_nk   = Matrix3d::Identity() - SkewSymmetric(phi_nk1_nk) * 0.5; //C_n(k-1)_n(k)
    dv_f_n = C_nk1_nk * PVA_Pre.att.Cbn * dv_f_b;

    // 重新计算重力、哥式积分项
    gravity = Gravity(posMiddle);
    g_l_n << 0, 0, gravity;
    dv_g_n = (g_l_n - (2.0 * w_ie_n + w_en_n).cross(velMiddle)) * IMU_Cur.dt;

    // 速度更新完成
    PVA_Cur.vel = PVA_Pre.vel + dv_f_n + dv_g_n;
}

void INS_Mechanization::PosUpdate(const PVA &PVA_Pre, PVA &PVA_Cur, const IMU &IMU_Pre, const IMU &IMU_Cur)
{
    // 重新计算中间时刻（k-1/2）的速度和位置
    Vector3d velMiddle = (PVA_Cur.vel + PVA_Pre.vel) * 0.5;
    Vector3d posMiddle = PVA_Pre.pos + DR_Inv(PVA_Pre.pos) * velMiddle * IMU_Cur.dt * 0.5;

    // 重新计算中间时刻（k-1/2）地理参数
    const Vector3d w_ie_n = Earth::w_ie_n(posMiddle[0]);
    const Vector3d w_en_n = Earth::w_en_n(posMiddle, velMiddle);

    // 重新计算 k 时刻到 k-1 时刻 n系旋转矢量
    const Vector3d phi_nk1_nk = (w_ie_n + w_en_n) * IMU_Cur.dt; //等效旋转矢量phi_n(k-1)_n(k)
    const Quaterniond q_nk_nk1 = RotVec2Quaternion(phi_nk1_nk); //q_n(k)_n(k-1)
    const Vector3d phi_ek_ek1{0, 0, -WIE * IMU_Cur.dt}; //phi_e(k)_e(k-1)，e系转动等效旋转矢量
    const Quaterniond q_ek1_ek = RotVec2Quaternion(phi_ek_ek1); //q_e(k-1)_e(k)

    // 位置更新完成
    const Quaterniond q_nk1_ek1 = qne(PVA_Pre.pos); //q_n(k-1)_e(k-1)
    const Quaterniond q_nk_ek   = (q_ek1_ek * q_nk1_ek1 * q_nk_nk1).normalized(); //q_n(k)_e(k) = q_e(k-1)_e(k) * q_n(k-1)_e(k-1) * q_n(k)_n(k-1)
    PVA_Cur.pos[2]  = PVA_Pre.pos[2] - velMiddle[2] * IMU_Cur.dt;
    PVA_Cur.pos     = blh(q_nk_ek, PVA_Cur.pos[2]);
}

void INS_Mechanization::AttUpdate(const PVA &PVA_Pre, PVA &PVA_Cur, const IMU &IMU_Pre, const IMU &IMU_Cur)
{
    // 重新计算中间时刻（k-1/2）的速度和位置
    const Vector3d velMiddle = (PVA_Pre.vel + PVA_Cur.vel) * 0.5;
    const Quaterniond q_nk1_ek1 = qne(PVA_Pre.pos); //q_n(k-1)_e(k-1)
    const Quaterniond q_nk_ek = qne(PVA_Cur.pos); //q_n(k)_e(k)
    const Vector3d phi_ek_ek1{0, 0, -WIE * IMU_Cur.dt}; //phi_e(k)_e(k-1)，e系转动等效旋转矢量
    const Quaterniond q_ek1_ek = RotVec2Quaternion(phi_ek_ek1); //q_e(k-1)_e(k)
    Vector3d phi_nk_nk1 = Quaternion2Vector((q_nk1_ek1.inverse() * q_ek1_ek.inverse() * q_nk_ek).normalized()); //q_n(k)_n(k-1) = q_e(k-1)_n(k-1) * q_e(k)_e(k-1) * q_n(k)_e(k)
    const Vector3d phi_ek12_ek1{0, 0, -WIE * IMU_Cur.dt * 0.5}; //phi_e(k-1/2)_e(k-1)，e系转动等效旋转矢量，因为WIE为常数，所以时间可以平移。
    const Quaterniond q_ek1_ek12 = RotVec2Quaternion(phi_ek12_ek1); //q_e(k-1)_e(k-1/2)
    const Quaterniond q_nk12_ek12 = (q_ek1_ek12 * q_nk1_ek1 * RotVec2Quaternion(phi_nk_nk1 * 0.5)).normalized(); //q_n(k-1/2)_e(k-1/2) = q_e(k-1)_e(k-1/2) * q_n(k-1)_e(k-1) * q_n(k-1/2)_n(k-1)
    Vector3d posMiddle;
    posMiddle[2] = (PVA_Cur.pos[2] + PVA_Pre.pos[2]) * 0.5;
    posMiddle    = blh(q_nk12_ek12, posMiddle[2]);

    // 重新计算中间时刻（k-1/2）的w_ie_n, w_en_n
    const Vector3d w_ie_n = Earth::w_ie_n(posMiddle[0]);
    const Vector3d w_en_n = Earth::w_en_n(posMiddle, velMiddle);

    // 计算n系的旋转四元数 k-1 时刻到k时刻变换
    phi_nk_nk1 = -(w_ie_n + w_en_n) * IMU_Cur.dt; //phi_n(k)_n(k-1)
    const Quaterniond q_nk1_nk = RotVec2Quaternion(phi_nk_nk1); //q_n(k-1)_n(k)

    // 计算b系旋转四元数 补偿二阶圆锥误差(基于相同采样间隔推导)
    const Vector3d phi_bk1_bk = IMU_Cur.deltaTheta + IMU_Pre.deltaTheta.cross(IMU_Cur.deltaTheta) * (1.0 / 12.0); //phi_b(k-1)_b(k)
    const Quaterniond q_bk_bk1 = RotVec2Quaternion(phi_bk1_bk); //q_b(k)_b(k-1)

    // 姿态更新完成
    PVA_Cur.att.qbn   = (q_nk1_nk * PVA_Pre.att.qbn * q_bk_bk1).normalized();
    PVA_Cur.att.Cbn   = Quaternion2DCM(PVA_Cur.att.qbn);
    PVA_Cur.att.euler = DCM2Euler(PVA_Cur.att.Cbn);
}
