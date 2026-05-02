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

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;

using Earth::Gravity;
using Earth::WIE;
using Earth::qne;
using Earth::blh;
using Earth::DR_Inv;

using Rotation::RotVec2Quaternion;
using Rotation::SkewSymmetric;
using Rotation::Quaternion2Vector;
using Rotation::Quaternion2DCM;
using Rotation::DCM2Euler;

using Aided_INS_Space::PVA;
using Aided_INS_Space::IMU;

namespace //函数前置声明
{
    /**
     * @brief 速度更新
     *        velocity update
     */
    void VelUpdate(const PVA &pvaPre, PVA &pvaCur, const IMU &imuPre, const IMU &imuCur);

    /**
     * @brief 位置更新
     *        position update
     */
    void PosUpdate(const PVA &pvaPre, PVA &pvaCur, const IMU &imuCur);

    /**
     * @brief 姿态更新
     *        attitude update
     */
    void AttUpdate(const PVA &pvaPre, PVA &pvaCur, const IMU &imuPre, const IMU &imuCur);
}

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
    void INS_Mech(const PVA &pvaPre, PVA &pvaCur, const IMU &imuPre, const IMU &imuCur)
    {
        //依次进行速度更新、位置更新、姿态更新, 不可调换顺序
        VelUpdate(pvaPre, pvaCur, imuPre, imuCur);
        PosUpdate(pvaPre, pvaCur, imuCur);
        AttUpdate(pvaPre, pvaCur, imuPre, imuCur);
    }
}

namespace
{
    /**
     * @brief 速度更新
     *        velocity update
     */
    void VelUpdate(const PVA &pvaPre, PVA &pvaCur, const IMU &imuPre, const IMU &imuCur)
    {
        //计算地理参数，地球自转角速度投影到n系, n系相对于e系转动角速度投影到n系
        Vector3d w_ie_n = Earth::w_ie_n(pvaPre.pos[0]);
        Vector3d w_en_n = Earth::w_en_n(pvaPre.pos, pvaPre.vel);

        //b系比力积分项
        const Vector3d dv_f_b = imuCur.deltaVel +
                                imuCur.deltaTheta.cross(imuCur.deltaVel) * 0.5 +          //旋转效应
                                imuPre.deltaTheta.cross(imuCur.deltaVel) * (1.0 / 12.0) +
                                imuPre.deltaVel.cross(imuCur.deltaTheta) * (1.0 / 12.0);  //双子样划桨效应(基于相同采样间隔推导)

        //比力积分项投影到n系。此处理论上应该用(w_ie_n + w_en_n)在中间时刻（k-1/2）的角速度，但目前因为尚无法获得k时刻的w_en_n，且这一项产生的误差小，所以使用k-1 时刻的量近似
        Vector3d phi_nk1_nk = (w_ie_n + w_en_n) * imuCur.dt; //等效旋转矢量phi_n(k-1)_n(k)
        Matrix3d C_nk1_nk = Matrix3d::Identity() - SkewSymmetric(phi_nk1_nk) * 0.5; //C_n(k-1)_n(k)
        Vector3d dv_f_n = C_nk1_nk * pvaPre.att.Cbn * dv_f_b;

        //计算重力/哥式积分项
        double gravity = Gravity(pvaPre.pos);
        Vector3d g_l_n{0, 0, gravity}; //当地重力加速度在n系的投影
        Vector3d dv_g_n = (g_l_n - (2.0 * w_ie_n + w_en_n).cross(pvaPre.vel)) * imuCur.dt;

        // 得到中间时刻（k-1/2）速度
        const Vector3d velMiddle = pvaPre.vel + (dv_f_n + dv_g_n) * 0.5;

        // 外推得到中间时刻（k-1/2）位置
        const Vector3d phi_nk1_nk12 = (w_ie_n + w_en_n) * imuCur.dt * 0.5; //phi_n(k-1)_n(k-1/2)。 w_ie_n + w_en_n近似为常数，对等效旋转矢量做内插
        const Quaterniond q_nk12_nk1 = RotVec2Quaternion(phi_nk1_nk12); //q_n(k-1/2)_n(k-1)。
        const Vector3d phi_ek12_ek1{0, 0, -WIE * imuCur.dt * 0.5}; //phi_e(k-1/2)_e(k-1)，e系转动等效旋转矢量，因为WIE为常数，所以时间可以平移。
        const Quaterniond q_ek1_ek12 = RotVec2Quaternion(phi_ek12_ek1); //q_e(k-1)_e(k-1/2)
        const Quaterniond q_nk1_ek1 = qne(pvaPre.pos); //q_n(k-1)_e(k-1)
        const Quaterniond q_nk12_ek12 = (q_ek1_ek12 * q_nk1_ek1 * q_nk12_nk1).normalized(); //q_n(k-1/2)_e(k-1/2) = q_e(k-1)_e(k-1/2) * q_n(k-1)_e(k-1) * q_n(k-1/2)_n(k-1)
        const double hMiddle = pvaPre.pos[2] - velMiddle[2] * imuCur.dt * 0.5;
        const Vector3d posMiddle = blh(q_nk12_ek12, hMiddle);

        // 重新计算中间时刻（k-1/2）的w_ie_n, w_en_n
        w_ie_n = Earth::w_ie_n(posMiddle[0]);
        w_en_n = Earth::w_en_n(posMiddle, velMiddle);

        // 重新计算n系下平均比力积分项
        phi_nk1_nk = (w_ie_n + w_en_n) * imuCur.dt; //等效旋转矢量phi_n(k-1)_n(k)
        C_nk1_nk   = Matrix3d::Identity() - SkewSymmetric(phi_nk1_nk) * 0.5; //C_n(k-1)_n(k)
        dv_f_n = C_nk1_nk * pvaPre.att.Cbn * dv_f_b;

        // 重新计算重力、哥式积分项
        gravity = Gravity(posMiddle);
        g_l_n << 0, 0, gravity;
        dv_g_n = (g_l_n - (2.0 * w_ie_n + w_en_n).cross(velMiddle)) * imuCur.dt;

        // 速度更新完成
        pvaCur.vel = pvaPre.vel + dv_f_n + dv_g_n;
    }


    /**
     * @brief 位置更新
     *        position update
     */
    void PosUpdate(const PVA &pvaPre, PVA &pvaCur, const IMU &imuCur)
    {
        // 重新计算中间时刻（k-1/2）的速度和位置
        const Vector3d velMiddle = (pvaCur.vel + pvaPre.vel) * 0.5;
        const Vector3d posMiddle = pvaPre.pos + DR_Inv(pvaPre.pos) * velMiddle * imuCur.dt * 0.5;

        // 重新计算中间时刻（k-1/2）地理参数
        const Vector3d w_ie_n = Earth::w_ie_n(posMiddle[0]);
        const Vector3d w_en_n = Earth::w_en_n(posMiddle, velMiddle);

        // 重新计算 k 时刻到 k-1 时刻 n系旋转矢量
        const Vector3d phi_nk1_nk = (w_ie_n + w_en_n) * imuCur.dt; //等效旋转矢量phi_n(k-1)_n(k)
        const Quaterniond q_nk_nk1 = RotVec2Quaternion(phi_nk1_nk); //q_n(k)_n(k-1)
        const Vector3d phi_ek_ek1{0, 0, -WIE * imuCur.dt}; //phi_e(k)_e(k-1)，e系转动等效旋转矢量
        const Quaterniond q_ek1_ek = RotVec2Quaternion(phi_ek_ek1); //q_e(k-1)_e(k)

        // 位置更新完成
        const Quaterniond q_nk1_ek1 = qne(pvaPre.pos); //q_n(k-1)_e(k-1)
        const Quaterniond q_nk_ek   = (q_ek1_ek * q_nk1_ek1 * q_nk_nk1).normalized(); //q_n(k)_e(k) = q_e(k-1)_e(k) * q_n(k-1)_e(k-1) * q_n(k)_n(k-1)
        const double hTemp = pvaPre.pos[2] - velMiddle[2] * imuCur.dt;
        pvaCur.pos = blh(q_nk_ek, hTemp);
    }

    /**
     * @brief 姿态更新
     *        attitude update
     */
    void AttUpdate(const PVA &pvaPre, PVA &pvaCur, const IMU &imuPre, const IMU &imuCur)
    {
        // 重新计算中间时刻（k-1/2）的速度和位置
        const Vector3d velMiddle = (pvaPre.vel + pvaCur.vel) * 0.5;
        const Quaterniond q_nk1_ek1 = qne(pvaPre.pos); //q_n(k-1)_e(k-1)
        const Quaterniond q_nk_ek = qne(pvaCur.pos); //q_n(k)_e(k)
        const Vector3d phi_ek_ek1{0, 0, -WIE * imuCur.dt}; //phi_e(k)_e(k-1)，e系转动等效旋转矢量
        const Quaterniond q_ek1_ek = RotVec2Quaternion(phi_ek_ek1); //q_e(k-1)_e(k)
        Vector3d phi_nk_nk1 = Quaternion2Vector((q_nk1_ek1.inverse() * q_ek1_ek.inverse() * q_nk_ek).normalized()); //q_n(k)_n(k-1) = q_e(k-1)_n(k-1) * q_e(k)_e(k-1) * q_n(k)_e(k)
        const Vector3d phi_ek12_ek1{0, 0, -WIE * imuCur.dt * 0.5}; //phi_e(k-1/2)_e(k-1)，e系转动等效旋转矢量，因为WIE为常数，所以时间可以平移。
        const Quaterniond q_ek1_ek12 = RotVec2Quaternion(phi_ek12_ek1); //q_e(k-1)_e(k-1/2)
        const Quaterniond q_nk12_ek12 = (q_ek1_ek12 * q_nk1_ek1 * RotVec2Quaternion(phi_nk_nk1 * 0.5)).normalized(); //q_n(k-1/2)_e(k-1/2) = q_e(k-1)_e(k-1/2) * q_n(k-1)_e(k-1) * q_n(k-1/2)_n(k-1)
        const double hMiddle = (pvaCur.pos[2] + pvaPre.pos[2]) * 0.5;
        const Vector3d posMiddle = blh(q_nk12_ek12, hMiddle);

        // 重新计算中间时刻（k-1/2）的w_ie_n, w_en_n
        const Vector3d w_ie_n = Earth::w_ie_n(posMiddle[0]);
        const Vector3d w_en_n = Earth::w_en_n(posMiddle, velMiddle);

        // 计算n系的旋转四元数 k-1 时刻到k时刻变换
        phi_nk_nk1 = -(w_ie_n + w_en_n) * imuCur.dt; //phi_n(k)_n(k-1)
        const Quaterniond q_nk1_nk = RotVec2Quaternion(phi_nk_nk1); //q_n(k-1)_n(k)

        // 计算b系旋转四元数 补偿二阶圆锥误差(基于相同采样间隔推导)
        const Vector3d phi_bk1_bk = imuCur.deltaTheta + imuPre.deltaTheta.cross(imuCur.deltaTheta) * (1.0 / 12.0); //phi_b(k-1)_b(k)
        const Quaterniond q_bk_bk1 = RotVec2Quaternion(phi_bk1_bk); //q_b(k)_b(k-1)

        // 姿态更新完成
        pvaCur.att.qbn   = (q_nk1_nk * pvaPre.att.qbn * q_bk_bk1).normalized();
        pvaCur.att.Cbn   = Quaternion2DCM(pvaCur.att.qbn);
        pvaCur.att.euler = DCM2Euler(pvaCur.att.Cbn);
    }
}
