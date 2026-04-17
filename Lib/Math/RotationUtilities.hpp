/******************************************************************************
 * @file    RotationUtilities.hpp
 * @brief   
 *
 * @details
 * 
 *
 * @author  Gray
 * @email   grayme12345@gmail.com
 * @date    2026/4/12
 *
 * @copyright
 * Copyright (c) 2026 Gray
 *
 * This software is provided "as is" without any warranties.
 *
 ******************************************************************************/

#ifndef AHRS_ROTATIONUTILITIES_HPP
#define AHRS_ROTATIONUTILITIES_HPP

#include <Eigen/Geometry>
#include <cmath>

namespace Rotation
{
    using Eigen::Matrix3d;
    using Eigen::Quaterniond;
    using Eigen::Vector3d;
    using Eigen::AngleAxisd;
    using Eigen::Matrix4d;

    constexpr double PI = 3.14159265358979323846;

    /**
      * @brief 将方向余弦矩阵转换为四元数
      * @param DCM 方向余弦矩阵
      * @retval 四元数
      */
    inline Quaterniond DCM2Quaternion(const Matrix3d& DCM)
    {
        return Quaterniond(DCM);
    }

    /**
      * @brief 将四元数转换为方向余弦矩阵
      * @param q 四元数
      * @retval 方向余弦矩阵
      */
    inline Matrix3d Quaternion2DCM(const Quaterniond& q)
    {
        return q.toRotationMatrix();
    }

    /**
      * @brief 根据方向余弦矩阵输出欧拉角。载体坐标系为前右下，旋转顺序为ZYX, 输出RPY（Roll、Pitch、Yaw）
      * @param DCM 方向余弦矩阵
      * @retval 欧拉角
      */
    inline Vector3d DCM2Euler(const Matrix3d& DCM)
    {
        Vector3d euler;

        if (const double sPitch = std::clamp(-DCM(2, 0), -1.0, 1.0); std::abs(sPitch) >= 0.999) // 奇异
        {
            euler[0] = 0.0;
            euler[1] = std::asin(sPitch);
            euler[2] = std::atan2(-DCM(0,1), DCM(1,1));
        }
        else
        {
            euler[0] = std::atan2(DCM(2,1), DCM(2,2));   // roll
            euler[1] = std::asin(sPitch);                // pitch
            euler[2] = std::atan2(DCM(1,0), DCM(0,0));   // yaw
        }

        // 将Yaw从±π转换为0~2π
        if (euler[2] < 0) euler[2] += 2.0 * PI;

        return euler;
    }

    /**
      * @brief 根据姿态四元数输出欧拉角
      * @param q 姿态四元数
      * @retval 欧拉角
      */
    inline Vector3d Quaternion2Euler(const Quaterniond& q)
    {
        return DCM2Euler(q.toRotationMatrix());
    }

    /**
      * @brief 等效旋转矢量转换为四元数
      * @param rotVec 等效旋转矢量
      * @retval 四元数
      */
    inline Quaterniond RotVec2Quaternion(const Vector3d &rotVec)
    {
        const double angle = rotVec.norm();
        const Vector3d vec = rotVec.normalized();
        return Quaterniond(Eigen::AngleAxisd(angle, vec));
    }

    /**
      * @brief 四元数转换为等效旋转矢量
      * @param quaternion 四元数
      * @retval 欧拉角
      */
    inline Vector3d Quaternion2Vector(const Quaterniond &quaternion)
    {
        Eigen::AngleAxisd axisd(quaternion);
        return axisd.angle() * axisd.axis();
    }

    // RPY --> C_b^n, ZYX顺序
    inline Matrix3d Euler2Matrix(const Vector3d &euler)
    {
        return Matrix3d(Eigen::AngleAxisd(euler[2], Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(euler[1], Vector3d::UnitY()) *
                        Eigen::AngleAxisd(euler[0], Vector3d::UnitX()));
    }

    inline Quaterniond Euler2Quaternion(const Vector3d &euler)
    {
        return Quaterniond(Eigen::AngleAxisd(euler[2], Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(euler[1], Vector3d::UnitY()) *
                           Eigen::AngleAxisd(euler[0], Vector3d::UnitX()));
    }

    //反对称矩阵
    inline Matrix3d SkewSymmetric(const Vector3d &vector)
    {
        Matrix3d mat;
        mat <<         0,        -vector(2), vector(1),
               vector(2),         0,        -vector(0),
              -vector(1),    vector(0),     0;
        return mat;
    }

    inline Matrix4d QuaternionLeft(const Quaterniond &q)
    {
        Matrix4d ans;
        ans(0, 0)             = q.w();
        ans.block<1, 3>(0, 1) = -q.vec().transpose();
        ans.block<3, 1>(1, 0) = q.vec();
        ans.block<3, 3>(1, 1) = q.w() * Eigen::Matrix3d::Identity() + SkewSymmetric(q.vec());
        return ans;
    }

    inline Matrix4d QuaternionRight(const Quaterniond &p)
    {
        Matrix4d ans;
        ans(0, 0)             = p.w();
        ans.block<1, 3>(0, 1) = -p.vec().transpose();
        ans.block<3, 1>(1, 0) = p.vec();
        ans.block<3, 3>(1, 1) = p.w() * Eigen::Matrix3d::Identity() - SkewSymmetric(p.vec());
        return ans;
    }
}

#endif //AHRS_ROTATIONUTILITIES_HPP
