/******************************************************************************
 * @file    EarthUtilities.hpp
 * @brief   地球模型相关的处理函数
 *
 * @details
 * 重力加速度计算、子午圈曲率半径、卯酉圈曲率半径；
 * n系到e系的转换矩阵、n系到e系的转换四元数、根据转换四元数得到纬经高；
 * 大地坐标(纬度、经度和高程)转地心地固坐标、地心地固坐标转大地坐标；
 * 大地坐标相对坐标（纬经高）转n系相对坐标（北东地坐标）的转换矩阵；
 * 局部坐标（n系下的坐标）转大地坐标、大地坐标转局部坐标；
 * 地球自转角速度投影到e系、地球自转角速度投影到n系；
 * 物体相对于e系转动角速度投影到n系。
 *
 * @author  Gray
 * @email   grayme12345@gmail.com
 * @date    2026/4/8
 *
 * @copyright
 * Copyright (c) 2026 Gray
 *
 * This software is provided "as is" without any warranties.
 *
 ******************************************************************************/

#ifndef AHRS_EARTHUTILITIES_HPP
#define AHRS_EARTHUTILITIES_HPP

#include <Eigen/Dense>
#include <cmath>

namespace Earth
{
    using Eigen::Matrix3d;
    using Eigen::Vector2d;
    using Eigen::Vector3d;
    using Eigen::Quaterniond;

    /*************************WGS84椭球模型参数*************************/
    constexpr double PI  = 3.14159265358979323846;
    constexpr double WIE = 7.2921151467E-5;       //地球自转角速度[rad/s]
    constexpr double F   = 0.0033528106647474805; //扁率
    constexpr double RA  = 6378137.0;             //长半轴a[m]
    constexpr double RB  = 6356752.3142451793;    //短半轴b[m]
    constexpr double GM0 = 398600441800000.0;     //地球引力常数
    constexpr double E1  = 0.0066943799901413156; //第一偏心率平方
    constexpr double E2  = 0.0067394967422764341; //第二偏心率平方
    /*****************************************************************/

    /**
      * @brief 根据维度、高度计算重力加速度
      * @param blh 纬经高[rad-rad-m]
      * @retval 重力加速度[m/s²]
      */
    inline double Gravity(const Vector3d& blh)
    {
        const double sinPhi = std::sin(blh[0]);
        const double sin2   = sinPhi * sinPhi;
        const double sin4   = sin2 * sin2;

        constexpr double gamma_a = 9.7803267715; //赤道处正常重力加速度

        //给定纬度处正常重力加速度的级数展开
        const double gamma0 = gamma_a * (1 + 0.0052790414 * sin2 + 0.0000232718 * sin4 +
                                       0.0000001262 * sin2 * sin4 + 0.0000000007 * sin4 * sin4);

        return gamma0 - (3.0877e-6 - 4.3e-9 * sin2) * blh[2] + 0.72e-12 * blh[2] * blh[2]; //正常重力加速度随高度变化
    }

    /**
      * @brief 根据纬度计算子午圈曲率半径和卯酉圈曲率半径
      * @param lat 纬度[rad]
      * @retval 二维向量：子午圈曲率半径[m]和卯酉圈曲率半径[m]
      */
    inline Vector2d RM_And_RN(const double lat)
    {
        const double sin_Lat = std::sin(lat);
        const double temp = 1.0 - E1 * sin_Lat * sin_Lat;
        const double sqrt_temp = std::sqrt(temp);

        return {RA * (1 - E1) / (sqrt_temp * temp), RA / sqrt_temp};
    }

    /**
     * @brief 根据纬度计算子午圈曲率半径
     * @param lat 纬度[rad]
     * @retval 子午圈曲率半径[m]
     */
    inline double RM(const double lat)
    {
        const double sin_Lat = std::sin(lat);
        const double temp = 1.0 - E1 * sin_Lat * sin_Lat;
        const double sqrt_temp = std::sqrt(temp);

        return RA * (1 - E1) / (sqrt_temp * temp);
    }

    /**
     * @brief 根据纬度计算卯酉圈曲率半径
     * @param lat 纬度[rad]
     * @retval 卯酉圈曲率半径[m]
     */
    inline double RN(const double lat)
    {
        const double sin_Lat = std::sin(lat);
        return RA / std::sqrt(1.0 - E1 * sin_Lat * sin_Lat);
    }

    /**
     * @brief n系(导航坐标系)到e系(地心地固坐标系)的方向余弦转换矩阵
     * @param blh 纬经高[rad-rad-m]
     * @retval 方向余弦转换矩阵
     */
    inline Matrix3d Cne(const Vector3d& blh)
    {
        const double s_Lat = std::sin(blh[0]);
        const double c_Lat = std::cos(blh[0]);
        const double s_Lon = std::sin(blh[1]);
        const double c_Lon = std::cos(blh[1]);

        Matrix3d C;
        C << -s_Lat * c_Lon, -s_Lon, -c_Lat * c_Lon,
             -s_Lat * s_Lon,  c_Lon, -c_Lat * s_Lon,
              c_Lat,          0.0,   -s_Lat;

        return C;
    }

    /**
     * @brief n系(导航坐标系)到e系(地心地固坐标系)转换四元数
     * @param blh 纬经高[rad-rad-m]
     * @retval 转换四元数
     */
    inline Quaterniond qne(const Vector3d& blh)
    {
        const double halfLat = -PI * 0.25 - blh[0] * 0.5;
        const double halfLon = blh[1] * 0.5;

        const double c_HalfLat = std::cos(halfLat);
        const double s_HalfLat = std::sin(halfLat);
        const double c_HalfLon = std::cos(halfLon);
        const double s_HalfLon = std::sin(halfLon);

        Quaterniond q;
        q.w() = c_HalfLat * c_HalfLon;
        q.x() = -s_HalfLat * s_HalfLon;
        q.y() =  s_HalfLat * c_HalfLon;
        q.z() =  c_HalfLat * s_HalfLon;

        return q.normalized();
    }

    /**
     * @brief 根据n系到e系的转换四元数、高度得到纬度、经度、高度
     * @param qne n系到e系的转换四元数
     * @param height 高度[m]
     * @retval 三维向量：纬经高[rad-rad-m]
     */
    inline Vector3d blh(const Quaterniond& qne, double height)
    {
        return
    {
            -2 * std::atan(qne.y() / qne.w()) - PI * 0.5,
             2 * std::atan2(qne.z(), qne.w()),
             height
        };
    }

    /**
     * @brief 大地坐标(纬度、经度和高程)转地心地固坐标
     * @param blh 纬经高[rad-rad-m]
     * @retval 三维向量：地心地固坐标系下的三维坐标
     */
    inline Vector3d blh2ECEF(const Vector3d& blh)
    {
        const double c_Lat = std::cos(blh[0]);
        const double s_Lat = std::sin(blh[0]);
        const double c_Lon = std::cos(blh[1]);
        const double s_Lon = std::sin(blh[1]);

        const double R_N  = RN(blh[0]);
        const double R_NH = R_N + blh[2];

        return
    {
            R_NH * c_Lat * c_Lon,
            R_NH * c_Lat * s_Lon,
            (R_NH - R_N * E1) * s_Lat
        };
    }

    /**
     * @brief 地心地固坐标转大地坐标
     * @param ECEF_xyz 三维向量：地心地固坐标系下的三维坐标[m]
     * @retval 纬经高[rad-rad-m]
     */
    inline Vector3d ECEF2blh(const Vector3d& ECEF_xyz)
    {
        const double p = std::hypot(ECEF_xyz[0], ECEF_xyz[1]); //求两个数的平方和的平方根

        double lat = std::atan(ECEF_xyz[2] / (p * (1 - E1)));
        double lon = 2.0 * std::atan2(ECEF_xyz[1], ECEF_xyz[0] + p);

        double h = 0, hPrev;

        do
        {
            hPrev = h;
            const double rn = RN(lat);
            h = p / std::cos(lat) - rn;
            lat = std::atan(ECEF_xyz[2] / (p * (1 - E1 * rn / (rn + h))));
        }
        while (std::fabs(h - hPrev) > 1e-4);

        return {lat, lon, h};
    }

    /**
     * @brief n系相对坐标（北东地坐标）转大地坐标相对坐标（纬经高）的转换矩阵（DR的逆）
     * @param blh 纬经高[rad-rad-m]
     * @retval 转换矩阵
     */
    inline Matrix3d DR_Inv(const Vector3d &blh)
    {
        Matrix3d DR_inv = Matrix3d::Zero();

        Vector2d RMN = RM_And_RN(blh[0]);

        DR_inv(0, 0) = 1.0 / (RMN[0] + blh[2]);
        DR_inv(1, 1) = 1.0 / ((RMN[1] + blh[2]) * cos(blh[0]));
        DR_inv(2, 2) = -1;
        return DR_inv;
    }

    /**
     * @brief 大地坐标相对坐标（纬经高）转n系相对坐标（北东地坐标）的转换矩阵
     * @param blh 纬经高[rad-rad-m]
     * @retval 转换矩阵
     */
    inline Matrix3d DR(const Vector3d &blh)
    {
        Matrix3d DR = Matrix3d::Zero();

        Vector2d RMN = RM_And_RN(blh[0]);

        DR(0, 0) = RMN[0] + blh[2];
        DR(1, 1) = (RMN[1] + blh[2]) * cos(blh[0]);
        DR(2, 2) = -1;
        return DR;
    }

    /**
     * @brief 局部坐标(在n系原点处展开)转大地坐标
     * @param origin_blh n系原点的纬经高[rad-rad-m]
     * @param local_xyz n系下某一点的坐标三维[m]
     * @retval blh 原n系下的一点的大地坐标纬经高[rad-rad-m]
     */
    inline Vector3d Local2Global(const Vector3d& origin_blh, const Vector3d& local_xyz)
    {
        auto ECEF0 = blh2ECEF(origin_blh);
        const auto C_ne     = Cne(origin_blh);
        return ECEF2blh(ECEF0 + C_ne * local_xyz);
    }

    /**
     * @brief 大地坐标转局部坐标(在n系原点处展开)
     * @param origin_blh n系原点的纬经高[rad-rad-m]
     * @param global_blh 某一点的大地坐标纬经高[rad-rad-m]
     * @retval blh 某点在n系下的坐标三维[m]
     */
    inline Vector3d Global2Local(const Vector3d& origin_blh, const Vector3d& global_blh)
    {
        auto ECEF0 = blh2ECEF(origin_blh);
        auto C_ne     = Cne(origin_blh);
        return C_ne.transpose() * (blh2ECEF(global_blh) - ECEF0);
    }

    /**
     * @brief 地球自转角速度投影到e系
     * @retval 转换矩阵
     */
    inline Vector3d w_ie_e()
    {
        return {0, 0, WIE};
    }

    /**
     * @brief 地球自转角速度投影到n系
     * @param lat 纬度[rad]
     * @retval 转换矩阵
     */
    inline Vector3d w_ie_n(const double lat)
    {
        return
    {
            WIE * std::cos(lat),
            0,
           -WIE * std::sin(lat)
        };
    }

    /**
     * @brief 地球自转角速度投影到n系
     * @param origin_blh n系原点的纬经高[rad-rad-m]
     * @param local_xyz n系下某一点的坐标三维[m]
     * @retval 转换矩阵
     */
    inline Vector3d w_ie_n(const Vector3d& origin_blh, const Vector3d& local_xyz)
    {
        Vector3d global = Local2Global(origin_blh, local_xyz);

        return w_ie_n(global[0]);
    }
    
    /**
     * @brief 物体相对于e系转动角速度投影到n系
     * @param rmn 二维向量：某点的子午圈和卯酉圈曲率半径[m]
     * @param blh 三维向量：某点的纬经高[rad-rad-m]
     * @param vel_en_n 三维向量：n系相对于e系的速度向量[m/s]
     * @retval 转换矩阵
     */
    inline Vector3d w_en_n(const Vector2d& rmn, const Vector3d& blh, const Vector3d& vel_en_n)
    {
        return
    {
            vel_en_n[1] / (rmn[1] + blh[2]),
           -vel_en_n[0] / (rmn[0] + blh[2]),
           -vel_en_n[1] * std::tan(blh[0]) / (rmn[1] + blh[2])
       };
    }

    /**
     * @brief 物体相对于e系转动角速度投影到n系
     * @param origin_blh n系原点的纬经高[rad-rad-m]
     * @param local_xyz n系下某一点的坐标三维[m]
     * @param vel_en_n 三维向量：n系相对于e系的速度向量[m/s]
     * @retval 转换矩阵
     */
    inline Vector3d w_en_n(const Vector3d& origin_blh, const Vector3d& local_xyz, const Vector3d& vel_en_n)
    {
        Vector3d global_blh = Local2Global(origin_blh, local_xyz);
        const Vector2d RMN = RM_And_RN(global_blh[0]);

        return w_en_n(RMN, global_blh, vel_en_n);
    }

    /**
     * @brief 物体相对于e系转动角速度投影到n系
     * @param global_blh 某一点的大地坐标纬经高[rad-rad-m]
     * @param vel_en_n 三维向量：n系相对于e系的速度向量[m/s]
     * @retval 转换矩阵
     */
    inline Vector3d w_en_n(const Vector3d& global_blh, const Vector3d& vel_en_n)
    {
        const Vector2d RMN = RM_And_RN(global_blh[0]);

        return w_en_n(RMN, global_blh, vel_en_n);
    }
}

#endif //AHRS_EARTHUTILITIES_HPP
