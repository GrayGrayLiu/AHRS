/******************************************************************************
 * @file    AngleUtilities.hpp
 * @brief   角度转换
 *
 * @details
 * 包括度转弧度、弧度转度，采用模版泛型编程
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

#ifndef AHRS_ANGLEUTILITIES_HPP
#define AHRS_ANGLEUTILITIES_HPP

// #include <Eigen/Dense>

namespace Angle
{
    constexpr double PI = 3.14159265358979323846;
    constexpr double D2R = PI / 180.0;
    constexpr double R2D = 180.0 / PI;

    inline double Rad2Deg(const double rad)
    {
        return rad * R2D;
    }

    inline double Deg2Rad(const double deg)
    {
        return deg * D2R;
    }

    // inline float Rad2Deg(const float rad)
    // {
    //     return rad * static_cast<float>(R2D);
    // }
    //
    // inline float Deg2Rad(const float deg)
    // {
    //     return deg * static_cast<float>(D2R);
    // }

    template <typename T, int Rows, int Cols>
    Eigen::Matrix<T, Rows, Cols> Rad2Deg(const Eigen::Matrix<T, Rows, Cols> &array)
    {
        return array * R2D;
    }

    template <typename T, int Rows, int Cols>
    Eigen::Matrix<T, Rows, Cols> Deg2Rad(const Eigen::Matrix<T, Rows, Cols> &array)
    {
        return array * D2R;
    }
}

#endif //AHRS_ANGLEUTILITIES_HPP
