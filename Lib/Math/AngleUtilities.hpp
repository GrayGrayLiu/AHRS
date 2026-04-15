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

#include <Eigen/Dense>

namespace Angle
{
    constexpr double PI = 3.14159265358979323846;
    constexpr double D2R = PI / 180.0;
    constexpr double R2D = 180.0 / PI;

    template <typename T>
    constexpr T Deg2Rad(T deg)
    {
        return deg * static_cast<T>(D2R);
    }

    template <typename T>
    constexpr T Rad2Deg(T rad)
    {
        return rad * static_cast<T>(R2D);
    }

    template <typename Derived>
    auto Deg2Rad(const Eigen::MatrixBase<Derived>& x)
    {
        using Scalar = typename Derived::Scalar;
        return x * static_cast<Scalar>(D2R);
    }

    template <typename Derived>
    auto Rad2Deg(const Eigen::MatrixBase<Derived>& x)
    {
        using Scalar = typename Derived::Scalar;
        return x * static_cast<Scalar>(R2D);
    }
}

#endif //AHRS_ANGLEUTILITIES_HPP
