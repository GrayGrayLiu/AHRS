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

};

#endif //AHRS_AIDED_INS_H
