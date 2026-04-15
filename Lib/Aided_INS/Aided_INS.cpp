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
#include "Configuration.h"

Aided_INS::Aided_INS(const uint8_t id)
{

}

int Aided_INS::InitialAlignment()
{
    return 0;
}

int Aided_INS::Run()
{
    return 0;
}
