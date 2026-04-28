/******************************************************************************
 * @file    Configuration.hpp
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

#ifndef AHRS_CONFIGURATION_H
#define AHRS_CONFIGURATION_H

namespace Aided_INS_Config
{
    /****************************************初始状态****************************************/
    namespace StateInit
    {
        //初始位置：纬度、经度、高程
        constexpr double initPosLatitude  = 30.263842; //纬度[deg]
        constexpr double initPosLongitude = 120.123077; //经度[deg]
        constexpr double initPosAltitude  = 20.0; //高程[m]

        //初始速度：北向速度、东向速度、垂向速度
        constexpr double initVelNorth = 0.0; //北向速度[m/s]
        constexpr double initVelEast  = 0.0; //东向速度[m/s]
        constexpr double initVelDown  = 0.0; //垂向速度[m/s]

        //初始姿态, 欧拉角(ZYX旋转顺序): 横滚、俯仰、偏航
        constexpr double initAttRoll  = 0.0; //横滚[deg]
        constexpr double initAttPitch = 0.0; //俯仰[deg]
        constexpr double initAttYaw   = 0.0; //偏航[deg]

        //初始IMU零偏，IMU的三个轴(x前、y右、z下)
        constexpr double initGyrBiasX = 0.0; //陀螺X轴零偏[deg/h]
        constexpr double initGyrBiasY = 0.0; //陀螺Y轴零偏[deg/h]
        constexpr double initGyrBiasZ = 0.0; //陀螺Z轴零偏[deg/h]
        constexpr double initAccBiasX = 0.0; //加速度计X轴零偏[mGal]，1Gal=10e^-2 m/s²，1mGal==10e^-5 m/s²
        constexpr double initAccBiasY = 0.0; //加速度计Y轴零偏[mGal]
        constexpr double initAccBiasZ = 0.0; //加速度计Z轴零偏[mGal]

        //初始IMU比例因子，IMU的三个轴(x前、y右、z下)
        constexpr double initGyrScaleX = 0.0; //陀螺X轴比例因子[ppm]，ppm：Parts Per Million，百万分之一
        constexpr double initGyrScaleY = 0.0; //陀螺Y轴比例因子[ppm]
        constexpr double initGyrScaleZ = 0.0; //陀螺Z轴比例因子[ppm]
        constexpr double initAccScaleX = 0.0; //加速度计X轴比例因子[ppm]
        constexpr double initAccScaleY = 0.0; //加速度计Y轴比例因子[ppm]
        constexpr double initAccScaleZ = 0.0; //加速度计Z轴比例因子[ppm]
    }
    /***************************************************************************************/


    /************************************初始误差状态标准差************************************/
    namespace ErrStateStdInit
    {
        //初始位置误差状态标准差：纬度、经度、高程
        constexpr double initPosStdLatitude  = 0.1; //纬度误差状态标准差[m]
        constexpr double initPosStdLongitude = 0.1; //经度误差状态标准差[m]
        constexpr double initPosStdAltitude  = 0.2; //高程误差状态标准差[m]

        //初始速度误差状态标准差：北向速度、东向速度、垂向速度
        constexpr double initVelStdNorth = 0.05; //北向速度误差状态标准差[m/s]
        constexpr double initVelStdEast  = 0.05; //东向速度误差状态标准差[m/s]
        constexpr double initVelStdDown  = 0.05; //垂向速度误差状态标准差[m/s]

        //初始姿态误差状态标准差, 欧拉角(ZYX旋转顺序): 横滚、俯仰、偏航
        constexpr double initAttStdRoll  = 0.5; //横滚误差状态标准差[deg]
        constexpr double initAttStdPitch = 0.5; //俯仰误差状态标准差[deg]
        constexpr double initAttStdYaw   = 1.0; //偏航误差状态标准差[deg]

        //初始IMU零偏误差状态标准差，IMU的三个轴(x前、y右、z下)
        constexpr double initGyrBiasStdX = 0.0; //陀螺X轴零偏误差状态标准差[deg/h]
        constexpr double initGyrBiasStdY = 0.0; //陀螺Y轴零偏误差状态标准差[deg/h]
        constexpr double initGyrBiasStdZ = 0.0; //陀螺Z轴零偏误差状态标准差[deg/h]
        constexpr double initAccBiasStdX = 0.0; //加速度计X轴零偏误差状态标准差[mGal]，1Gal=10e^-2 m/s²，1mGal==10e^-5 m/s²
        constexpr double initAccBiasStdY = 0.0; //加速度计Y轴零偏误差状态标准差[mGal]
        constexpr double initAccBiasStdZ = 0.0; //加速度计Z轴零偏误差状态标准差[mGal]

        //初始IMU比例因子误差状态标准差，IMU的三个轴(x前、y右、z下)
        constexpr double initGyrScaleStdX = 0.0; //陀螺X轴比例因子误差状态标准差[ppm]，ppm：Parts Per Million，百万分之一
        constexpr double initGyrScaleStdY = 0.0; //陀螺Y轴比例因子误差状态标准差[ppm]
        constexpr double initGyrScaleStdZ = 0.0; //陀螺Z轴比例因子误差状态标准差[ppm]
        constexpr double initAccScaleStdX = 0.0; //加速度计X轴比例因子误差状态标准差[ppm]
        constexpr double initAccScaleStdY = 0.0; //加速度计Y轴比例因子误差状态标准差[ppm]
        constexpr double initAccScaleStdZ = 0.0; //加速度计Z轴比例因子误差状态标准差[ppm]
    }
    /***************************************************************************************/


    /**************************************IMU噪声参数***************************************/
    namespace IMU_Noise
    {
        //IMU随机游走
        constexpr double gyrArwX = 0.0; //陀螺X轴角度随机游走[deg/sqrt(hr)]
        constexpr double gyrArwY = 0.0; //陀螺Y轴角度随机游走[deg/sqrt(hr)]
        constexpr double gyrArwZ = 0.0; //陀螺Z轴角度随机游走[deg/sqrt(hr)]
        constexpr double accVrwX = 0.0; //加速度计X轴速度随机游走[m/s/sqrt(hr)]
        constexpr double accVrwY = 0.0; //加速度计Y轴速度随机游走[m/s/sqrt(hr)]
        constexpr double accVrwZ = 0.0; //加速度计Z轴速度随机游走[m/s/sqrt(hr)]

        //IMU零偏标准差，IMU的三个轴(x前、y右、z下)
        constexpr double gyrBiasStdX = 0.0; //陀螺X轴零偏标准差[deg/h]
        constexpr double gyrBiasStdY = 0.0; //陀螺Y轴零偏标准差[deg/h]
        constexpr double gyrBiasStdZ = 0.0; //陀螺Z轴零偏标准差[deg/h]
        constexpr double accBiasStdX = 0.0; //加速度计X轴零偏标准差[mGal]，1Gal=10e^-2 m/s²
        constexpr double accBiasStdY = 0.0; //加速度计Y轴零偏标准差[mGal]
        constexpr double accBiasStdZ = 0.0; //加速度计Z轴零偏标准差[mGal]

        //IMU比例因子标准差，IMU的三个轴(x前、y右、z下)
        constexpr double gyrScaleStdX = 0.0; //陀螺X轴比例因子标准差[ppm]，ppm：Parts Per Million，百万分之一
        constexpr double gyrScaleStdY = 0.0; //陀螺Y轴比例因子标准差[ppm]
        constexpr double gyrScaleStdZ = 0.0; //陀螺Z轴比例因子标准差[ppm]
        constexpr double accScaleStdX = 0.0; //加速度计X轴比例因子标准差[ppm]
        constexpr double accScaleStdY = 0.0; //加速度计Y轴比例因子标准差[ppm]
        constexpr double accScaleStdZ = 0.0; //加速度计Z轴比例因子标准差[ppm]

        //相关时间
        constexpr double imuCorrTime = 1.0; //陀螺和加速度计的零偏、比例因子的相关时间[h]
    }
    /***************************************************************************************/


    /**************************************磁力计噪声参数*************************************/
    namespace MagNoise
    {
        constexpr double magMeasureYawStd = 2.0; //磁力计测量偏航角的标准差[deg]
    }
    /***************************************************************************************/


    /*******************************天线杆臂，IMU坐标系前右下方向*******************************/
    namespace GNSS_AntennaLever
    {
        constexpr double gnssAntennaLeverX = 0.0; //[m]
        constexpr double gnssAntennaLeverY = 0.0; //[m]
        constexpr double gnssAntennaLeverZ = 0.0; //[m]
    }
    /***************************************************************************************/
}

#endif //AHRS_CONFIGURATION_H
