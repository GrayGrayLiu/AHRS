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

#ifndef AHRS_CONFIGURATION_H
#define AHRS_CONFIGURATION_H

/****************************************初始状态****************************************/
//初始位置：纬度、经度、高程
#define INIT_POS_LATITUDE   30.263842 //纬度[deg]
#define INIT_POS_LONGITUDE 120.123077 //经度[deg]
#define INIT_POS_ALTITUDE   20.0      //高程[m]

//初始速度：北向速度、东向速度、垂向速度
#define INIT_VEL_NORTH 0.0 //北向速度[m/s]
#define INIT_VEL_EAST  0.0 //东向速度[m/s]
#define INIT_VEL_DOWN  0.0 //垂向速度[m/s]

//初始姿态, 欧拉角(ZYX旋转顺序): 横滚、俯仰、偏航
#define INIT_ATT_ROLL  0.0 //横滚[deg]
#define INIT_ATT_PITCH 0.0 //俯仰[deg]
#define INIT_ATT_YAW   0.0 //偏航[deg]

//初始IMU零偏，IMU的三个轴(x前、y右、z下)
#define INIT_GYR_BIAS_X 0.0 //陀螺X轴零偏[deg/h]
#define INIT_GYR_BIAS_Y 0.0 //陀螺Y轴零偏[deg/h]
#define INIT_GYR_BIAS_Z 0.0 //陀螺Z轴零偏[deg/h]
#define INIT_ACC_BIAS_X 0.0 //加速度计X轴零偏[mGal]，1Gal=10e^-2 m/s²
#define INIT_ACC_BIAS_Y 0.0 //加速度计Y轴零偏[mGal]
#define INIT_ACC_BIAS_Z 0.0 //加速度计Z轴零偏[mGal]

//初始IMU比例因子，IMU的三个轴(x前、y右、z下)
#define INIT_GYR_SCALE_X 0.0 //陀螺X轴比例因子[ppm]，ppm：Parts Per Million，百万分之一
#define INIT_GYR_SCALE_Y 0.0 //陀螺Y轴比例因子[ppm]
#define INIT_GYR_SCALE_Z 0.0 //陀螺Z轴比例因子[ppm]
#define INIT_ACC_SCALE_X 0.0 //加速度计X轴比例因子[ppm]
#define INIT_ACC_SCALE_Y 0.0 //加速度计Y轴比例因子[ppm]
#define INIT_ACC_SCALE_Z 0.0 //加速度计Z轴比例因子[ppm]
/***************************************************************************************/


/************************************初始误差状态标准差************************************/
//初始位置误差状态标准差：纬度、经度、高程
#define INIT_POS_STD_LATITUDE  0.1 //纬度误差状态标准差[m]
#define INIT_POS_STD_LONGITUDE 0.1 //经度误差状态标准差[m]
#define INIT_POS_STD_ALTITUDE  0.2 //高程误差状态标准差[m]

//初始速度误差状态标准差：北向速度、东向速度、垂向速度
#define INIT_VEL_STD_NORTH 0.05 //北向速度误差状态标准差[m/s]
#define INIT_VEL_STD_EAST  0.05 //东向速度误差状态标准差[m/s]
#define INIT_VEL_STD_DOWN  0.05 //垂向速度误差状态标准差[m/s]

//初始姿态误差状态标准差, 欧拉角(ZYX旋转顺序): 横滚、俯仰、偏航
#define INIT_ATT_STD_ROLL  0.5 //横滚误差状态标准差[deg]
#define INIT_ATT_STD_PITCH 0.5 //俯仰误差状态标准差[deg]
#define INIT_ATT_STD_YAW   1.0 //偏航误差状态标准差[deg]

//初始IMU零偏误差状态标准差，IMU的三个轴(x前、y右、z下)
#define INIT_GYR_BIAS_STD_X 0.0 //陀螺X轴零偏误差状态标准差[deg/h]
#define INIT_GYR_BIAS_STD_Y 0.0 //陀螺Y轴零偏误差状态标准差[deg/h]
#define INIT_GYR_BIAS_STD_Z 0.0 //陀螺Z轴零偏误差状态标准差[deg/h]
#define INIT_ACC_BIAS_STD_X 0.0 //加速度计X轴零偏误差状态标准差[mGal]，1Gal=10e^-2 m/s²
#define INIT_ACC_BIAS_STD_Y 0.0 //加速度计Y轴零偏误差状态标准差[mGal]
#define INIT_ACC_BIAS_STD_Z 0.0 //加速度计Z轴零偏误差状态标准差[mGal]

//初始IMU比例因子误差状态标准差，IMU的三个轴(x前、y右、z下)
#define INIT_GYR_SCALE_STD_X 0.0 //陀螺X轴比例因子误差状态标准差[ppm]，ppm：Parts Per Million，百万分之一
#define INIT_GYR_SCALE_STD_Y 0.0 //陀螺Y轴比例因子误差状态标准差[ppm]
#define INIT_GYR_SCALE_STD_Z 0.0 //陀螺Z轴比例因子误差状态标准差[ppm]
#define INIT_ACC_SCALE_STD_X 0.0 //加速度计X轴比例因子误差状态标准差[ppm]
#define INIT_ACC_SCALE_STD_Y 0.0 //加速度计Y轴比例因子误差状态标准差[ppm]
#define INIT_ACC_SCALE_STD_Z 0.0 //加速度计Z轴比例因子误差状态标准差[ppm]
/***************************************************************************************/


/**************************************IMU噪声参数***************************************/
//IMU随机游走
#define GYR_ARW_X 0.0 //陀螺X轴角度随机游走[deg/sqrt(hr)]
#define GYR_ARW_Y 0.0 //陀螺Y轴角度随机游走[deg/sqrt(hr)]
#define GYR_ARW_Z 0.0 //陀螺Z轴角度随机游走[deg/sqrt(hr)]
#define ACC_VRW_X 0.0 //加速度计X轴速度随机游走[m/s/sqrt(hr)]
#define ACC_VRW_Y 0.0 //加速度计Y轴速度随机游走[m/s/sqrt(hr)]
#define ACC_VRW_Z 0.0 //加速度计Z轴速度随机游走[m/s/sqrt(hr)]

//IMU零偏标准差，IMU的三个轴(x前、y右、z下)
#define GYR_BIAS_STD_X 0.0 //陀螺X轴零偏标准差[deg/h]
#define GYR_BIAS_STD_Y 0.0 //陀螺Y轴零偏标准差[deg/h]
#define GYR_BIAS_STD_Z 0.0 //陀螺Z轴零偏标准差[deg/h]
#define ACC_BIAS_STD_X 0.0 //加速度计X轴零偏标准差[mGal]，1Gal=10e^-2 m/s²
#define ACC_BIAS_STD_Y 0.0 //加速度计Y轴零偏标准差[mGal]
#define ACC_BIAS_STD_Z 0.0 //加速度计Z轴零偏标准差[mGal]

//IMU比例因子标准差，IMU的三个轴(x前、y右、z下)
#define GYR_SCALE_STD_X 0.0 //陀螺X轴比例因子标准差[ppm]，ppm：Parts Per Million，百万分之一
#define GYR_SCALE_STD_Y 0.0 //陀螺Y轴比例因子标准差[ppm]
#define GYR_SCALE_STD_Z 0.0 //陀螺Z轴比例因子标准差[ppm]
#define ACC_SCALE_STD_X 0.0 //加速度计X轴比例因子标准差[ppm]
#define ACC_SCALE_STD_Y 0.0 //加速度计Y轴比例因子标准差[ppm]
#define ACC_SCALE_STD_Z 0.0 //加速度计Z轴比例因子标准差[ppm]

//相关时间
#define IMU_CORR_TIME 1.0 //陀螺和加速度计的零偏、比例因子的相关时间[h]
/***************************************************************************************/


/**************************************磁力计噪声参数*************************************/
#define MAG_MEASURE_YAW_STD 2.0 //磁力计测量偏航角的标准差[deg]
/***************************************************************************************/


/*******************************天线杆臂，IMU坐标系前右下方向*******************************/
#define GNSS_ANTENNA_LEVER_X 0.0 //[m]
#define GNSS_ANTENNA_LEVER_Y 0.0 //[m]
#define GNSS_ANTENNA_LEVER_Z 0.0 //[m]
/***************************************************************************************/

#endif //AHRS_CONFIGURATION_H
