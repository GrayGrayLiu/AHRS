/******************************************************************************
 * @file    Aided_INS_Types.hpp
 * @brief   定义组合惯性导航所需的数据结构
 *
 * @details
 *
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

#ifndef AHRS_AIDED_INS_TYPES_HPP
#define AHRS_AIDED_INS_TYPES_HPP

#include <Eigen/Dense>
#include "AngleUtilities.hpp"

namespace Aided_INS_Space
{
    /******************导航解算（机械编排+KF+状态误差反馈）状态量********************/
    struct Attitude
    {
        Eigen::Vector3d euler;  //欧拉角
        Eigen::Matrix3d Cbn;    //方向余弦矩阵（b系转到n系）
        Eigen::Quaterniond qbn; //姿态四元数（b系转到n系）
    };

    struct PVA
    {
        Eigen::Vector3d pos; //位置
        Eigen::Vector3d vel; //速度
        Attitude att;        //姿态
    };

    struct ImuError
    {
        Eigen::Vector3d gyrBias;  //陀螺零偏
        Eigen::Vector3d accBias;  //加速度计零偏
        Eigen::Vector3d gyrScale; //陀螺比例因子
        Eigen::Vector3d accScale; //加速度计比例因子
    };
    /*************************************************************************/


    /******************************初始配置数据结构******************************/
    struct NavState
    {
        Eigen::Vector3d pos;      //位置
        Eigen::Vector3d vel;      //速度
        Eigen::Vector3d euler;    //欧拉角
        Eigen::Vector3d gyrBias;  //陀螺零偏
        Eigen::Vector3d accBias;  //加速度计零偏
        Eigen::Vector3d gyrScale; //陀螺比例因子
        Eigen::Vector3d accScale; //加速度计比例因子
    };

    struct ImuNoise
    {
        Eigen::Vector3d gyrArw;     //陀螺角度随机游走
        Eigen::Vector3d accVrw;      //加速度计速度随机游走
        Eigen::Vector3d gyrBiasStd;  //陀螺零偏标准差
        Eigen::Vector3d accBiasStd;   //加速度计零偏标准差
        Eigen::Vector3d gyrScaleStd; //陀螺比例因子标准差
        Eigen::Vector3d accScaleStd;  //加速度计比例因子标准差
        double corr_time;             //陀螺和加速度计的零偏、比例因子的相关时间
    };

    struct Config
    {
        NavState initState;    //初始状态值
        NavState initStateStd; //初始状态标准差

        ImuNoise imuNoise; //IMU噪声参数

        double magMeasureYawStd{0}; //磁力计测量偏航角的标准差

        Eigen::Vector3d antennaLever = {0, 0, 0}; //GNSS天线杆臂
    };
    /*************************************************************************/


    /*****************************传感器数据结构*********************************/
    struct IMU
    {
        double time; //IMU采样时间
        double dt;   //时间间隔

        Eigen::Vector3d deltaTheta; //角度增量
        Eigen::Vector3d deltaVel;   //速度增量
    };

    struct Mag
    {
        double time; //磁力计采样时间
        Eigen::Vector3d mag;

        bool isUpdate; //是否更新了数据
    };

    struct GNSS
    {
        double time; //GPS采样时间

        Eigen::Vector3d blh; //纬经高
        Eigen::Vector3d std; //标准差

        bool isUpdate; //是否更新了数据
    };
    /*************************************************************************/
}

#endif //AHRS_AIDED_INS_TYPES_HPP
