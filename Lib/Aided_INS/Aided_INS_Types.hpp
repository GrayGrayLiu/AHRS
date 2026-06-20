/******************************************************************************
 * @file    Aided_INS_Types.hpp
 * @brief   定义组合惯性导航所需的数据结构
 *
 * @details
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

namespace Aided_INS_Space
{
    /******************导航解算（机械编排+KF+状态误差反馈）状态量********************/
    struct Attitude
    {
        Eigen::Vector3d euler    = Eigen::Vector3d::Zero(); //欧拉角
        Eigen::Matrix3d Cbn      = Eigen::Matrix3d::Identity(); //方向余弦矩阵（b系转到n系）
        Eigen::Quaterniond qbn   = Eigen::Quaterniond::Identity(); //姿态四元数（b系转到n系）
    };

    struct PVA
    {
        Eigen::Vector3d pos = Eigen::Vector3d::Zero(); //位置
        Eigen::Vector3d vel = Eigen::Vector3d::Zero(); //速度
        Attitude att{};                                //姿态
    };

    struct ImuError
    {
        Eigen::Vector3d gyrBias  = Eigen::Vector3d::Zero(); //陀螺零偏
        Eigen::Vector3d accBias  = Eigen::Vector3d::Zero(); //加速度计零偏
        Eigen::Vector3d gyrScale = Eigen::Vector3d::Zero(); //陀螺比例因子
        Eigen::Vector3d accScale = Eigen::Vector3d::Zero(); //加速度计比例因子
    };
    /*************************************************************************/


    /******************************初始配置数据结构******************************/
    struct NavState
    {
        Eigen::Vector3d pos      = Eigen::Vector3d::Zero(); //位置
        Eigen::Vector3d vel      = Eigen::Vector3d::Zero(); //速度
        Eigen::Vector3d euler    = Eigen::Vector3d::Zero(); //欧拉角
        Eigen::Vector3d gyrBias  = Eigen::Vector3d::Zero(); //陀螺零偏
        Eigen::Vector3d accBias  = Eigen::Vector3d::Zero(); //加速度计零偏
        Eigen::Vector3d gyrScale = Eigen::Vector3d::Zero(); //陀螺比例因子
        Eigen::Vector3d accScale = Eigen::Vector3d::Zero(); //加速度计比例因子
    };

    struct ImuNoise
    {
        Eigen::Vector3d gyrArw      = Eigen::Vector3d::Zero(); //陀螺角度随机游走
        Eigen::Vector3d accVrw      = Eigen::Vector3d::Zero(); //加速度计速度随机游走
        Eigen::Vector3d gyrBiasStd  = Eigen::Vector3d::Zero(); //陀螺零偏标准差
        Eigen::Vector3d accBiasStd  = Eigen::Vector3d::Zero(); //加速度计零偏标准差
        Eigen::Vector3d gyrScaleStd = Eigen::Vector3d::Zero(); //陀螺比例因子标准差
        Eigen::Vector3d accScaleStd = Eigen::Vector3d::Zero(); //加速度计比例因子标准差
        double corr_time{0.0};                                  //陀螺和加速度计的零偏、比例因子的相关时间
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
        double time{0.0};                            //IMU采样时间
        double dt{0.0};                              //时间间隔
        Eigen::Vector3d deltaTheta = Eigen::Vector3d::Zero(); //角度增量
        Eigen::Vector3d deltaVel   = Eigen::Vector3d::Zero(); //速度增量
    };

    struct Mag
    {
        double time{0.0};                          //磁力计采样时间
        Eigen::Vector3d mag = Eigen::Vector3d::Zero();
        bool isUpdate{false};                      //是否更新了数据
    };

    struct GNSS
    {
        double time{0.0};                          //GPS采样时间
        Eigen::Vector3d blh = Eigen::Vector3d::Zero(); //纬经高
        Eigen::Vector3d std = Eigen::Vector3d::Zero(); //标准差
        bool isUpdate{false};                      //是否更新了数据
    };
    /*************************************************************************/
}

#endif //AHRS_AIDED_INS_TYPES_HPP
