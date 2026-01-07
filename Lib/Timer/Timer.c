//
// Created by Gray on 25-1-7.
//

#include "Timer.h"

// void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) // 输入捕获中断回调函数
// {
    // if(htim->Instance == TIM1)
    // {
    //     if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) //判断输入捕获中断通道
    //     {
    //         PWM1_Cmd.Public.InputCaptureInterruptHandler(&PWM1_Cmd, htim); //调用处理函数
    //     }
    // }
// }

// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
// {
    // if(htim->Instance == TIM1)
    // {
    //     PWM1_Cmd.Public.UpdateInterruptHandler(&PWM1_Cmd, htim); //调用处理函数
    // }
// }