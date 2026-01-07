//
// Created by Gray on 2025/9/5.
//

#ifndef TILTMOTOR3508_RGB_H
#define TILTMOTOR3508_RGB_H

#include "stm32h7xx_hal.h" // 若用其它系列，请替换为对应系列的 HAL 头

#ifdef __cplusplus
extern "C" {
#endif

// typedef struct
// {
//     TIM_HandleTypeDef *htim;  // 定时器句柄指针（实际的句柄需用户或本库初始化）
//     uint32_t ChannelR;        // 红色通道 (eg. TIM_CHANNEL_3)
//     uint32_t ChannelG;        // 绿色通道
//     uint32_t ChannelB;        // 蓝色通道
//
//     uint32_t Period;          // ARR
//     uint8_t R, G, B;          // 当前RGB (0-255)
//     float Brightness;         // 亮度(0.0-1.0)
// } RGB_HandleTypeDef;
//
// typedef struct
// {
//     uint32_t PSC;
//     uint32_t ARR;
//     uint32_t ActualFreq;   // 实际得到的 PWM 频率
//     uint32_t TimerClk;     // 定时器时钟
//     uint32_t ActualSteps;  // 实际阶数 = ARR+1
// } PWM_ConfigTypeDef;
//
// /**
//  * @brief 传统手动初始化（保留）
//  */
// void RGB_Init(RGB_HandleTypeDef *hrgb, TIM_HandleTypeDef *htim,
//               uint32_t chR, uint32_t chG, uint32_t chB);
//
// /**
//  * @brief 自动步骤：计算 PSC/ARR 并初始化 TIM、PWM 通道，然后初始化 RGB 句柄
//  * @param hrgb RGB 句柄指针（库会填充 hrgb->htim = htim）
//  * @param htim 指向要用于 PWM 的 TIM_HandleTypeDef（将被本函数初始化）
//  * @param target_freq 目标 PWM 频率 (Hz)
//  * @param resolution_bit 分辨率位数（8/10/12/...）
//  * @param chR/chG/chB 三个通道
//  * @retval HAL_OK / HAL_ERROR
//  */
// HAL_StatusTypeDef RGB_InitAuto(RGB_HandleTypeDef *hrgb,
//                                TIM_HandleTypeDef *htim,
//                                uint32_t target_freq,
//                                uint8_t resolution_bit,
//                                uint32_t chR, uint32_t chG, uint32_t chB);
//
// /* 设置颜色 / 亮度 */
// void RGB_SetColor(RGB_HandleTypeDef *hrgb, uint8_t r, uint8_t g, uint8_t b);
// void RGB_SetBrightness(RGB_HandleTypeDef *hrgb, float brightness);
// void RGB_SetColorAndBrightness(RGB_HandleTypeDef *hrgb,
//                                uint8_t r, uint8_t g, uint8_t b,
//                                float brightness);
//
// #ifdef __cplusplus
// }
// #endif

#endif //TILTMOTOR3508_RGB_H