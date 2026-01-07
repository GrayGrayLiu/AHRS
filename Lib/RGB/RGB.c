//
// Created by Gray on 2025/9/5.
//

#include "RGB.h"
#include <math.h>

// /**
//  * @brief 获取定时器时钟频率（通用接口，针对各系列通过条件编译扩展）
//  * @param htim 指向 TIM_HandleTypeDef（例如 &htim5）
//  * @return 定时器时钟频率(Hz)，返回0表示出错/不支持该系列
//  */
// static uint32_t HAL_TIM_GetClockFreqFromHandle(const TIM_HandleTypeDef *htim)
// {
//     if (htim == NULL || htim->Instance == NULL) return 0;
//
//     /* 采用简单可靠的策略：
//      * - 使用 HAL_RCC_GetPCLK1Freq / GetPCLK2Freq 得到 pclk
//      * - 再检查对应 APB prescaler 是否为 1（通过 RCC->CFGR 简单判定），若不是 1 则 timer clock = pclk * 2
//      *
//      * 说明：不同系列的 CFGR 位域位置可能不同，当前在 F4 上此实现是可信的。
//      */
// #if defined(STM32F4)
//     uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
//     uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
//
//     const TIM_TypeDef *TIMx = htim->Instance;
//     uint32_t timclk = 0;
//
//     // 判断 TIM 属于 APB1 还是 APB2
//     if (TIMx == TIM1 || TIMx == TIM8 ||
//         TIMx == TIM9 || TIMx == TIM10 || TIMx == TIM11)
//     {
//         // APB2
//         // 检查 APB2 prescaler 是否为 1：使用 CFGR 的 PPRE2，如果 PPRE2 位域不为 0b000 则分频 >=2
//         uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
//         if (ppre2 == 0) timclk = pclk2;
//         else timclk = pclk2 * 2;
//     }
//     else
//     {
//         // APB1
//         uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
//         if (ppre1 == 0) timclk = pclk1;
//         else timclk = pclk1 * 2;
//     }
//
//     return timclk;
//
// #elif defined(STM32F1)
//     // TODO: 为 F1 系列实现映射（F1 的 RCC/时钟树不同）
//     (void)htim;
//     return 0;
// #elif defined(STM32H7)
//     // TODO: H7 的时钟树更复杂（D2/D1 域分离），需要使用 HAL_RCCEx_GetPeriphCLK 或其他 API
//     (void)htim;
//     return 0;
// #else
//     #warning "HAL_TIM_GetClockFreqFromHandle 未为本 MCU 系列实现，请添加相应实现"
//     (void)htim;
//     return 0;
// #endif
// }
//
// /**
//  * @brief 返回该定时器的最大ARR（根据计数位宽 16/32）
//  * @param htim 指向 TIM_HandleTypeDef
//  * @return 最大ARR（例如 0xFFFF 或 0xFFFFFFFF）
//  */
// static uint64_t HAL_TIM_GetMaxARRFromHandle(const TIM_HandleTypeDef *htim)
// {
//     if (htim == NULL || htim->Instance == NULL) return 0;
//
// #if defined(STM32F4)
//     const TIM_TypeDef *TIMx = htim->Instance;
//     // 在 F4 中 TIM2 和 TIM5 为 32-bit，其它多数为 16-bit
//     if (TIMx == TIM2 || TIMx == TIM5)
//         return 0xFFFFFFFFULL;
//     else
//         return 0xFFFFULL;
// #else
//     // 其他系列：保守返回 0xFFFF，具体系列可扩展
//     (void)htim;
//     return 0xFFFFULL;
// #endif
// }
//
// /**
//  * @brief 自动根据 TIM_HandleTypeDef、目标 PWM 频率 和 分辨率计算 PSC/ARR
//  * @param htim 指向 TIM_HandleTypeDef (eg. &htim5)
//  * @param target_freq 目标 PWM 频率 (Hz)
//  * @param resolution_bit 分辨率位数（8 -> 256 阶）
//  * @param cfg 输出配置
//  * @retval HAL_OK / HAL_ERROR
//  */
// static HAL_StatusTypeDef PWM_AutoConfigFromHandle(const TIM_HandleTypeDef *htim,
//                                            const uint32_t target_freq,
//                                            const uint8_t resolution_bit,
//                                            PWM_ConfigTypeDef *cfg)
// {
//     if (htim == NULL || cfg == NULL) return HAL_ERROR;
//     if (resolution_bit < 4 || resolution_bit > 30) return HAL_ERROR; // 限定 4..30
//
//     const uint64_t desired_steps = (1ULL << resolution_bit);
//     const uint64_t maxARR = HAL_TIM_GetMaxARRFromHandle(htim);
//     if (maxARR == 0) return HAL_ERROR;
//
//     const uint32_t tim_clk = HAL_TIM_GetClockFreqFromHandle(htim);
//     if (tim_clk == 0) return HAL_ERROR;
//
//     uint64_t arr = desired_steps - 1;
//     uint64_t psc = 0;
//
//     // 如果 arr 超出计数器，退化为最大 ARR
//     if (arr > maxARR) arr = maxARR;
//
//     // 计算 PSC = tim_clk / (freq*(ARR+1)) - 1
//     double denom = (double)target_freq * (double)(arr + 1);
//     if (denom <= 0.0) return HAL_ERROR;
//     double psc_d = ((double)tim_clk / denom) - 1.0;
//     if (psc_d < 0.0) psc = 0;
//     else psc = (uint64_t)(psc_d + 0.5);
//
//     // 若 PSC 超过 16-bit（大多数 TIM 的 PSC 为 16-bit），尝试用更大 arr（如果可用）
//     if (psc > 0xFFFFULL)
//     {
//         if (maxARR > 0xFFFFULL)
//         {
//             // 令 PSC = 0xFFFF，解出需要的 ARR
//             uint64_t psc_limit = 0xFFFFULL;
//             double arr_needed_d = ((double)tim_clk / ((double)(psc_limit + 1) * (double)target_freq)) - 1.0;
//             if (arr_needed_d < 0.0) return HAL_ERROR;
//             uint64_t arr_needed = (uint64_t)(arr_needed_d + 0.5);
//             if (arr_needed > maxARR) return HAL_ERROR;
//             arr = arr_needed;
//             psc = psc_limit;
//         }
//         else
//         {
//             return HAL_ERROR;
//         }
//     }
//
//     uint32_t actual_psc = (uint32_t)psc;
//     uint32_t actual_arr = (uint32_t)arr;
//     uint32_t actual_freq = (uint32_t)((double)tim_clk / ((double)(actual_psc + 1) * (double)(actual_arr + 1)));
//     uint32_t actual_steps = actual_arr + 1;
//
//     cfg->PSC = actual_psc;
//     cfg->ARR = actual_arr;
//     cfg->ActualFreq = actual_freq;
//     cfg->TimerClk = tim_clk;
//     cfg->ActualSteps = actual_steps;
//
//     return HAL_OK;
// }
//
// /* ----------------- RGB 库实现 ----------------- */
//
// void RGB_Init(RGB_HandleTypeDef *hrgb, TIM_HandleTypeDef *htim,
//               const uint32_t chR, const uint32_t chG, const uint32_t chB)
// {
//     hrgb->htim = htim;
//     hrgb->ChannelR = chR;
//     hrgb->ChannelG = chG;
//     hrgb->ChannelB = chB;
//
//     hrgb->Period = __HAL_TIM_GET_AUTORELOAD(htim);
//     hrgb->R = hrgb->G = hrgb->B = 0;
//     hrgb->Brightness = 1.0f;
//
//     HAL_TIM_PWM_Start(htim, chR);
//     HAL_TIM_PWM_Start(htim, chG);
//     HAL_TIM_PWM_Start(htim, chB);
// }
//
// static void RGB_UpdatePWM(const RGB_HandleTypeDef *hrgb)
// {
//     uint32_t arr = hrgb->Period;
//     if (arr == 0) arr = 1;
//
//     const uint32_t compareR = (uint32_t)((float)hrgb->R / 255.0f * hrgb->Brightness * (float)arr + 0.5f);
//     const uint32_t compareG = (uint32_t)((float)hrgb->G / 255.0f * hrgb->Brightness * (float)arr + 0.5f);
//     const uint32_t compareB = (uint32_t)((float)hrgb->B / 255.0f * hrgb->Brightness * (float)arr + 0.5f);
//
//     __HAL_TIM_SET_COMPARE(hrgb->htim, hrgb->ChannelR, compareR);
//     __HAL_TIM_SET_COMPARE(hrgb->htim, hrgb->ChannelG, compareG);
//     __HAL_TIM_SET_COMPARE(hrgb->htim, hrgb->ChannelB, compareB);
// }
//
// void RGB_SetColor(RGB_HandleTypeDef *hrgb, const uint8_t r, const uint8_t g, const uint8_t b)
// {
//     hrgb->R = r; hrgb->G = g; hrgb->B = b;
//     RGB_UpdatePWM(hrgb);
// }
//
// void RGB_SetBrightness(RGB_HandleTypeDef *hrgb, float brightness)
// {
//     if (brightness < 0.0f) brightness = 0.0f;
//     if (brightness > 1.0f) brightness = 1.0f;
//     hrgb->Brightness = brightness;
//     RGB_UpdatePWM(hrgb);
// }
//
// void RGB_SetColorAndBrightness(RGB_HandleTypeDef *hrgb,
//                                const uint8_t r, const uint8_t g, const uint8_t b,
//                                float brightness)
// {
//     hrgb->R = r; hrgb->G = g; hrgb->B = b;
//     if (brightness < 0.0f) brightness = 0.0f;
//     if (brightness > 1.0f) brightness = 1.0f;
//     hrgb->Brightness = brightness;
//     RGB_UpdatePWM(hrgb);
// }
//
// /* ---- 自动初始化接口 ---- */
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
//                                const uint32_t target_freq,
//                                const uint8_t resolution_bit,
//                                const uint32_t chR, const uint32_t chG, const uint32_t chB)
// {
//     if (hrgb == NULL || htim == NULL) return HAL_ERROR;
//
//     PWM_ConfigTypeDef cfg;
//     if (PWM_AutoConfigFromHandle(htim, target_freq, resolution_bit, &cfg) != HAL_OK)
//     {
//         return HAL_ERROR;
//     }
//
//     // 填充 htim Init 并初始化定时器 PWM
//     htim->Init.Prescaler = cfg.PSC;
//     htim->Init.CounterMode = TIM_COUNTERMODE_UP;
//     htim->Init.Period = cfg.ARR;
//     htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//     htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//
//     if (HAL_TIM_PWM_Init(htim) != HAL_OK)
//     {
//         return HAL_ERROR;
//     }
//
//     // 配置 OC（使用默认设置）
//     TIM_OC_InitTypeDef sConfigOC = {0};
//     sConfigOC.OCMode = TIM_OCMODE_PWM1;
//     sConfigOC.Pulse = 0;
//     sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//     sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//
//     if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, chR) != HAL_OK) return HAL_ERROR;
//     if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, chG) != HAL_OK) return HAL_ERROR;
//     if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, chB) != HAL_OK) return HAL_ERROR;
//
//     HAL_TIM_PWM_Start(htim, chR);
//     HAL_TIM_PWM_Start(htim, chG);
//     HAL_TIM_PWM_Start(htim, chB);
//
//     // 初始化 RGB 句柄
//     RGB_Init(hrgb, htim, chR, chG, chB);
//     hrgb->Period = cfg.ARR; // 覆盖 Period
//
//     return HAL_OK;
// }
