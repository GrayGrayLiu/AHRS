/**
 * @file    Timer.cpp
 * @brief   HAL TIM 回调桥接（C++ 编译单元）
 *
 * @details
 * 本文件以 C++ 编译，提供 HAL 弱回调 HAL_TIM_PeriodElapsedCallback 的实现。
 * 当前仅将 TIM 更新中断转发到 TimeBase 模块的软件溢出计数。
 *
 * Timer.h 保持 .h 后缀作为 C ABI 边界，供 C 和 C++ 文件共同 include。
 *
 * @note   HAL_TIM_PeriodElapsedCallback 由 HAL_TIM_IRQHandler 在 ISR 中调用，
 *         必须具有 C linkage（extern "C"），否则 C 编译的 HAL 库无法通过
 *         弱符号机制覆盖默认实现。
 */

//
// Created by Gray on 25-1-7.
//

#include "Timer.h"
#include "TimeBase.h"

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    TimeBase_OnTimPeriodElapsed(htim);

    // Future timer services should be dispatched here.
}
