#include "TimeBase.h"

static volatile uint32_t timebase_tim2_overflow_count;

void TimeBase_Start(void)
{
    timebase_tim2_overflow_count = 0u;
    __HAL_TIM_SET_COUNTER(&htim2, 0u);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    (void)HAL_TIM_Base_Start_IT(&htim2);
}

uint64_t TimeBase_Micros(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t high = timebase_tim2_overflow_count;
    const uint32_t low = __HAL_TIM_GET_COUNTER(&htim2);

    // Account for an overflow whose update interrupt is pending but has not
    // yet advanced the software high word.
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET
        && low < 0x80000000u) {
        ++high;
    }

    if (primask == 0u) {
        __enable_irq();
    }

    return ((uint64_t)high << 32u) | low;
}

uint32_t TimeBase_Millis(void)
{
    return (uint32_t)(TimeBase_Micros() / 1000u);
}

void TimeBase_OnTimPeriodElapsed(TIM_HandleTypeDef *htim)
{
    if (htim != NULL && htim->Instance == TIM2) {
        ++timebase_tim2_overflow_count;
    }
}
