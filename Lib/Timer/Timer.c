//
// Created by Gray on 25-1-7.
//

#include "Timer.h"
#include "TimeBase.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    TimeBase_OnTimPeriodElapsed(htim);

    // Future timer services should be dispatched here.
}
