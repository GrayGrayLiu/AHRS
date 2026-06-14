#pragma once

#include <stdint.h>
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

void TimeBase_Start(void);
uint64_t TimeBase_Micros(void);
uint32_t TimeBase_Millis(void);
void TimeBase_OnTimPeriodElapsed(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif
