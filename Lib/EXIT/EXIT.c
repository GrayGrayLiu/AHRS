//
// Created by Gray on 2026/1/7.
//

#include "EXIT.h"

#include "main.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // if(GPIO_Pin == TILT_ORIGIN_POINT_Pin)
    // {
    //     TiltControlData.OriginPointExitFlag = 1;
    //     TiltControlData.ExpectedOriginPointState = HAL_GPIO_ReadPin(TILT_ORIGIN_POINT_GPIO_Port, TILT_ORIGIN_POINT_Pin);
    // }
    //
    // if(GPIO_Pin == TILT_END_POINT_Pin)
    // {
    //     TiltControlData.EndPointExitFlag = 1;
    //     TiltControlData.ExpectedEndPointState = HAL_GPIO_ReadPin(TILT_END_POINT_GPIO_Port, TILT_END_POINT_Pin);
    // }
}
