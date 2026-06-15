//
// Created by Gray on 2026/1/7.
//

#include "EXIT.h"

#include "ICM42688_Service.hpp"
#include "TimeBase.h"
#include "main.h"

/**
 * @brief  HAL GPIO EXTI 回调入口（ISR 上下文）
 * @param  GPIO_Pin 触发中断的引脚
 *
 * @note   当前只处理 IMU_INT1_Pin 的 data-ready 中断。
 *         ISR 中仅调用 NotifyDataReadyFromISR() 转发时间戳和投递事件，
 *         不执行 SPI、不 printf、不 delay。
 *         实际 FIFO 读取和处理留在 Scheduler 的普通上下文中完成。
 */
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == IMU_INT1_Pin)
    {
        icm42688_service::NotifyDataReadyFromISR(TimeBase_Micros());
    }

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
