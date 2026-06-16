//
// Created by Gray on 2026/1/7.
//

/**
 * @file    EXIT.h
 * @brief   STM32 EXTI 回调桥接头文件
 *
 * @details
 * 本文件作为 EXTI 模块头文件，提供 HAL GPIO 相关 include 和 ISR 边界说明。
 * 当前 MCU 仅使用 IMU INT1 引脚作为 EXTI 数据源。
 *
 * @note   HAL_GPIO_EXTI_Callback 在 EXIT.cpp 中实现，运行在 ISR 上下文。
 *         回调内仅转发 MCU 时间戳和投递 scheduler 高优先级事件，不执行 SPI、
 *         FIFO 读取或阻塞操作。
 */

#ifndef TILTMOTOR3508_EXIT_H
#define TILTMOTOR3508_EXIT_H

#include "stm32h7xx_hal.h"

#endif //TILTMOTOR3508_EXIT_H
