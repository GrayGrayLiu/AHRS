/**
 * @file    Uart.h
 * @brief   UART 串口通信模块 C ABI 接口
 *
 * @details
 * 本头文件保持 .h 后缀作为 C/C++ 共用 C ABI 边界。
 * 当前 Uart 模块为预留模块，暂无对外 C ABI 声明和有效功能代码。
 * 后续如需对外暴露 UART 初始化、收发或回调函数，应在本文件的 extern "C" 块中声明。
 *
 * @note   Uart.cpp 以 C++ 编译，现有的旧 UART buffer 和回调实现均已注释。
 */

//
// Created by Gray on 2025/1/7.
//

#ifndef ADJUSTMOTORSPEED_USART_H
#define ADJUSTMOTORSPEED_USART_H

#include "stm32h7xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 后续 UART 对外 C ABI 声明放在此处 */

#ifdef __cplusplus
}
#endif

#endif //ADJUSTMOTORSPEED_USART_H
