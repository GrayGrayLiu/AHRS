/**
 * @file    Timer.h
 * @brief   TIM 硬件定时器服务模块 C ABI 接口
 *
 * @details
 * 本头文件保持 .h 后缀作为 C/C++ 共用 C ABI 边界。
 * 当前 Timer 模块仅作为 HAL TIM 回调到 TimeBase 的桥接，暂无对外 C ABI 声明。
 * 后续如需对外暴露定时器服务函数，应在本文件的 extern "C" 块中声明。
 *
 * @note   Timer.cpp 以 C++ 编译，HAL 回调函数在 .cpp 中以 extern "C" 定义。
 */

//
// Created by Gray on 25-1-7.
//

#ifndef LINEARACTUATOR_TIMER_H
#define LINEARACTUATOR_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

/* 后续 Timer 对外 C ABI 声明放在此处 */

#ifdef __cplusplus
}
#endif

#endif //LINEARACTUATOR_TIMER_H
