/**
 * @file    CriticalSectionGuard.hpp
 * @brief   C++ RAII 临界区守卫（基于 SystemPort）
 *
 * @details
 * 构造时进入临界区，析构时自动退出。不可复制、不可移动。
 * 适用于 C++ 业务模块（ICM42688P、UART 等）中需要保护 ISR 共享状态的代码块。
 *
 * 用法示例：
 *   {
 *       CriticalSectionGuard lock;
 *       data_ready_pending_ = 0u;
 *       // …
 *   }  // lock 析构，自动退出临界区
 *
 * @note   不引入动态内存，不依赖 Scheduler。
 *         头文件仅包含 SystemPort.h，可在任何 C++ 翻译单元中使用。
 */

#pragma once

#include "SystemPort.h"

class CriticalSectionGuard
{
public:
    CriticalSectionGuard()
        : token_(SystemPort_EnterCritical())
    {
    }

    ~CriticalSectionGuard()
    {
        SystemPort_ExitCritical(token_);
    }

    // 不可复制
    CriticalSectionGuard(const CriticalSectionGuard&) = delete;
    CriticalSectionGuard& operator=(const CriticalSectionGuard&) = delete;

    // 不可移动
    CriticalSectionGuard(CriticalSectionGuard&&) = delete;
    CriticalSectionGuard& operator=(CriticalSectionGuard&&) = delete;

private:
    SystemPortCriticalToken token_{};
};
