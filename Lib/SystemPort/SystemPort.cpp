/**
 * @file    SystemPort.cpp
 * @brief   系统平台适配层实现（STM32H723 + CMSIS + TimeBase）
 *
 * @details
 * 本文件是 SystemPort 在 STM32H723 平台上的具体实现。
 * 时间委托给现有 TimeBase_Millis() / TimeBase_Micros()；
 * 临界区使用 CMSIS __get_PRIMASK / __disable_irq / __set_PRIMASK。
 *
 * 不包含任何 Scheduler 逻辑或业务模块代码。
 */

#include "SystemPort.h"

#include "TimeBase.h"
#include "stm32h7xx.h"

// ============================================================================
// 时间函数：委托给现有 TimeBase 模块
// ============================================================================

extern "C" uint32_t SystemPort_GetMillis(void)
{
    return TimeBase_Millis();
}

extern "C" uint64_t SystemPort_GetMicros(void)
{
    return TimeBase_Micros();
}

// ============================================================================
// 临界区：token-based enter/exit
// ============================================================================

/**
 * @brief  进入临界区：保存 PRIMASK 并关中断
 * @retval 调用前的 PRIMASK 值作为恢复 token
 *
 * @note   使用 CMSIS __get_PRIMASK / __disable_irq，不依赖 RTOS。
 *         返回的 token 必须由调用方传入 SystemPort_ExitCritical() 恢复。
 */
extern "C" SystemPortCriticalToken SystemPort_EnterCritical(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/**
 * @brief  退出临界区：恢复到 token 对应的 PRIMASK 状态
 * @param  token  SystemPort_EnterCritical() 的返回值
 */
extern "C" void SystemPort_ExitCritical(const SystemPortCriticalToken token)
{
    __set_PRIMASK(token);
}
