/**
 * @file    SystemPort.h
 * @brief   系统平台适配层接口（C ABI）
 *
 * @details
 * 本模块是独立于 Scheduler 和业务模块的底层系统工具层，提供：
 *   - 毫秒/微秒时间戳
 *   - token-based 临界区 enter/exit
 *
 * 调用方可直接使用本模块保护 ISR ↔ 普通上下文共享状态。
 * Scheduler 后续通过 port callback 间接使用本模块提供的函数，其核心代码
 * 不直接 include 本头文件也不直接依赖 TimeBase / CMSIS。
 *
 * C 和 C++ 文件均可 include。
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 时间函数
// ============================================================================

/**
 * @brief  获取系统毫秒时间戳
 * @retval 系统启动以来的毫秒数（32-bit，约 49.7 天回绕）
 * @note   可在 ISR 和普通上下文中安全调用。
 */
uint32_t SystemPort_GetMillis(void);

/**
 * @brief  获取系统微秒时间戳
 * @retval 系统启动以来的微秒数（64-bit，工程上不回绕）
 * @note   可在 ISR 和普通上下文中安全调用。
 */
uint64_t SystemPort_GetMicros(void);

// ============================================================================
// 临界区（token-based）
// ============================================================================

/**
 * @brief  临界区状态 token
 *
 * 在 STM32/Cortex-M 上，token 即为 __get_PRIMASK() 的返回值。
 * enter_critical() 返回进入临界区前的 PRIMASK 值并关中断；
 * exit_critical(token) 用 token 恢复之前的中断状态。
 *
 * 典型用法：
 *   SystemPortCriticalToken token = SystemPort_EnterCritical();
 *   // … 访问 ISR 共享变量 …
 *   SystemPort_ExitCritical(token);
 */
typedef uint32_t SystemPortCriticalToken;

/**
 * @brief  进入临界区：关中断并返回调用前的 PRIMASK 值
 * @retval 调用前的中断状态 token（用于 exit_critical 恢复）
 * @note   可嵌套：每层持有自己的 token 并在退出时恢复。
 *         不可在 ISR 中调用（ISR 已被硬件自动屏蔽同级中断）。
 */
SystemPortCriticalToken SystemPort_EnterCritical(void);

/**
 * @brief  退出临界区：恢复到 token 对应的中断状态
 * @param  token  SystemPort_EnterCritical() 的返回值
 */
void SystemPort_ExitCritical(SystemPortCriticalToken token);

#ifdef __cplusplus
}
#endif
