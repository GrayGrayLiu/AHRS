/**
 * @file    TimeBase.cpp
 * @brief   全系统时间基准实现（TIM2 硬件定时器 + 软件溢出扩展）
 *
 * @note    TimeBase.h 保持 .h 后缀作为 C ABI 边界，本文件以 C++ 编译。
 *          TimeBase.h 中的 extern "C" 声明与本文件的定义自动匹配 C linkage。
 */

#include "TimeBase.h"

// ── 软件 32 位溢出高字 ──
// TIM2 CNT 每溢出一次，TimeBase_OnTimPeriodElapsed() 在 ISR 中加 1。
// TimeBase_Micros() 用临界区保护读取 {high, CNT} 对，确保 64 位原子性。
static volatile uint32_t timebase_tim2_overflow_count;

/**
 * @brief  启动 TIM2 并启用更新中断
 * @note   复位软件溢出计数和硬件 CNT，清零更新标志后启动。
 */
void TimeBase_Start()
{
    timebase_tim2_overflow_count = 0u;
    __HAL_TIM_SET_COUNTER(&htim2, 0u);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    (void)HAL_TIM_Base_Start_IT(&htim2);
}

/**
 * @brief  原子读取 64 位微秒时间戳
 *
 * @note   临界区保护读取顺序：
 *         1. 关中断；
 *         2. 读软件 high（TIM2 溢出次数）；
 *         3. 读硬件 CNT（TIM2 当前计数值）；
 *         4. 检查 TIM2 更新中断标志：如果标志已置位但 CNT 较小，
 *            说明在步骤 2-3 之间发生了溢出而软件 high 尚未更新，
 *            此时对 high 加 1 补偿；
 *         5. 恢复原始中断状态。
 *         64 位结果 = (high << 32) | CNT。
 *
 *         该设计允许在 ISR 和普通上下文中都可安全调用。
 */
uint64_t TimeBase_Micros()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t high = timebase_tim2_overflow_count;
    const uint32_t low = __HAL_TIM_GET_COUNTER(&htim2);

    // 补偿在 {读 high, 读 CNT} 之间发生的溢出：
    // 若更新中断标志已置位且 CNT 处于下半区，表明溢出已发生但
    // TIM2_IRQHandler 尚未递增软件高字。
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET
        && low < 0x80000000u) {
        ++high;
    }

    if (primask == 0u) {
        __enable_irq();
    }

    return (static_cast<uint64_t>(high) << 32u) | low;
}

/**
 * @brief  毫秒时间戳（微秒值 / 1000）
 */
uint32_t TimeBase_Millis()
{
    return static_cast<uint32_t>(TimeBase_Micros() / 1000u);
}

/**
 * @brief  TIM2 更新中断 ISR 回调
 * @note   仅处理 TIM2；在 ISR 上下文中执行，只递增溢出计数，不做任何阻塞操作。
 */
void TimeBase_OnTimPeriodElapsed(TIM_HandleTypeDef *htim)
{
    if (htim != nullptr && htim->Instance == TIM2) {
        ++timebase_tim2_overflow_count;
    }
}
