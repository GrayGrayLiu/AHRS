/**
 * @file    ICM42688_Service.hpp
 * @brief   ICM42688P 应用层 Service 接口
 *
 * @details
 * 本层是 ICM42688P 驱动的唯一实例所有者和对外接口层。
 * - SPI、FIFO 解码、状态机和积分均由 ICM42688P 类负责，Service 不直接处理。
 * - Service 拥有全工程唯一的 ICM42688P 实例（placement new 在静态存储上构造）。
 * - Run() 由 scheduler 的高优事件路径和 1 kHz 周期兜底路径共用。
 * - NotifyDataReadyFromISR() 在 EXTI ISR 中调用，是 ISR → 普通上下文桥接入口。
 *
 * ISR 边界：
 *   NotifyDataReadyFromISR → ICM42688P::DataReady (锁存 timestamp, 置 pending)
 *                         → Scheduler_PostHighPriorityEventFromISR (置位事件位图)
 *   ISR 中不执行 SPI、不读 FIFO、不 printf、不 delay。
 */

#pragma once

#include <cstdint>

#include "ICM42688P.hpp"
#include "scheduler.h"

namespace icm42688_service
{

// 面向应用调试的轻量 delta 数据。从驱动最新完整 Sample 中提取积分增量，
// 供 scheduler 的低频输出使用，不触发 SPI 读取，也不消费底层缓存。
struct DeltaSample
{
    uint64_t timestamp_us{0u};          // 当前 FIFO batch 对应的 MCU data-ready 时间戳，us
    float delta_angle_rad[3]{};         // 本 batch 三轴角增量，rad
    float delta_velocity_m_s[3]{};      // 本 batch 三轴比力速度增量，m/s
    float delta_time_s{0.0F};           // 本 batch 实测积分时间，s
    uint16_t delta_samples{0u};         // 本 batch 参与积分的有效 FIFO sample 数
    uint32_t sample_counter{0u};        // 驱动累计有效 FIFO sample 数（用于判断数据是否更新）
    float temperature_deg_c{0.0F};      // 当前最新温度，°C
};

/**
 * @brief  EXTI ISR 桥接入口：记录 data-ready 时间戳并投递高优先级调度事件
 * @param  timestamp_us ISR 时刻的 MCU 微秒时间戳
 *
 * @note   本函数在 ISR 上下文中调用，仅转发时间戳和投递事件，不执行 SPI 访问。
 *         ICM42688P::DataReady() 只锁存 timestamp 和置 pending，不做 FIFO 读取。
 */
void NotifyDataReadyFromISR(uint64_t timestamp_us);

/**
 * @brief  统一 Service 执行入口：负责初始化推进、驱动状态机运行和可选调试输出
 *
 * @note   由 Scheduler 的高优先级事件路径和 1 kHz 周期兜底路径共用。
 *         驱动未启动时按固定节拍尝试构造和 Init，启动后调用 Update() 推进 RunImpl。
 *         内部 polling 标志防止嵌套重入。
 */
void Run();

/**
 * @brief  向 Service 注入 imu_drdy 等关键任务的 Scheduler task ID。
 * @note   由 scheduler_app_tasks 注册成功后调用，用于 Run() 后续调用 Scheduler_ScheduleNow/Delayed。
 */
void SetSchedulerTaskId(SchedulerTaskId task_id);

/**
 * @brief  读取驱动缓存中的完整最新 Sample，不访问 SPI，不消费缓存
 */
[[nodiscard]] ICM42688P::Status GetLatest(ICM42688P::Sample *sample);

/**
 * @brief  从完整 Sample 中提取积分增量和调试所需字段，不访问 SPI 且不消费缓存
 */
[[nodiscard]] ICM42688P::Status GetDeltaLatest(DeltaSample *sample);

} // namespace icm42688_service
