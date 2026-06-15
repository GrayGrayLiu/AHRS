#pragma once

#include <cstdint>

#include "ICM42688P.hpp"

// ICM42688P 应用层 Service 接口。
//
// 本层不是寄存器驱动：底层 SPI、FIFO 解码和状态机均由 ICM42688P 类负责。
// scheduler.cpp 通过 Run() 提供周期兜底执行机会，EXIT.cpp 则通过
// NotifyDataReadyFromISR() 将 IMU data-ready 事件转交给驱动和调度器。
namespace icm42688_service
{
// 面向应用调试的轻量 delta 数据。它从驱动最新完整 Sample 中提取积分增量，
// 供 scheduler 的低频输出使用，不触发 SPI 读取，也不消费底层缓存。
struct DeltaSample
{
    // 当前 FIFO batch 对应的 MCU data-ready 时间戳，单位 us。
    uint64_t timestamp_us{0u};
    // 当前 batch 的三轴角增量，单位 rad。
    float delta_angle_rad[3]{};
    // 当前 batch 的三轴比力积分速度增量，单位 m/s。
    float delta_velocity_m_s[3]{};
    // 当前 batch 的实测积分时间，单位 s。
    float delta_time_s{0.0F};
    // 当前 batch 参与积分的有效 FIFO sample 数量。
    uint16_t delta_samples{0u};
    // 驱动累计处理的有效 FIFO sample 数量，用于判断数据是否更新。
    uint32_t sample_counter{0u};
    // 当前最新温度，单位摄氏度。
    float temperature_deg_c{0.0F};
};

/**
 * @brief  EXTI ISR 桥接入口：记录 data-ready 时间戳并投递高优先级调度事件
 * @param  timestamp_us ISR 时刻的 MCU 微秒时间戳
 *
 * @note   本函数在 ISR 上下文中调用，仅转发时间戳和投递事件，不执行 SPI 访问。
 */
void NotifyDataReadyFromISR(uint64_t timestamp_us);

/**
 * @brief  统一 Service 执行入口：负责初始化节流、驱动状态机推进和可选调试输出
 *
 * @note   由 Scheduler 的高优先级事件路径和 1 kHz 周期兜底路径共用。
 *         驱动未启动时按固定节拍尝试构造和 Init，启动后调用 Update() 推进 RunImpl。
 */
void Run();

/**
 * @brief  读取驱动缓存中的完整最新 Sample，不访问 SPI
 */
[[nodiscard]] ICM42688P::Status GetLatest(ICM42688P::Sample *sample);

/**
 * @brief  从完整 Sample 中提取积分增量和调试所需字段，不访问 SPI 且不消费缓存
 */
[[nodiscard]] ICM42688P::Status GetDeltaLatest(DeltaSample *sample);
} // namespace icm42688_service
