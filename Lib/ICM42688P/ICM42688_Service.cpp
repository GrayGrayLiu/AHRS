/**
 * @file    ICM42688_Service.cpp
 * @brief   ICM42688P 应用层 Service 实现
 *
 * @details
 * Service 层职责：
 *   - 拥有并管理全工程唯一的 ICM42688P 驱动实例；
 *   - 提供 ISR 桥接入口 NotifyDataReadyFromISR()；
 *   - 提供 Run() 作为 scheduler 的统一执行入口（初始化推进 + Update + 调试输出）；
 *   - 提供 GetLatest/GetDeltaLatest 缓存读取接口；
 *   - 用 RGB LED 指示驱动状态。
 *
 * 不负责：
 *   - 任何 SPI 寄存器操作、FIFO packet 解析、惯性积分、状态机实现。
 */

#include "ICM42688_Service.hpp"

#include <new>
#include <cstdint>
#include <stdio.h>

#include "TimeBase.h"
#include "main.h"
#include "scheduler.h"

// ============================================================================
// 编译开关和 Service 参数
// ============================================================================

// 驱动运行期完整 sample 输出默认关闭，避免阻塞式串口输出影响 FIFO 消费。
#define ICM42688_RUNTIME_SAMPLE_PRINT_ENABLE 0

enum
{
    ICM42688_INIT_SERVICE_INTERVAL_MS = 100u,   // Service::Run() 中初始化尝试节拍
    ICM42688_INIT_RETRY_INTERVAL_MS = 1000u,    // Init() 失败后重试间隔
    ICM42688_DEBUG_SERVICE_INTERVAL_MS = 1000u,  // PrintLatest 调试输出节拍
    ICM42688_PRINT_INTERVAL_MS = 10000u,         // PrintLatest 内部 printf 节流
};

namespace
{
// ============================================================================
// 唯一 ICM42688P 实例与 Service 状态
// ============================================================================

// ICM42688P 需要在构造时绑定 SPI 和 CS，不能默认构造。裸机环境不使用动态
// 内存，因此在静态存储上通过 placement new 构造全工程唯一的驱动实例。
// 生命周期：在第一次 ServiceInit() 调用时构造，之后持续存在直到系统复位。
alignas(ICM42688P) uint8_t icm42688_storage[sizeof(ICM42688P)]{};
ICM42688P *icm42688 = nullptr;

// ── 驱动生命周期标志 ──
// bound：   C++ 对象已构造并绑定 SPI/CS（placement new 完成）
// started： Init() 已成功，驱动状态机已启动，可由 Update() 继续推进
bool icm42688_bound = false;
bool icm42688_started = false;

// ── 驱动运行状态与调度节拍 ──
uint8_t icm42688_running = 0u;                      // 0=未进入正常 FIFO_READ；1=正在运行
uint32_t icm42688_last_init_service_tick = 0u;      // 上次 ServiceInit 尝试的毫秒时间戳
uint32_t icm42688_next_init_tick = 0u;              // 下次允许执行 ServiceInit 的时间
uint32_t icm42688_last_debug_service_tick = 0u;     // 上次 PrintLatest 调试检查时间
uint32_t icm42688_last_print_tick = 0u;             // 上次实际 printf 输出时间（节流用）
ICM42688P::Status icm42688_last_status = ICM42688P::Status::Ok; // 最近一次 Update() 返回码
uint8_t icm42688_service_polling = 0u;              // Run() 重入保护标志

// ============================================================================
// 通用小工具函数
// ============================================================================

// 使用有符号差值处理 uint32_t 毫秒计数回绕。
bool TickReached(const uint32_t now, const uint32_t target)
{
    return static_cast<int32_t>(now - target) >= 0;
}

// 设置板载 RGB LED。
// LED 含义：
//   蓝 — 初始化/非运行状态
//   红 — 错误状态
//   绿 — 正常运行状态
void SetLed(const GPIO_PinState red, const GPIO_PinState green, const GPIO_PinState blue)
{
    HAL_GPIO_WritePin(RGB_R_GPIO_Port, RGB_R_Pin, red);
    HAL_GPIO_WritePin(RGB_G_GPIO_Port, RGB_G_Pin, green);
    HAL_GPIO_WritePin(RGB_B_GPIO_Port, RGB_B_Pin, blue);
}

// ============================================================================
// 初始化流程
// ============================================================================

// 只负责唯一对象的构造和 Init() 启动流程。底层 RESET、CONFIGURE、FIFO_READ
// 等硬件生命周期仍由 ICM42688P::RunImpl() 在后续 Update() 调用中推进。
// 初始化节拍由 ICM42688_INIT_SERVICE_INTERVAL_MS 控制（Service::Run 内判断）；
// Init 失败后按 ICM42688_INIT_RETRY_INTERVAL_MS 重试。
void ServiceInit(const uint32_t now)
{
    if (icm42688_started || !TickReached(now, icm42688_next_init_tick)) {
        return;
    }

    // 初始化中：蓝灯
    SetLed(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);

    if (!icm42688_bound) {
        printf("[ICM42688P] bind...\r\n");
        icm42688 = ::new (static_cast<void *>(icm42688_storage)) ICM42688P(&hspi1, IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin);
        icm42688_bound = true;
        printf("[ICM42688P] bind ok\r\n");
    }

    printf("[ICM42688P] init...\r\n");
    icm42688_last_status = icm42688->Init();

    if (icm42688_last_status != ICM42688P::Status::Ok) {
        // 初始化失败：红灯
        SetLed(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
        printf("[ICM42688P] init failed status=%ld\r\n",
               static_cast<long>(icm42688_last_status));
        icm42688_next_init_tick = now + ICM42688_INIT_RETRY_INTERVAL_MS;
        return;
    }

    icm42688_started = true;
    // 初始化成功但配置未完成：蓝灯（等待 RunImpl 推进到 FIFO_READ）
    SetLed(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);
}

// ============================================================================
// 周期更新与状态指示
// ============================================================================

// 推进底层驱动状态机，并根据最新 sample 的 configured 状态和调用结果更新
// running 标志与 RGB LED。
// 该函数不实现任何寄存器或 FIFO 处理逻辑。
// LED 状态：
//   蓝灯 — 未配置完成（configured == false）
//   红灯 — 驱动返回错误（非 Ok 且非 NoData）
//   绿灯 — 正常运行（configured && Ok/NoData）
void Update()
{
    if (!icm42688_started || icm42688 == nullptr) {
        return;
    }

    icm42688_last_status = icm42688->Update();
    ICM42688P::Sample sample{};
    const ICM42688P::Status sample_status = icm42688->GetLatest(sample);
    const bool configured = sample_status == ICM42688P::Status::Ok && sample.configured;

    if (!configured) {
        icm42688_running = 0u;

        if (icm42688_last_status == ICM42688P::Status::Ok
            || icm42688_last_status == ICM42688P::Status::NoData) {
            // 未配置但在正常推进中：蓝灯
            SetLed(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);

        } else {
            // 错误：红灯
            SetLed(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
        }

        return;
    }

    if (icm42688_running == 0u) {
        icm42688_running = 1u;
        printf("[ICM42688P] init ok, interrupt FIFO update started\r\n");
    }

    if (icm42688_last_status == ICM42688P::Status::Ok
        || icm42688_last_status == ICM42688P::Status::NoData) {
        // 正常运行：绿灯
        SetLed(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);

    } else {
        // 运行中错误：红灯
        SetLed(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
    }
}

// ============================================================================
// 可选调试输出
// ============================================================================

// 输出完整 sample 摘要。编译开关默认关闭，以免阻塞式 printf 影响高频 FIFO。
// 仅在 icm42688_running 为 true 且配置完成时输出。
void PrintLatest()
{
#if ICM42688_RUNTIME_SAMPLE_PRINT_ENABLE == 0
    return;
#endif

    if (icm42688_running == 0u) {
        return;
    }

    ICM42688P::Sample sample{};
    const ICM42688P::Status status = icm42688_service::GetLatest(&sample);
    const uint32_t now = TimeBase_Millis();

    if (!TickReached(now, icm42688_last_print_tick + ICM42688_PRINT_INTERVAL_MS)) {
        return;
    }

    icm42688_last_print_tick = now;

    if (status != ICM42688P::Status::Ok || !sample.data_valid) {
        printf("[ICM42688P] unavailable get=%ld st=%ld e=%lu\r\n",
               static_cast<long>(status),
               static_cast<long>(icm42688_last_status),
               static_cast<unsigned long>(sample.error_counter));
        return;
    }

    const long accel_z_milli = static_cast<long>(sample.accel_m_s2[2] * 1000.0f);
    const long gyro_z_milli = static_cast<long>(sample.gyro_rad_s[2] * 1000.0f);
    const long delta_time_ms = static_cast<long>(sample.delta_time_s * 1000.0f);
    const long delta_angle_z_micro = static_cast<long>(sample.delta_angle_rad[2] * 1000000.0f);
    const long delta_velocity_z_micro = static_cast<long>(sample.delta_velocity_m_s[2] * 1000000.0f);

    printf("[ICM42688P] st=%ld src=%u cnt=%u n=%u hdr=0x%02X s=%lu e=%lu "
           "irq=%lu dt=%ld dthz=%ld dvz=%ld az=%ld gz=%ld\r\n",
           static_cast<long>(icm42688_last_status),
           static_cast<unsigned int>(sample.data_source),
           static_cast<unsigned int>(sample.fifo_count_bytes),
           static_cast<unsigned int>(sample.fifo_valid_packets),
           static_cast<unsigned int>(sample.fifo_header),
           static_cast<unsigned long>(sample.sample_counter),
           static_cast<unsigned long>(sample.error_counter),
           static_cast<unsigned long>(sample.interrupt_counter),
           delta_time_ms,
           delta_angle_z_micro,
           delta_velocity_z_micro,
           accel_z_milli,
           gyro_z_milli);
}

} // namespace

// ============================================================================
// 对外 Service 接口
// ============================================================================

/**
 * @brief  EXTI ISR 桥接入口：将 MCU 时间戳转发到 driver 并投递高优先级事件
 * @param  timestamp_us ISR 时刻的 MCU 微秒时间戳
 *
 * @note   本函数在 ISR 上下文中调用，不读 SPI、不打印、不延时。
 *         实际 FIFO 处理发生在普通上下文中的 Run() → Update() 链路内。
 */
void icm42688_service::NotifyDataReadyFromISR(const uint64_t timestamp_us)
{
    if (icm42688 != nullptr) {
        icm42688->DataReady(timestamp_us);
    }

    Scheduler_PostHighPriorityEventFromISR(SCHED_HP_EVENT_IMU_DRDY);
}

/**
 * @brief  Service 统一执行入口
 *
 * @note   由 Scheduler 的高优先级事件路径和 1 kHz 周期兜底路径共用。
 *         内部 polling 标志防止嵌套重入（如 handler 间接触发 scheduler）。
 *
 *         执行阶段：
 *         1. 驱动尚未启动时按固定节拍尝试构造和 Init；
 *         2. 启动后调用 Update() 推进 RunImpl；
 *         3. 按独立节拍进入默认关闭的调试输出路径。
 */
void icm42688_service::Run()
{
    // 防重入保护：如果 scheduler 在 handler 中间接触发本函数，直接返回。
    if (icm42688_service_polling != 0u) {
        return;
    }

    icm42688_service_polling = 1u;
    const uint32_t now = TimeBase_Millis();

    // 阶段 1：驱动尚未完成 Init 时，按节拍尝试 ServiceInit。
    if (!icm42688_started) {
        if (TickReached(now, icm42688_last_init_service_tick + ICM42688_INIT_SERVICE_INTERVAL_MS)) {
            icm42688_last_init_service_tick = now;
            ServiceInit(now);
        }

        icm42688_service_polling = 0u;
        return;
    }

    // 阶段 2：推进底层驱动状态机。
    Update();

    // 阶段 3：按独立节拍触发调试输出（默认由编译开关关闭）。
    if (TickReached(now, icm42688_last_debug_service_tick + ICM42688_DEBUG_SERVICE_INTERVAL_MS)) {
        icm42688_last_debug_service_tick = now;
        PrintLatest();
    }

    icm42688_service_polling = 0u;
}

/**
 * @brief  只读返回 driver 最新完整 Sample 缓存，不访问 SPI
 */
ICM42688P::Status icm42688_service::GetLatest(ICM42688P::Sample *sample)
{
    if (!icm42688_bound || !icm42688_started || icm42688 == nullptr || sample == nullptr) {
        return ICM42688P::Status::InvalidArgument;
    }

    return icm42688->GetLatest(*sample);
}

/**
 * @brief  从完整 Sample 中提取积分增量和调试字段，只读缓存、不访问 SPI
 */
ICM42688P::Status icm42688_service::GetDeltaLatest(DeltaSample *sample)
{
    if (sample == nullptr) {
        return ICM42688P::Status::InvalidArgument;
    }

    ICM42688P::Sample latest{};
    const ICM42688P::Status status = GetLatest(&latest);

    if (status != ICM42688P::Status::Ok) {
        return status;
    }

    if (!latest.configured || !latest.data_valid) {
        return ICM42688P::Status::NoData;
    }

    sample->timestamp_us = latest.timestamp_us;

    for (uint8_t axis = 0u; axis < 3u; ++axis) {
        sample->delta_angle_rad[axis] = latest.delta_angle_rad[axis];
        sample->delta_velocity_m_s[axis] = latest.delta_velocity_m_s[axis];
    }

    sample->delta_time_s = latest.delta_time_s;
    sample->delta_samples = latest.delta_samples;
    sample->sample_counter = latest.sample_counter;
    sample->temperature_deg_c = latest.temperature_deg_c;
    return ICM42688P::Status::Ok;
}
