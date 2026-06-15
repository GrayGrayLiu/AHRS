//
// Created by Gray on 2026/1/7.
//

#include "scheduler.h"

#include "ICM42688_Service.hpp"
#include "TimeBase.h"

#include <stdio.h>

namespace
{
// ============================================================================
// 应用层初始化区域
// ============================================================================

// 应用层初始化入口。这里用于组织未来的应用模块初始化，不属于调度器自身
// 的任务表初始化；Scheduler_Setup() 会在调度器准备完成后调用该入口。
void Scheduler_AppSetup()
{
    // 当前没有需要集中初始化的应用模块。
}

// ============================================================================
// 响应式 / 高优先级事件调度区域
// ============================================================================

// 高优先级事件位图。ISR 或 ISR adapter 只负责置位事件，不直接执行驱动、
// SPI 或其他耗时逻辑；实际处理统一由主循环中的 HighPriorityPoll() 完成。
volatile uint32_t scheduler_hp_events = 0u;

using sched_hp_handler_t = void (*)(void);

struct sched_hp_handler_entry_t
{
    uint32_t event;
    sched_hp_handler_t handler;
};

// 静态事件处理表用于把事件位映射到普通上下文中的处理函数。后续新增响应式
// 事件时只需扩展表项，不需要把分发器重新改成多段硬编码判断。
const sched_hp_handler_entry_t scheduler_hp_handlers[] =
{
    {SCHED_HP_EVENT_IMU_DRDY, icm42688_service::Run},
};

constexpr uint8_t SCHED_HP_HANDLER_NUM =
    static_cast<uint8_t>(sizeof(scheduler_hp_handlers) / sizeof(scheduler_hp_handlers[0]));

// 原子地取出并清空事件位图。临界区保证“读取 + 清零”不可被 ISR 插入，
// 避免刚到达的事件在主循环清零位图时丢失。
uint32_t Scheduler_TakeHighPriorityEvents()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const uint32_t events = scheduler_hp_events;
    scheduler_hp_events = 0u;

    __set_PRIMASK(primask);
    return events;
}

// 在普通上下文中分发高优先级事件。函数内部的静态 polling 标志用于防止
// 事件 handler 间接触发调度器时发生嵌套分发；它具有状态保持语义，不能移除。
void Scheduler_HighPriorityPoll()
{
    static uint8_t polling = 0u;

    if (polling != 0u) {
        return;
    }

    polling = 1u;
    const uint32_t events = Scheduler_TakeHighPriorityEvents();

    for (uint8_t index = 0u; index < SCHED_HP_HANDLER_NUM; ++index) {
        if ((events & scheduler_hp_handlers[index].event) != 0u) {
            scheduler_hp_handlers[index].handler();
        }
    }

    polling = 0u;
}

// ============================================================================
// 周期轮询任务调度区域
// ============================================================================

// 记录最近一次已打印的 IMU sample counter，避免 1 Hz 调试任务重复打印同一批数据。
uint32_t imu_last_sample_counter = 0u;

// 1 Hz IMU delta 调试输出，用于观察 FIFO batch 的积分结果和实测 delta time。
// 该函数只读取 Service 缓存，不直接访问 SPI，也不参与驱动运行状态机。
void Print_ICM42688_Delta_Debug()
{
    icm42688_service::DeltaSample imu{};

    const ICM42688P::Status status = icm42688_service::GetDeltaLatest(&imu);

    if (status != ICM42688P::Status::Ok) {
        printf("[imu] st=%d\r\n", static_cast<int>(status));
        return;
    }

    if (imu.sample_counter == imu_last_sample_counter) {
        return; // 没有新数据时不重复打印。
    }

    imu_last_sample_counter = imu.sample_counter;

    if (imu.delta_time_s <= 0.0f) {
        printf("[imu] bad dt\r\n");
        return;
    }

    float gyro_rad_s[3];
    float force_m_s2[3];

    for (uint8_t i = 0; i < 3; i++) {
        gyro_rad_s[i] = imu.delta_angle_rad[i] / imu.delta_time_s;
        force_m_s2[i] = imu.delta_velocity_m_s[i] / imu.delta_time_s;
    }

    printf("t=%llu c=%lu n=%u dt=%.6f "
           "w=%.1f %.1f %.1f "
           "f=%.2f %.2f %.2f "
           "T=%.2f\r\n",
           (unsigned long long)imu.timestamp_us,
           (unsigned long)imu.sample_counter,
           (unsigned int)imu.delta_samples,
           imu.delta_time_s,
           gyro_rad_s[0] * 57.2957795f,
           gyro_rad_s[1] * 57.2957795f,
           gyro_rad_s[2] * 57.2957795f,
           force_m_s2[0],
           force_m_s2[1],
           force_m_s2[2],
           imu.temperature_deg_c);
}

void Loop_1000Hz() // 每 1 ms 执行一次。
{
    // 周期 backup 路径与 EXTI 高优先级事件共用同一个 Service 入口。
    // 即使事件通知暂时缺失，也能持续推进 IMU 初始化和 watchdog 状态机。
    icm42688_service::Run();
}

void Loop_500Hz() // 每 2 ms 执行一次。
{
}

void Loop_250Hz() // 每 4 ms 执行一次。
{
}

void Loop_200Hz() // 每 5 ms 执行一次。
{
}

void Loop_100Hz() // 每 10 ms 执行一次。
{
}

void Loop_50Hz() // 每 20 ms 执行一次。
{
}

void Loop_20Hz() // 每 50 ms 执行一次。
{
}

void Loop_10Hz() // 每 100 ms 执行一次。
{
}

void Loop_5Hz() // 每 200 ms 执行一次。
{
}

void Loop_4Hz() // 每 250 ms 执行一次。
{
}

void Loop_2Hz() // 每 500 ms 执行一次。
{
}

void Loop_1Hz() // 每 1000 ms 执行一次。
{
    Print_ICM42688_Delta_Debug();
}

void Loop_0_5Hz() // 每 2 s 执行一次。
{
}

void Loop_0_2Hz() // 每 5 s 执行一次。
{
}

void Loop_0_1Hz() // 每 10 s 执行一次。
{
}

// 周期任务静态配置表。rate_hz 和任务顺序属于当前调度行为，本轮保持不变；
// Scheduler_Setup() 根据频率计算 interval_ticks。
sched_task_t sched_tasks[] =
{
    {Loop_1000Hz, 1000, 0, 0},
    {Loop_500Hz, 500, 0, 0},
    {Loop_250Hz, 250, 0, 0},
    {Loop_200Hz, 200, 0, 0},
    {Loop_100Hz, 100, 0, 0},
    {Loop_50Hz, 50, 0, 0},
    {Loop_20Hz, 20, 0, 0},
    {Loop_10Hz, 10, 0, 0},
    {Loop_5Hz, 5, 0, 0},
    {Loop_4Hz, 4, 0, 0},
    {Loop_2Hz, 2, 0, 0},
    {Loop_1Hz, 1, 0, 0},
    {Loop_0_5Hz, 0.5f, 0, 0},
    {Loop_0_2Hz, 0.2f, 0, 0},
    {Loop_0_1Hz, 0.1f, 0, 0},
};

constexpr size_t TASK_NUM = sizeof(sched_tasks) / sizeof(sched_tasks[0]);
} // namespace

// 供 ISR 或 ISR adapter 调用：仅原子置位事件，不在中断上下文执行 handler。
extern "C" void Scheduler_PostHighPriorityEventFromISR(const uint32_t event)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    scheduler_hp_events |= event;
    __set_PRIMASK(primask);
}

// 调度器初始化负责计算周期任务的 tick 间隔；Scheduler_AppSetup() 则是独立的
// 应用层初始化扩展点，两者职责不同。
extern "C" void Scheduler_Setup(void)
{
    for (size_t index = 0u; index < TASK_NUM; ++index) {
        sched_tasks[index].interval_ticks = TICK_PER_SECOND / sched_tasks[index].rate_hz;

        if (sched_tasks[index].interval_ticks < 1) {
            sched_tasks[index].interval_ticks = 1;
        }
    }

    Scheduler_AppSetup();
}

// cooperative 主循环入口：在遍历周期任务前、任务检查前以及任务执行前后插入
// 高优先级事件服务点，使 ISR 投递的事件尽快在普通上下文得到处理。周期任务仍按
// TimeBase_Millis() 和 interval_ticks 判断是否到期，原有调度顺序保持不变。
extern "C" void Scheduler_Run(void)
{
    Scheduler_HighPriorityPoll();

    for (size_t index = 0u; index < TASK_NUM; ++index) {
        Scheduler_HighPriorityPoll();
        const uint32_t t_now = TimeBase_Millis();

        if (t_now - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks) {
            sched_tasks[index].last_run = t_now;
            Scheduler_HighPriorityPoll();
            sched_tasks[index].task_func();
            Scheduler_HighPriorityPoll();
        }
    }
}
