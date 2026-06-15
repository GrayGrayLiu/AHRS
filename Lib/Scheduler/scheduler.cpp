//
// Created by Gray on 2026/1/7.
//

#include "scheduler.h"
#include "ICM42688_Service.hpp"
#include "TimeBase.h"

#include <stdio.h>

namespace
{
// App initialization section.
static void Scheduler_AppSetup()
{
    // Application-layer initialization hook.
}

// High-priority / event-driven scheduler section.
//全局事件位图，每一位代表一种高优先级事件，事件定义在定义在 scheduler.h
static volatile uint32_t scheduler_hp_events = 0u;

typedef void (*sched_hp_handler_t)(void);

typedef struct
{
    uint32_t event;
    sched_hp_handler_t handler;
} sched_hp_handler_entry_t;

static const sched_hp_handler_entry_t scheduler_hp_handlers[] =
{
    {SCHED_HP_EVENT_IMU_DRDY, icm42688_service::Run},
};

static const uint8_t SCHED_HP_HANDLER_NUM =
    (uint8_t)(sizeof(scheduler_hp_handlers) / sizeof(scheduler_hp_handlers[0]));
}

//事件投递函数，通常由 ISR 或 ISR-adapter 调用,把对应事件位设置为 1
extern "C" void Scheduler_PostHighPriorityEventFromISR(const uint32_t event)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    scheduler_hp_events |= event;
    __set_PRIMASK(primask);
}

namespace
{
//取出并清空事件位图
static uint32_t Scheduler_TakeHighPriorityEvents(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    //读取当前有哪些高优先级事件
    const uint32_t events = scheduler_hp_events;

    //把事件位图清零
    scheduler_hp_events = 0u;

    //返回刚刚取出的 events
    __set_PRIMASK(primask);
    return events;
}

//高优先级事件分发器
static void Scheduler_HighPriorityPoll(void)
{
    //通过polling变量防重入
    static uint8_t polling = 0u;

    if (polling != 0u) {
        return;
    }

    polling = 1u;

    //取出当前所有 high-priority events
    const uint32_t events = Scheduler_TakeHighPriorityEvents();

    //根据静态注册表把事件分发到对应模块的 service run 函数
    for (uint8_t index = 0u; index < SCHED_HP_HANDLER_NUM; ++index) {
        if ((events & scheduler_hp_handlers[index].event) != 0u) {
            scheduler_hp_handlers[index].handler();
        }
    }

    polling = 0u;
}

// Periodic polling scheduler section.
static uint32_t imu_last_sample_counter = 0;
void Print_ICM42688_Delta_Debug(void)
{
    icm42688_service::DeltaSample imu{};

    const ICM42688P::Status status = icm42688_service::GetDeltaLatest(&imu);

    if (status != ICM42688P::Status::Ok) {
        printf("[imu] st=%d\r\n", static_cast<int>(status));
        return;
    }

    if (imu.sample_counter == imu_last_sample_counter) {
        return; // 没有新数据，不重复打印
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

static void Loop_1000Hz(void) //1ms执行一次
{
    icm42688_service::Run();
}

static void Loop_500Hz(void) //2ms执行一次
{
}

static void Loop_250Hz(void) //4ms执行一次
{
}

static void Loop_200Hz(void) //5ms执行一次
{
}

static void Loop_100Hz(void) //10ms执行一次
{
}

static void Loop_50Hz(void) //20ms执行一次
{
}

static void Loop_20Hz(void) //50ms执行一次
{
}

static void Loop_10Hz(void) //100ms执行一次
{
}

static void Loop_5Hz(void) //200ms执行一次
{
}

static void Loop_4Hz(void) //250ms执行一次
{
}

static void Loop_2Hz(void) //500ms执行一次
{
}

static void Loop_1Hz(void) //1000ms执行一次
{
    Print_ICM42688_Delta_Debug();
}

static void Loop_0_5Hz(void) //2s执行一次
{
}

static void Loop_0_2Hz(void) //5s执行一次
{
}

static void Loop_0_1Hz(void) //10s执行一次
{
}

//////////////////////////////////////////////////////////////////////
//调度器初始化
//////////////////////////////////////////////////////////////////////
//系统任务配置，创建不同执行频率的“线程”
static sched_task_t sched_tasks[] =
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
//根据数组长度，判断线程数量
#define TASK_NUM (sizeof(sched_tasks) / sizeof(sched_task_t))
}

extern "C" void Scheduler_Setup(void)
{
    uint8_t index = 0;
    //初始化任务表
    for (index = 0; index < TASK_NUM; index++) {
        //计算每个任务的延时周期数
        sched_tasks[index].interval_ticks = TICK_PER_SECOND / sched_tasks[index].rate_hz;
        //最短周期为1，也就是1ms
        if (sched_tasks[index].interval_ticks < 1) {
            sched_tasks[index].interval_ticks = 1;
        }
    }

    Scheduler_AppSetup();
}

//这个函数放到main函数的while(1)中，不停判断是否有线程应该执行
extern "C" void Scheduler_Run(void)
{
    uint8_t index = 0;
    Scheduler_HighPriorityPoll();
    //循环判断所有线程，是否应该执行

    for (index = 0; index < TASK_NUM; index++) {
        Scheduler_HighPriorityPoll();
        //获取系统当前时间，单位MS
        uint32_t tnow = TimeBase_Millis();
        //进行判断，如果当前时间减去上一次执行的时间，大于等于该线程的执行周期，则执行线程
        if (tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks) {

            //更新线程的执行时间，用于下一次判断
            sched_tasks[index].last_run = tnow;
            Scheduler_HighPriorityPoll();
            //执行线程函数，使用的是函数指针
            sched_tasks[index].task_func();
            Scheduler_HighPriorityPoll();
        }
    }
}
