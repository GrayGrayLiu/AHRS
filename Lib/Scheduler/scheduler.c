//
// Created by Gray on 2026/1/7.
//

#include "scheduler.h"
#include <stdio.h>

#include "ICM42688_API.h"
#include "main.h"

enum
{
    ICM42688P_INIT_RETRY_INTERVAL_MS = 1000u,
};

static uint8_t icm42688p_bound = 0u;
static uint8_t icm42688p_initialized = 0u;
static uint32_t icm42688p_next_init_tick = 0u;
static ICM42688_Status icm42688p_last_status = ICM42688_STATUS_OK;

static uint8_t ICM42688P_TickReached(const uint32_t now, const uint32_t target)
{
    return (uint8_t)((int32_t)(now - target) >= 0);
}

static void ICM42688P_SetLed(const GPIO_PinState red,
                             const GPIO_PinState green,
                             const GPIO_PinState blue)
{
    HAL_GPIO_WritePin(RGB_R_GPIO_Port, RGB_R_Pin, red);
    HAL_GPIO_WritePin(RGB_G_GPIO_Port, RGB_G_Pin, green);
    HAL_GPIO_WritePin(RGB_B_GPIO_Port, RGB_B_Pin, blue);
}

static void ICM42688P_ServiceInit(void)
{
    const uint32_t now = HAL_GetTick();

    if (icm42688p_initialized != 0u
        || !ICM42688P_TickReached(now, icm42688p_next_init_tick)) {
        return;
    }

    ICM42688P_SetLed(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);

    if (icm42688p_bound == 0u) {
        printf("[ICM42688P] bind...\r\n");
        icm42688p_last_status = ICM42688_Bind(&hspi1,
                                               IMU_SPI_CS_GPIO_Port,
                                               IMU_SPI_CS_Pin);

        if (icm42688p_last_status != ICM42688_STATUS_OK) {
            ICM42688P_SetLed(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
            printf("[ICM42688P] bind failed status=%ld\r\n",
                   (long)icm42688p_last_status);
            icm42688p_next_init_tick = now + ICM42688P_INIT_RETRY_INTERVAL_MS;
            return;
        }

        icm42688p_bound = 1u;
        printf("[ICM42688P] bind ok\r\n");
    }

    printf("[ICM42688P] init...\r\n");
    icm42688p_last_status = ICM42688_Init();

    if (icm42688p_last_status != ICM42688_STATUS_OK) {
        ICM42688P_SetLed(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
        printf("[ICM42688P] init failed status=%ld\r\n",
               (long)icm42688p_last_status);
        icm42688p_next_init_tick = now + ICM42688P_INIT_RETRY_INTERVAL_MS;
        return;
    }

    icm42688p_initialized = 1u;
    ICM42688P_SetLed(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);
    printf("[ICM42688P] init ok, polling update started\r\n");
}

static void ICM42688P_UpdateTask(void)
{
    if (icm42688p_initialized == 0u) {
        return;
    }

    icm42688p_last_status = ICM42688_Update();

    if (icm42688p_last_status == ICM42688_STATUS_OK) {
        ICM42688P_SetLed(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);

    } else {
        ICM42688P_SetLed(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
    }
}

static void ICM42688P_PrintLatest(void)
{
    if (icm42688p_initialized == 0u) {
        return;
    }

    ICM42688_Sample sample = {0};
    const ICM42688_Status status = ICM42688_GetLatest(&sample);

    if (status != ICM42688_STATUS_OK || sample.data_valid == 0u) {
        printf("[ICM42688P] sample unavailable status=%ld update_status=%ld errors=%lu\r\n",
               (long)status,
               (long)icm42688p_last_status,
               (unsigned long)sample.error_counter);
        return;
    }

    const long accel_x_milli = (long)(sample.accel_m_s2[0] * 1000.0f);
    const long accel_y_milli = (long)(sample.accel_m_s2[1] * 1000.0f);
    const long accel_z_milli = (long)(sample.accel_m_s2[2] * 1000.0f);
    const long gyro_x_milli = (long)(sample.gyro_rad_s[0] * 1000.0f);
    const long gyro_y_milli = (long)(sample.gyro_rad_s[1] * 1000.0f);
    const long gyro_z_milli = (long)(sample.gyro_rad_s[2] * 1000.0f);
    const long temperature_centi = (long)(sample.temperature_deg_c * 100.0f);

    printf("[ICM42688P] raw acc=(%d,%d,%d) gyro=(%d,%d,%d) "
           "acc_m_s2_x1000=(%ld,%ld,%ld) gyro_rad_s_x1000=(%ld,%ld,%ld) "
           "temp_c_x100=%ld samples=%lu errors=%lu\r\n",
           (int)sample.accel_raw[0],
           (int)sample.accel_raw[1],
           (int)sample.accel_raw[2],
           (int)sample.gyro_raw[0],
           (int)sample.gyro_raw[1],
           (int)sample.gyro_raw[2],
           accel_x_milli,
           accel_y_milli,
           accel_z_milli,
           gyro_x_milli,
           gyro_y_milli,
           gyro_z_milli,
           temperature_centi,
           (unsigned long)sample.sample_counter,
           (unsigned long)sample.error_counter);
}
static void Loop_1000Hz(void) //1ms执行一次
{

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
    ICM42688P_UpdateTask();
}

static void Loop_50Hz(void) //20ms执行一次
{
}

static void Loop_20Hz(void) //50ms执行一次
{
}

static void Loop_10Hz(void) //100ms执行一次
{
    ICM42688P_ServiceInit();
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
    ICM42688P_PrintLatest();
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

void Scheduler_Setup(void)
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
}

//这个函数放到main函数的while(1)中，不停判断是否有线程应该执行
void Scheduler_Run(void)
{
    uint8_t index = 0;
    //循环判断所有线程，是否应该执行

    for (index = 0; index < TASK_NUM; index++) {
        //获取系统当前时间，单位MS
        uint32_t tnow = HAL_GetTick();
        //进行判断，如果当前时间减去上一次执行的时间，大于等于该线程的执行周期，则执行线程
        if (tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks) {

            //更新线程的执行时间，用于下一次判断
            sched_tasks[index].last_run = tnow;
            //执行线程函数，使用的是函数指针
            sched_tasks[index].task_func();
        }
    }
}
