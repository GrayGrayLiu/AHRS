//
// Created by Gray on 2026/1/7.
//

#include "scheduler.h"
#include <stdio.h>

#include "ICM42688_API.h"
#include "main.h"

enum
{
    ICM42688P_TEST_STAGE_BIND = 0u,
    ICM42688P_TEST_STAGE_LOW_LEVEL_WHO_AM_I = 1u,
    ICM42688P_TEST_STAGE_INIT = 2u,
    ICM42688P_TEST_STAGE_CONFIG_READBACK = 3u,
    ICM42688P_TEST_STAGE_RAW_READ = 4u,
};

enum
{
    ICM42688P_TEST_WHO_AM_I_REGISTER = 0x75u,
    ICM42688P_TEST_WHO_AM_I_EXPECTED = 0x47u,
    ICM42688P_TEST_PWR_MGMT0_REGISTER = 0x4Eu,
    ICM42688P_TEST_GYRO_CONFIG0_REGISTER = 0x4Fu,
    ICM42688P_TEST_ACCEL_CONFIG0_REGISTER = 0x50u,
    ICM42688P_TEST_PWR_MGMT0_EXPECTED = 0x0Fu,
    ICM42688P_TEST_GYRO_CONFIG0_EXPECTED = 0x06u,
    ICM42688P_TEST_ACCEL_CONFIG0_EXPECTED = 0x06u,
    ICM42688P_TEST_RETRY_INTERVAL_MS = 1000u,
    ICM42688P_TEST_RAW_READ_INTERVAL_MS = 500u,
    ICM42688P_TEST_RAW_PRINT_INTERVAL_MS = 1000u,
};

volatile uint32_t icm42688p_test_tick = 0u;
volatile uint32_t icm42688p_test_counter = 0u;
volatile uint32_t icm42688p_test_stage = ICM42688P_TEST_STAGE_BIND;
volatile uint32_t icm42688p_test_last_status = ICM42688_STATUS_OK;
volatile uint32_t icm42688p_test_error_count = 0u;
volatile uint32_t icm42688p_test_probe_ok_count = 0u;
volatile uint32_t icm42688p_test_raw_ok_count = 0u;
volatile uint8_t icm42688p_test_whoami = 0u;
volatile uint8_t icm42688p_test_pwr_mgmt0 = 0u;
volatile uint8_t icm42688p_test_gyro_config0 = 0u;
volatile uint8_t icm42688p_test_accel_config0 = 0u;
volatile int16_t icm42688p_test_accel_raw_x = 0;
volatile int16_t icm42688p_test_accel_raw_y = 0;
volatile int16_t icm42688p_test_accel_raw_z = 0;
volatile int16_t icm42688p_test_gyro_raw_x = 0;
volatile int16_t icm42688p_test_gyro_raw_y = 0;
volatile int16_t icm42688p_test_gyro_raw_z = 0;

static uint8_t icm42688p_test_bound = 0u;
static uint32_t icm42688p_test_next_action_tick = 0u;
static uint32_t icm42688p_test_last_raw_tick = 0u;
static uint32_t icm42688p_test_last_print_tick = 0u;
static uint8_t icm42688p_test_spi_config_printed = 0u;

static uint8_t ICM42688P_Test_TickReached(const uint32_t now, const uint32_t target)
{
    return (uint8_t)((int32_t)(now - target) >= 0);
}

static void ICM42688P_Test_SetLed(const GPIO_PinState red,
                                  const GPIO_PinState green,
                                  const GPIO_PinState blue)
{
    HAL_GPIO_WritePin(RGB_R_GPIO_Port, RGB_R_Pin, red);
    HAL_GPIO_WritePin(RGB_G_GPIO_Port, RGB_G_Pin, green);
    HAL_GPIO_WritePin(RGB_B_GPIO_Port, RGB_B_Pin, blue);
}

static void ICM42688P_Test_SetLedIdle(void)
{
    ICM42688P_Test_SetLed(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);
}

static void ICM42688P_Test_SetLedOk(void)
{
    ICM42688P_Test_SetLed(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);
}

static void ICM42688P_Test_SetLedError(void)
{
    ICM42688P_Test_SetLed(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
}

static void ICM42688P_Test_PrintSpiConfigOnce(void)
{
    if (icm42688p_test_spi_config_printed != 0u) {
        return;
    }

    icm42688p_test_spi_config_printed = 1u;
    printf("[SPI1] Mode=%lu Direction=%lu DataSize=%lu CLKPolarity=%lu CLKPhase=%lu NSS=%lu Prescaler=%lu FirstBit=%lu\r\n",
           (unsigned long)hspi1.Init.Mode,
           (unsigned long)hspi1.Init.Direction,
           (unsigned long)hspi1.Init.DataSize,
           (unsigned long)hspi1.Init.CLKPolarity,
           (unsigned long)hspi1.Init.CLKPhase,
           (unsigned long)hspi1.Init.NSS,
           (unsigned long)hspi1.Init.BaudRatePrescaler,
           (unsigned long)hspi1.Init.FirstBit);
}

static void ICM42688P_Test_RecordError(const uint32_t now, const ICM42688_Status status)
{
    icm42688p_test_last_status = (uint32_t)(int32_t)status;
    ++icm42688p_test_error_count;
    icm42688p_test_next_action_tick = now + ICM42688P_TEST_RETRY_INTERVAL_MS;
    ICM42688P_Test_SetLedError();
    printf("[ICM42688P] error stage=%lu status=%ld\r\n",
           (unsigned long)icm42688p_test_stage,
           (long)status);
}

static ICM42688_Status ICM42688P_Test_ReadConfigRegisters(void)
{
    uint8_t pwr_mgmt0 = 0u;
    uint8_t gyro_config0 = 0u;
    uint8_t accel_config0 = 0u;

    ICM42688_Status status = ICM42688_RegisterRead(ICM42688P_TEST_PWR_MGMT0_REGISTER,
                                                    &pwr_mgmt0);

    if (status != ICM42688_STATUS_OK) {
        return status;
    }

    status = ICM42688_RegisterRead(ICM42688P_TEST_GYRO_CONFIG0_REGISTER, &gyro_config0);

    if (status != ICM42688_STATUS_OK) {
        return status;
    }

    status = ICM42688_RegisterRead(ICM42688P_TEST_ACCEL_CONFIG0_REGISTER, &accel_config0);

    if (status != ICM42688_STATUS_OK) {
        return status;
    }

    icm42688p_test_pwr_mgmt0 = pwr_mgmt0;
    icm42688p_test_gyro_config0 = gyro_config0;
    icm42688p_test_accel_config0 = accel_config0;
    return ICM42688_STATUS_OK;
}

static void ICM42688P_TestTask(void)
{
    const uint32_t now = HAL_GetTick();
    ICM42688_Status status = ICM42688_STATUS_OK;

    icm42688p_test_tick = now;
    ++icm42688p_test_counter;

    if (!ICM42688P_Test_TickReached(now, icm42688p_test_next_action_tick)) {
        return;
    }

    switch (icm42688p_test_stage) {
    case ICM42688P_TEST_STAGE_BIND:
        ICM42688P_Test_SetLedIdle();

        if (icm42688p_test_bound == 0u) {
            printf("[ICM42688P] stage=0 bind...\r\n");
            status = ICM42688_Bind(&hspi1, IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin);

            if (status != ICM42688_STATUS_OK) {
                ICM42688P_Test_RecordError(now, status);
                return;
            }

            icm42688p_test_bound = 1u;
        }

        icm42688p_test_last_status = ICM42688_STATUS_OK;
        printf("[ICM42688P] bind ok\r\n");
        icm42688p_test_stage = ICM42688P_TEST_STAGE_LOW_LEVEL_WHO_AM_I;
        break;

    case ICM42688P_TEST_STAGE_LOW_LEVEL_WHO_AM_I:
    {
        uint8_t who_am_i = 0u;

        ICM42688P_Test_PrintSpiConfigOnce();
        status = ICM42688_RegisterRead(ICM42688P_TEST_WHO_AM_I_REGISTER, &who_am_i);
        icm42688p_test_whoami = who_am_i;
        printf("[ICM42688P] low-level WHO_AM_I read status=%ld value=0x%02X\r\n",
               (long)status,
               (unsigned int)who_am_i);

        if (status != ICM42688_STATUS_OK) {
            ICM42688P_Test_RecordError(now, status);
            return;
        }

        if (who_am_i != ICM42688P_TEST_WHO_AM_I_EXPECTED) {
            ICM42688P_Test_RecordError(now, ICM42688_STATUS_WRONG_DEVICE_ID);
            return;
        }

        icm42688p_test_last_status = ICM42688_STATUS_OK;
        ++icm42688p_test_probe_ok_count;
        printf("[ICM42688P] low-level WHO_AM_I ok\r\n");
        icm42688p_test_stage = ICM42688P_TEST_STAGE_INIT;
        break;
    }

    case ICM42688P_TEST_STAGE_INIT:
        printf("[ICM42688P] stage=2 init...\r\n");
        status = ICM42688_Init();

        if (status != ICM42688_STATUS_OK) {
            printf("[ICM42688P] init failed status=%ld whoami=0x%02X\r\n",
                   (long)status,
                   (unsigned int)icm42688p_test_whoami);
            ICM42688P_Test_RecordError(now, status);
            icm42688p_test_stage = ICM42688P_TEST_STAGE_LOW_LEVEL_WHO_AM_I;
            return;
        }

        icm42688p_test_last_status = ICM42688_STATUS_OK;
        printf("[ICM42688P] init ok\r\n");
        icm42688p_test_stage = ICM42688P_TEST_STAGE_CONFIG_READBACK;
        break;

    case ICM42688P_TEST_STAGE_CONFIG_READBACK:
        status = ICM42688P_Test_ReadConfigRegisters();

        if (status != ICM42688_STATUS_OK) {
            ICM42688P_Test_RecordError(now, status);
            icm42688p_test_stage = ICM42688P_TEST_STAGE_LOW_LEVEL_WHO_AM_I;
            return;
        }

        printf("[ICM42688P] config PWR_MGMT0=0x%02X GYRO_CONFIG0=0x%02X ACCEL_CONFIG0=0x%02X\r\n",
               (unsigned int)icm42688p_test_pwr_mgmt0,
               (unsigned int)icm42688p_test_gyro_config0,
               (unsigned int)icm42688p_test_accel_config0);

        if (icm42688p_test_pwr_mgmt0 != ICM42688P_TEST_PWR_MGMT0_EXPECTED
            || icm42688p_test_gyro_config0 != ICM42688P_TEST_GYRO_CONFIG0_EXPECTED
            || icm42688p_test_accel_config0 != ICM42688P_TEST_ACCEL_CONFIG0_EXPECTED) {
            ICM42688P_Test_RecordError(now, ICM42688_STATUS_CONFIG_MISMATCH);
            icm42688p_test_stage = ICM42688P_TEST_STAGE_LOW_LEVEL_WHO_AM_I;
            return;
        }

        icm42688p_test_last_status = ICM42688_STATUS_OK;
        icm42688p_test_stage = ICM42688P_TEST_STAGE_RAW_READ;
        icm42688p_test_last_raw_tick = now;
        icm42688p_test_last_print_tick = now;
        ICM42688P_Test_SetLedOk();
        printf("[ICM42688P] communication ok, entering raw read stage\r\n");
        break;

    case ICM42688P_TEST_STAGE_RAW_READ:
    {
        ICM42688_RawVector accel = {0};
        ICM42688_RawVector gyro = {0};

        if ((now - icm42688p_test_last_raw_tick) < ICM42688P_TEST_RAW_READ_INTERVAL_MS) {
            return;
        }

        icm42688p_test_last_raw_tick = now;
        status = ICM42688_ReadRawAccel(&accel);

        if (status != ICM42688_STATUS_OK) {
            ICM42688P_Test_RecordError(now, status);
            return;
        }

        status = ICM42688_ReadRawGyro(&gyro);

        if (status != ICM42688_STATUS_OK) {
            ICM42688P_Test_RecordError(now, status);
            return;
        }

        icm42688p_test_accel_raw_x = accel.x;
        icm42688p_test_accel_raw_y = accel.y;
        icm42688p_test_accel_raw_z = accel.z;
        icm42688p_test_gyro_raw_x = gyro.x;
        icm42688p_test_gyro_raw_y = gyro.y;
        icm42688p_test_gyro_raw_z = gyro.z;
        icm42688p_test_last_status = ICM42688_STATUS_OK;
        ++icm42688p_test_raw_ok_count;
        ICM42688P_Test_SetLedOk();

        if ((now - icm42688p_test_last_print_tick) >= ICM42688P_TEST_RAW_PRINT_INTERVAL_MS) {
            icm42688p_test_last_print_tick = now;
            printf("[ICM42688P] raw acc=(%d,%d,%d), gyro=(%d,%d,%d)\r\n",
                   (int)accel.x,
                   (int)accel.y,
                   (int)accel.z,
                   (int)gyro.x,
                   (int)gyro.y,
                   (int)gyro.z);

            if (accel.x == INT16_MIN && accel.y == INT16_MIN && accel.z == INT16_MIN
                && gyro.x == INT16_MIN && gyro.y == INT16_MIN && gyro.z == INT16_MIN) {
                status = ICM42688P_Test_ReadConfigRegisters();

                if (status == ICM42688_STATUS_OK) {
                    printf("[ICM42688P] raw still invalid, PWR_MGMT0=0x%02X GYRO_CONFIG0=0x%02X ACCEL_CONFIG0=0x%02X\r\n",
                           (unsigned int)icm42688p_test_pwr_mgmt0,
                           (unsigned int)icm42688p_test_gyro_config0,
                           (unsigned int)icm42688p_test_accel_config0);

                } else {
                    ICM42688P_Test_RecordError(now, status);
                }
            }
        }
        break;
    }

    default:
        icm42688p_test_stage = ICM42688P_TEST_STAGE_BIND;
        ICM42688P_Test_RecordError(now, ICM42688_STATUS_INVALID_ARGUMENT);
        return;
    }

    icm42688p_test_next_action_tick = now;
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
}

static void Loop_50Hz(void) //20ms执行一次
{
}

static void Loop_20Hz(void) //50ms执行一次
{
}

static void Loop_10Hz(void) //100ms执行一次
{
    ICM42688P_TestTask();
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
