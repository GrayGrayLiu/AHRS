#include "ICM42688_Service.h"

#include <stdint.h>
#include <stdio.h>

#include "ICM42688_API.h"
#include "TimeBase.h"
#include "main.h"
#include "scheduler.h"

#define ICM42688_RUNTIME_SAMPLE_PRINT_ENABLE 0

enum
{
    ICM42688_INIT_SERVICE_INTERVAL_MS = 100u,
    ICM42688_INIT_RETRY_INTERVAL_MS = 1000u,
    ICM42688_DEBUG_SERVICE_INTERVAL_MS = 1000u,
    ICM42688_PRINT_INTERVAL_MS = 10000u,
};

static uint8_t icm42688_bound = 0u;
static uint8_t icm42688_started = 0u;
static uint8_t icm42688_running = 0u;
static uint32_t icm42688_last_init_service_tick = 0u;
static uint32_t icm42688_next_init_tick = 0u;
static uint32_t icm42688_last_debug_service_tick = 0u;
static uint32_t icm42688_last_print_tick = 0u;
static ICM42688_Status icm42688_last_status = ICM42688_STATUS_OK;
static uint8_t icm42688_service_polling = 0u;

static uint8_t TickReached(const uint32_t now, const uint32_t target)
{
    return (uint8_t)((int32_t)(now - target) >= 0);
}

static void SetLed(const GPIO_PinState red,
                   const GPIO_PinState green,
                   const GPIO_PinState blue)
{
    HAL_GPIO_WritePin(RGB_R_GPIO_Port, RGB_R_Pin, red);
    HAL_GPIO_WritePin(RGB_G_GPIO_Port, RGB_G_Pin, green);
    HAL_GPIO_WritePin(RGB_B_GPIO_Port, RGB_B_Pin, blue);
}

static void ServiceInit(const uint32_t now)
{
    if (icm42688_started != 0u
        || !TickReached(now, icm42688_next_init_tick)) {
        return;
    }

    SetLed(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);

    if (icm42688_bound == 0u) {
        printf("[ICM42688P] bind...\r\n");
        icm42688_last_status = ICM42688_Bind(&hspi1,
                                              IMU_SPI_CS_GPIO_Port,
                                              IMU_SPI_CS_Pin);

        if (icm42688_last_status != ICM42688_STATUS_OK) {
            SetLed(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
            printf("[ICM42688P] bind failed status=%ld\r\n",
                   (long)icm42688_last_status);
            icm42688_next_init_tick = now + ICM42688_INIT_RETRY_INTERVAL_MS;
            return;
        }

        icm42688_bound = 1u;
        printf("[ICM42688P] bind ok\r\n");
    }

    printf("[ICM42688P] init...\r\n");
    icm42688_last_status = ICM42688_Init();

    if (icm42688_last_status != ICM42688_STATUS_OK) {
        SetLed(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
        printf("[ICM42688P] init failed status=%ld\r\n",
               (long)icm42688_last_status);
        icm42688_next_init_tick = now + ICM42688_INIT_RETRY_INTERVAL_MS;
        return;
    }

    icm42688_started = 1u;
    SetLed(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);
}

static void Update(void)
{
    if (icm42688_started == 0u) {
        return;
    }

    icm42688_last_status = ICM42688_Update();
    ICM42688_Sample sample = {0};
    const ICM42688_Status sample_status = ICM42688_GetLatest(&sample);
    const uint8_t configured = (uint8_t)(
        sample_status == ICM42688_STATUS_OK && sample.configured != 0u);

    if (configured == 0u) {
        icm42688_running = 0u;

        if (icm42688_last_status == ICM42688_STATUS_OK
            || icm42688_last_status == ICM42688_STATUS_NO_DATA) {
            SetLed(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);

        } else {
            SetLed(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
        }

        return;
    }

    if (icm42688_running == 0u) {
        icm42688_running = 1u;
        printf("[ICM42688P] init ok, interrupt FIFO update started\r\n");
    }

    if (icm42688_last_status == ICM42688_STATUS_OK
        || icm42688_last_status == ICM42688_STATUS_NO_DATA) {
        SetLed(GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET);

    } else {
        SetLed(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
    }
}

static void PrintLatest(void)
{
#if ICM42688_RUNTIME_SAMPLE_PRINT_ENABLE == 0
    return;
#endif

    if (icm42688_running == 0u) {
        return;
    }

    ICM42688_Sample sample = {0};
    const ICM42688_Status status = ICM42688_GetLatest(&sample);
    const uint32_t now = TimeBase_Millis();

    if (!TickReached(now, icm42688_last_print_tick + ICM42688_PRINT_INTERVAL_MS)) {
        return;
    }

    icm42688_last_print_tick = now;

    if (status != ICM42688_STATUS_OK || sample.data_valid == 0u) {
        printf("[ICM42688P] unavailable get=%ld st=%ld e=%lu\r\n",
               (long)status,
               (long)icm42688_last_status,
               (unsigned long)sample.error_counter);
        return;
    }

    const long accel_z_milli = (long)(sample.accel_m_s2[2] * 1000.0f);
    const long gyro_z_milli = (long)(sample.gyro_rad_s[2] * 1000.0f);
    const long delta_time_ms = (long)(sample.delta_time_s * 1000.0f);
    const long delta_angle_z_micro = (long)(sample.delta_angle_rad[2] * 1000000.0f);
    const long delta_velocity_z_micro = (long)(sample.delta_velocity_m_s[2] * 1000000.0f);

    printf("[ICM42688P] st=%ld src=%u cnt=%u n=%u hdr=0x%02X s=%lu e=%lu "
           "irq=%lu dt=%ld dthz=%ld dvz=%ld az=%ld gz=%ld\r\n",
           (long)icm42688_last_status,
           (unsigned int)sample.data_source,
           (unsigned int)sample.fifo_count_bytes,
           (unsigned int)sample.fifo_valid_packets,
           (unsigned int)sample.fifo_header,
           (unsigned long)sample.sample_counter,
           (unsigned long)sample.error_counter,
           (unsigned long)sample.interrupt_counter,
           delta_time_ms,
           delta_angle_z_micro,
           delta_velocity_z_micro,
           accel_z_milli,
           gyro_z_milli);
}

void ICM42688_ServiceNotifyDataReadyFromISR(const uint64_t timestamp_us)
{
    ICM42688_OnDataReadyInterrupt(timestamp_us);
    Scheduler_PostHighPriorityEventFromISR(SCHED_HP_EVENT_IMU_DRDY);
}

void ICM42688_ServiceRun(void)
{
    if (icm42688_service_polling != 0u) {
        return;
    }

    icm42688_service_polling = 1u;
    const uint32_t now = TimeBase_Millis();

    // Bare-metal scheduler adapter only: binding, startup indication and the
    // periodic call live here; the hardware lifecycle stays in RunImpl().
    if (icm42688_started == 0u) {
        if (TickReached(now,
                        icm42688_last_init_service_tick
                        + ICM42688_INIT_SERVICE_INTERVAL_MS)) {
            icm42688_last_init_service_tick = now;
            ServiceInit(now);
        }

        icm42688_service_polling = 0u;
        return;
    }

    Update();

    if (TickReached(now,
                    icm42688_last_debug_service_tick
                    + ICM42688_DEBUG_SERVICE_INTERVAL_MS)) {
        icm42688_last_debug_service_tick = now;
        PrintLatest();
    }

    icm42688_service_polling = 0u;
}
