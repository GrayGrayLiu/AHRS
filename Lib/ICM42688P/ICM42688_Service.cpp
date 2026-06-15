#include "ICM42688_Service.hpp"

#include <new>
#include <stdint.h>
#include <stdio.h>

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

namespace
{
alignas(ICM42688P) uint8_t icm42688_storage[sizeof(ICM42688P)]{};
ICM42688P *icm42688 = nullptr;
bool icm42688_bound = false;
bool icm42688_started = false;
static uint8_t icm42688_running = 0u;
static uint32_t icm42688_last_init_service_tick = 0u;
static uint32_t icm42688_next_init_tick = 0u;
static uint32_t icm42688_last_debug_service_tick = 0u;
static uint32_t icm42688_last_print_tick = 0u;
ICM42688P::Status icm42688_last_status = ICM42688P::Status::Ok;
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
    if (icm42688_started
        || !TickReached(now, icm42688_next_init_tick)) {
        return;
    }

    SetLed(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);

    if (!icm42688_bound) {
        printf("[ICM42688P] bind...\r\n");
        icm42688 = ::new (static_cast<void *>(icm42688_storage))
            ICM42688P(&hspi1, IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin);
        icm42688_bound = true;
        printf("[ICM42688P] bind ok\r\n");
    }

    printf("[ICM42688P] init...\r\n");
    icm42688_last_status = icm42688->Init();

    if (icm42688_last_status != ICM42688P::Status::Ok) {
        SetLed(GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET);
        printf("[ICM42688P] init failed status=%ld\r\n",
               static_cast<long>(icm42688_last_status));
        icm42688_next_init_tick = now + ICM42688_INIT_RETRY_INTERVAL_MS;
        return;
    }

    icm42688_started = true;
    SetLed(GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET);
}

static void Update(void)
{
    if (!icm42688_started || icm42688 == nullptr) {
        return;
    }

    icm42688_last_status = icm42688->Update();
    ICM42688P::Sample sample{};
    const ICM42688P::Status sample_status = icm42688->GetLatest(sample);
    const uint8_t configured = (uint8_t)(
        sample_status == ICM42688P::Status::Ok && sample.configured);

    if (configured == 0u) {
        icm42688_running = 0u;

        if (icm42688_last_status == ICM42688P::Status::Ok
            || icm42688_last_status == ICM42688P::Status::NoData) {
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

    if (icm42688_last_status == ICM42688P::Status::Ok
        || icm42688_last_status == ICM42688P::Status::NoData) {
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
           static_cast<long>(icm42688_last_status),
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

} // namespace

void icm42688_service::NotifyDataReadyFromISR(const uint64_t timestamp_us)
{
    if (icm42688 != nullptr) {
        icm42688->DataReady(timestamp_us);
    }

    Scheduler_PostHighPriorityEventFromISR(SCHED_HP_EVENT_IMU_DRDY);
}

void icm42688_service::Run()
{
    if (icm42688_service_polling != 0u) {
        return;
    }

    icm42688_service_polling = 1u;
    const uint32_t now = TimeBase_Millis();

    // Bare-metal scheduler adapter only: binding, startup indication and the
    // periodic call live here; the hardware lifecycle stays in RunImpl().
    if (!icm42688_started) {
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

ICM42688P::Status icm42688_service::GetLatest(ICM42688P::Sample *sample)
{
    if (!icm42688_bound || !icm42688_started || icm42688 == nullptr || sample == nullptr) {
        return ICM42688P::Status::InvalidArgument;
    }

    return icm42688->GetLatest(*sample);
}

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
