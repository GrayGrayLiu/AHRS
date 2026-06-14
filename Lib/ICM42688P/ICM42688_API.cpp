#include "ICM42688_API.h"

#include <new>

#include "ICM42688P.hpp"

namespace
{
alignas(ICM42688P) unsigned char g_icm_storage[sizeof(ICM42688P)]{};
ICM42688P *g_icm = nullptr;
bool g_bound = false;
bool g_started = false;

bool IsBound()
{
    return g_bound && g_icm != nullptr;
}

ICM42688_Status ToApiStatus(const ICM42688P::Status status)
{
    return static_cast<ICM42688_Status>(static_cast<int32_t>(status));
}

static_assert(static_cast<int32_t>(ICM42688P::Status::Ok) == ICM42688_STATUS_OK);
static_assert(static_cast<int32_t>(ICM42688P::Status::InvalidArgument) == ICM42688_STATUS_INVALID_ARGUMENT);
static_assert(static_cast<int32_t>(ICM42688P::Status::SpiError) == ICM42688_STATUS_SPI_ERROR);
static_assert(static_cast<int32_t>(ICM42688P::Status::WrongDeviceId) == ICM42688_STATUS_WRONG_DEVICE_ID);
static_assert(static_cast<int32_t>(ICM42688P::Status::ResetTimeout) == ICM42688_STATUS_RESET_TIMEOUT);
static_assert(static_cast<int32_t>(ICM42688P::Status::Unsupported) == ICM42688_STATUS_UNSUPPORTED);
static_assert(static_cast<int32_t>(ICM42688P::Status::ConfigMismatch) == ICM42688_STATUS_CONFIG_MISMATCH);
static_assert(static_cast<int32_t>(ICM42688P::Status::NoData) == ICM42688_STATUS_NO_DATA);
static_assert(static_cast<int32_t>(ICM42688P::Status::FifoOverflow) == ICM42688_STATUS_FIFO_OVERFLOW);
static_assert(static_cast<int32_t>(ICM42688P::Status::BadFifoPacket) == ICM42688_STATUS_BAD_FIFO_PACKET);
} // namespace

extern "C" ICM42688_Status ICM42688_Bind(SPI_HandleTypeDef *hspi,
                                          GPIO_TypeDef *cs_port,
                                          const uint16_t cs_pin)
{
    if (hspi == nullptr || cs_port == nullptr || cs_pin == 0u) {
        return ICM42688_STATUS_INVALID_ARGUMENT;
    }

    if (IsBound()) {
        return ICM42688_STATUS_INVALID_ARGUMENT;
    }

    g_icm = ::new (static_cast<void*>(g_icm_storage)) ICM42688P(hspi, cs_port, cs_pin);
    g_bound = true;
    g_started = false;
    return ICM42688_STATUS_OK;
}

extern "C" ICM42688_Status ICM42688_Init(void)
{
    if (!IsBound()) {
        return ICM42688_STATUS_INVALID_ARGUMENT;
    }

    const ICM42688P::Status status = g_icm->Init();
    g_started = status == ICM42688P::Status::Ok;
    return ToApiStatus(status);
}

extern "C" ICM42688_Status ICM42688_Probe(void)
{
    if (!IsBound()) {
        return ICM42688_STATUS_INVALID_ARGUMENT;
    }

    return ToApiStatus(g_icm->Probe());
}

extern "C" ICM42688_Status ICM42688_Reset(void)
{
    if (!IsBound()) {
        return ICM42688_STATUS_INVALID_ARGUMENT;
    }

    const ICM42688P::Status status = g_icm->Reset();
    g_started = status == ICM42688P::Status::Ok;
    return ToApiStatus(status);
}

extern "C" ICM42688_Status ICM42688_RegisterRead(const uint8_t reg, uint8_t *value)
{
    if (!IsBound() || value == nullptr) {
        return ICM42688_STATUS_INVALID_ARGUMENT;
    }

    return ToApiStatus(g_icm->RegisterRead(static_cast<ICM42688P_Regs::RegsAdd::BANK0>(reg), *value));
}

extern "C" ICM42688_Status ICM42688_RegisterWrite(const uint8_t reg, const uint8_t value)
{
    if (!IsBound()) {
        return ICM42688_STATUS_INVALID_ARGUMENT;
    }

    return ToApiStatus(g_icm->RegisterWrite(static_cast<ICM42688P_Regs::RegsAdd::BANK0>(reg), value));
}

extern "C" ICM42688_Status ICM42688_ReadBuffer(const uint8_t start_reg,
                                                uint8_t *buffer,
                                                const uint16_t length)
{
    if (!IsBound() || buffer == nullptr || length == 0u) {
        return ICM42688_STATUS_INVALID_ARGUMENT;
    }

    return ToApiStatus(g_icm->ReadBuffer(static_cast<ICM42688P_Regs::RegsAdd::BANK0>(start_reg),
                                         buffer,
                                         length));
}

extern "C" ICM42688_Status ICM42688_ReadRawAccel(ICM42688_RawVector *data)
{
    if (!IsBound() || !g_started || data == nullptr) {
        return ICM42688_STATUS_INVALID_ARGUMENT;
    }

    ICM42688P::RawVector raw{};
    const ICM42688P::Status status = g_icm->ReadRawAccel(raw);

    if (status != ICM42688P::Status::Ok) {
        return ToApiStatus(status);
    }

    data->x = raw.x;
    data->y = raw.y;
    data->z = raw.z;
    return ICM42688_STATUS_OK;
}

extern "C" ICM42688_Status ICM42688_ReadRawGyro(ICM42688_RawVector *data)
{
    if (!IsBound() || !g_started || data == nullptr) {
        return ICM42688_STATUS_INVALID_ARGUMENT;
    }

    ICM42688P::RawVector raw{};
    const ICM42688P::Status status = g_icm->ReadRawGyro(raw);

    if (status != ICM42688P::Status::Ok) {
        return ToApiStatus(status);
    }

    data->x = raw.x;
    data->y = raw.y;
    data->z = raw.z;
    return ICM42688_STATUS_OK;
}

extern "C" ICM42688_Status ICM42688_Update(void)
{
    if (!IsBound() || !g_started) {
        return ICM42688_STATUS_INVALID_ARGUMENT;
    }

    return ToApiStatus(g_icm->Update());
}

extern "C" void ICM42688_OnDataReadyInterrupt(const uint64_t timestamp_us)
{
    if (IsBound()) {
        g_icm->DataReady(timestamp_us);
    }
}

extern "C" ICM42688_Status ICM42688_GetLatest(ICM42688_Sample *sample)
{
    if (!IsBound() || !g_started || sample == nullptr) {
        return ICM42688_STATUS_INVALID_ARGUMENT;
    }

    ICM42688P::Sample latest{};
    const ICM42688P::Status status = g_icm->GetLatest(latest);

    if (status != ICM42688P::Status::Ok) {
        return ToApiStatus(status);
    }

    sample->timestamp_ms = latest.timestamp_ms;

    for (uint8_t axis = 0u; axis < 3u; ++axis) {
        sample->accel_raw[axis] = latest.accel_raw[axis];
        sample->gyro_raw[axis] = latest.gyro_raw[axis];
        sample->accel_m_s2[axis] = latest.accel_m_s2[axis];
        sample->gyro_rad_s[axis] = latest.gyro_rad_s[axis];
        sample->accel_raw20[axis] = latest.accel_raw20[axis];
        sample->gyro_raw20[axis] = latest.gyro_raw20[axis];
        sample->accel_effective[axis] = latest.accel_effective[axis];
        sample->gyro_effective[axis] = latest.gyro_effective[axis];
        sample->delta_angle_rad[axis] = latest.delta_angle_rad[axis];
        sample->delta_velocity_m_s[axis] = latest.delta_velocity_m_s[axis];
    }

    sample->temp_raw = latest.temp_raw;
    sample->temperature_deg_c = latest.temperature_deg_c;
    sample->sample_counter = latest.sample_counter;
    sample->error_counter = latest.error_counter;
    sample->configured = latest.configured ? 1u : 0u;
    sample->data_valid = latest.data_valid ? 1u : 0u;
    sample->fifo_count_bytes = latest.fifo_count_bytes;
    sample->fifo_valid_packets = latest.fifo_valid_packets;
    sample->fifo_timestamp = latest.fifo_timestamp;
    sample->fifo_header = latest.fifo_header;
    sample->data_source = latest.data_source;
    sample->delta_time_s = latest.delta_time_s;
    sample->delta_samples = latest.delta_samples;
    sample->interrupt_counter = latest.interrupt_counter;
    sample->last_interrupt_timestamp_ms = latest.last_interrupt_timestamp_ms;
    sample->timestamp_us = latest.timestamp_us;
    sample->last_interrupt_timestamp_us = latest.last_interrupt_timestamp_us;
    return ICM42688_STATUS_OK;
}

extern "C" ICM42688_Status ICM42688_GetDeltaLatest(ICM42688_DeltaSample *sample)
{
    if (!IsBound() || !g_started || sample == nullptr) {
        return ICM42688_STATUS_INVALID_ARGUMENT;
    }

    ICM42688P::Sample latest{};
    const ICM42688P::Status status = g_icm->GetLatest(latest);

    if (status != ICM42688P::Status::Ok) {
        return ToApiStatus(status);
    }

    if (!latest.configured || !latest.data_valid) {
        return ICM42688_STATUS_NO_DATA;
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
    return ICM42688_STATUS_OK;
}
