#include "ICM42688_API.h"

#include <new>

#include "ICM42688P.hpp"

namespace
{
alignas(ICM42688P) unsigned char g_icm_storage[sizeof(ICM42688P)]{};
ICM42688P *g_icm = nullptr;
bool g_bound = false;
bool g_initialized = false;

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
    g_initialized = false;
    return ICM42688_STATUS_OK;
}

extern "C" ICM42688_Status ICM42688_Init(void)
{
    if (!IsBound()) {
        return ICM42688_STATUS_INVALID_ARGUMENT;
    }

    const ICM42688P::Status status = g_icm->Init();
    g_initialized = status == ICM42688P::Status::Ok;
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

    g_initialized = false;
    return ToApiStatus(g_icm->Reset());
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
    if (!IsBound() || !g_initialized || data == nullptr) {
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
    if (!IsBound() || !g_initialized || data == nullptr) {
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
    if (!IsBound() || !g_initialized) {
        return ICM42688_STATUS_INVALID_ARGUMENT;
    }

    return ToApiStatus(g_icm->Update());
}

extern "C" ICM42688_Status ICM42688_GetLatest(ICM42688_Sample *sample)
{
    if (!IsBound() || !g_initialized || sample == nullptr) {
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
    }

    sample->temp_raw = latest.temp_raw;
    sample->temperature_deg_c = latest.temperature_deg_c;
    sample->sample_counter = latest.sample_counter;
    sample->error_counter = latest.error_counter;
    sample->configured = latest.configured ? 1u : 0u;
    sample->data_valid = latest.data_valid ? 1u : 0u;
    return ICM42688_STATUS_OK;
}

extern "C" ICM42688_Status ICM42688_Read(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
    if (!IsBound() || !g_initialized || accel == nullptr || gyro == nullptr || temp == nullptr) {
        return ICM42688_STATUS_INVALID_ARGUMENT;
    }

    return ICM42688_STATUS_UNSUPPORTED;
}
