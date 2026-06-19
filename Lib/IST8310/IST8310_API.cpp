/******************************************************************************
 * @file    IST8310_API.cpp
 * @brief   C API wrapper for the IST8310 C++ driver
 *
 * @details
 * Contains only handle management, pointer validation and calls into the
 * IST8310 C++ driver.
 *
 * @author  Gray
 * @email   grayme12345@gmail.com
 * @date    2026/6/4
 *
 * @copyright
 * Copyright (c) 2026 Gray
 *
 * This software is provided "as is" without any warranties.
 *
 ******************************************************************************/

#include "IST8310_API.h"
#include "IST8310.hpp"

#include <new>

struct IST8310_HandleImpl
{
    IST8310_HandleImpl(I2C_HandleTypeDef* hi2c,
                       const uint8_t address_7bit,
                       GPIO_TypeDef* reset_port,
                       const uint16_t reset_pin)
        : driver(hi2c, address_7bit, reset_port, reset_pin)
    {
    }

    IST8310 driver;
};

namespace
{
    // The C API uses one statically allocated handle and never allocates from
    // the heap. This matches the current single-IST8310 hardware. A fixed-size
    // object pool can replace this storage if multiple devices are needed.
    alignas(IST8310_HandleImpl) uint8_t handle_storage[sizeof(IST8310_HandleImpl)]{};
    IST8310_Handle active_handle = nullptr;

    bool IsActiveHandle(const IST8310_Handle handle)
    {
        return handle != nullptr && handle == active_handle;
    }

    IST8310_Status ToApiStatus(const IST8310::Status status)
    {
        return static_cast<IST8310_Status>(static_cast<int32_t>(status));
    }

    static_assert(static_cast<int32_t>(IST8310::Status::Ok) == IST8310_STATUS_OK);
    static_assert(static_cast<int32_t>(IST8310::Status::InvalidArgument) == IST8310_STATUS_INVALID_ARGUMENT);
    static_assert(static_cast<int32_t>(IST8310::Status::I2cError) == IST8310_STATUS_I2C_ERROR);
    static_assert(static_cast<int32_t>(IST8310::Status::WrongDeviceId) == IST8310_STATUS_WRONG_DEVICE_ID);
    static_assert(static_cast<int32_t>(IST8310::Status::ResetTimeout) == IST8310_STATUS_RESET_TIMEOUT);
    static_assert(static_cast<int32_t>(IST8310::Status::DataNotReady) == IST8310_STATUS_DATA_NOT_READY);
    static_assert(static_cast<int32_t>(IST8310::Status::DataOverrun) == IST8310_STATUS_DATA_OVERRUN);
    static_assert(static_cast<int32_t>(IST8310::Status::ConfigMismatch) == IST8310_STATUS_CONFIG_MISMATCH);
}

IST8310_Handle IST8310_Create(I2C_HandleTypeDef* hi2c,
                              const uint8_t address_7bit,
                              GPIO_TypeDef* reset_port,
                              const uint16_t reset_pin)
{
    if (hi2c == nullptr
        || address_7bit < IST8310_Regs::I2C_ADDRESS_MIN_7BIT
        || address_7bit > IST8310_Regs::I2C_ADDRESS_MAX_7BIT
        || (reset_port == nullptr && reset_pin != 0u)
        || (reset_port != nullptr && reset_pin == 0u))
    {
        return nullptr;
    }

    if (active_handle != nullptr)
    {
        return nullptr;
    }

    active_handle = ::new (static_cast<void*>(handle_storage))
        IST8310_HandleImpl(hi2c, address_7bit, reset_port, reset_pin);

    return active_handle;
}

void IST8310_Destroy(const IST8310_Handle handle)
{
    if (!IsActiveHandle(handle))
    {
        return;
    }

    handle->~IST8310_HandleImpl();
    active_handle = nullptr;
}

IST8310_Status IST8310_Init(const IST8310_Handle handle)
{
    if (!IsActiveHandle(handle))
    {
        return IST8310_STATUS_INVALID_ARGUMENT;
    }

    return ToApiStatus(handle->driver.Init());
}

IST8310_Status IST8310_Probe(const IST8310_Handle handle)
{
    if (!IsActiveHandle(handle))
    {
        return IST8310_STATUS_INVALID_ARGUMENT;
    }

    return ToApiStatus(handle->driver.Probe());
}

IST8310_Status IST8310_Reset(const IST8310_Handle handle)
{
    if (!IsActiveHandle(handle))
    {
        return IST8310_STATUS_INVALID_ARGUMENT;
    }

    return ToApiStatus(handle->driver.Reset());
}

IST8310_Status IST8310_RegisterRead(const IST8310_Handle handle,
                                    const uint8_t reg,
                                    uint8_t* value)
{
    if (!IsActiveHandle(handle) || value == nullptr)
    {
        return IST8310_STATUS_INVALID_ARGUMENT;
    }

    return ToApiStatus(handle->driver.RegisterRead(static_cast<IST8310_Regs::Register>(reg), *value));
}

IST8310_Status IST8310_RegisterWrite(const IST8310_Handle handle,
                                     const uint8_t reg,
                                     const uint8_t value)
{
    if (!IsActiveHandle(handle))
    {
        return IST8310_STATUS_INVALID_ARGUMENT;
    }

    return ToApiStatus(handle->driver.RegisterWrite(static_cast<IST8310_Regs::Register>(reg), value));
}

IST8310_Status IST8310_ReadBuffer(const IST8310_Handle handle,
                                  const uint8_t start_reg,
                                  uint8_t* buffer,
                                  const uint16_t length)
{
    if (!IsActiveHandle(handle) || buffer == nullptr || length == 0u)
    {
        return IST8310_STATUS_INVALID_ARGUMENT;
    }

    return ToApiStatus(handle->driver.ReadBuffer(static_cast<IST8310_Regs::Register>(start_reg),
                                                 buffer,
                                                 length));
}

IST8310_Status IST8310_ReadRawMag(const IST8310_Handle handle, IST8310_RawMagData* data)
{
    if (!IsActiveHandle(handle) || data == nullptr)
    {
        return IST8310_STATUS_INVALID_ARGUMENT;
    }

    IST8310::RawMagData raw_data{};
    const IST8310::Status status = handle->driver.ReadRawMag(raw_data);

    if (status != IST8310::Status::Ok)
    {
        return ToApiStatus(status);
    }

    data->x = raw_data.x;
    data->y = raw_data.y;
    data->z = raw_data.z;

    return IST8310_STATUS_OK;
}

IST8310_Status IST8310_ReadMag_uT(const IST8310_Handle handle, IST8310_MagData_uT* data)
{
    if (!IsActiveHandle(handle) || data == nullptr)
    {
        return IST8310_STATUS_INVALID_ARGUMENT;
    }

    IST8310::MagData_uT mag_data{};
    const IST8310::Status status = handle->driver.ReadMag_uT(mag_data);

    if (status != IST8310::Status::Ok)
    {
        return ToApiStatus(status);
    }

    data->x = mag_data.x;
    data->y = mag_data.y;
    data->z = mag_data.z;

    return IST8310_STATUS_OK;
}
