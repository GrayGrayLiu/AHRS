/******************************************************************************
 * @file    IST8310.cpp
 * @brief   Minimal blocking I2C driver for the IST8310 magnetometer
 *
 * @details
 * Implements register access, device probing, reset, configuration, single
 * measurement polling and magnetic field conversion.
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

#include "IST8310.hpp"

namespace
{
    template<typename Enum>
    constexpr uint8_t ToByte(const Enum value)
    {
        return static_cast<uint8_t>(value);
    }
}

IST8310::IST8310(I2C_HandleTypeDef* hi2c,
                 const uint8_t address_7bit,
                 GPIO_TypeDef* reset_port,
                 const uint16_t reset_pin)
    : hi2c_(hi2c),
      address_7bit_(address_7bit),
      reset_port_(reset_port),
      reset_pin_(reset_pin)
{
}

IST8310::Status IST8310::Init()
{
    if (hi2c_ == nullptr
        || address_7bit_ < IST8310_Regs::I2C_ADDRESS_MIN_7BIT
        || address_7bit_ > IST8310_Regs::I2C_ADDRESS_MAX_7BIT)
    {
        return Status::InvalidArgument;
    }

    Status status = ReleaseHardwareReset();

    if (status != Status::Ok)
    {
        return status;
    }

    status = Probe();

    if (status != Status::Ok)
    {
        return status;
    }

    status = Reset();

    if (status != Status::Ok)
    {
        return status;
    }

    status = Configure();

    if (status != Status::Ok)
    {
        return status;
    }

    return Probe();
}

IST8310::Status IST8310::Probe() const
{
    uint8_t device_id{};
    const Status status = RegisterRead(IST8310_Regs::Register::WAI, device_id);

    if (status != Status::Ok)
    {
        return status;
    }

    return device_id == IST8310_Regs::DEVICE_ID ? Status::Ok : Status::WrongDeviceId;
}

IST8310::Status IST8310::Reset() const
{
    Status status = RegisterWrite(IST8310_Regs::Register::CNTL2,
                                  ToByte(IST8310_Regs::CNTL2_BITS::SRST));

    if (status != Status::Ok)
    {
        return status;
    }

    const uint32_t reset_start = HAL_GetTick();
    HAL_Delay(IST8310_Regs::POR_RESET_WAIT_MS);

    while ((HAL_GetTick() - reset_start) <= IST8310_Regs::SOFT_RESET_TIMEOUT_MS)
    {
        uint8_t control_2{};
        status = RegisterRead(IST8310_Regs::Register::CNTL2, control_2);

        if (status != Status::Ok)
        {
            return status;
        }

        if ((control_2 & ToByte(IST8310_Regs::CNTL2_BITS::SRST)) == IST8310_Regs::BitNone)
        {
            return Probe();
        }

        HAL_Delay(IST8310_Regs::RESET_POLL_INTERVAL_MS);
    }

    return Status::ResetTimeout;
}

IST8310::Status IST8310::RegisterRead(const IST8310_Regs::Register reg, uint8_t& value) const
{
    return ReadBuffer(reg, &value, IST8310_Regs::REGISTER_VALUE_LENGTH);
}

IST8310::Status IST8310::RegisterWrite(const IST8310_Regs::Register reg, const uint8_t value) const
{
    if (hi2c_ == nullptr
        || address_7bit_ < IST8310_Regs::I2C_ADDRESS_MIN_7BIT
        || address_7bit_ > IST8310_Regs::I2C_ADDRESS_MAX_7BIT)
    {
        return Status::InvalidArgument;
    }

    uint8_t write_value = value;
    const HAL_StatusTypeDef result = HAL_I2C_Mem_Write(hi2c_,
                                                       HalDeviceAddress(),
                                                       static_cast<uint16_t>(reg),
                                                       I2C_MEMADD_SIZE_8BIT,
                                                       &write_value,
                                                       IST8310_Regs::REGISTER_VALUE_LENGTH,
                                                       IST8310_Regs::I2C_TRANSACTION_TIMEOUT_MS);

    return result == HAL_OK ? Status::Ok : Status::I2cError;
}

IST8310::Status IST8310::ReadBuffer(const IST8310_Regs::Register start_reg,
                                    uint8_t* buffer,
                                    const uint16_t length) const
{
    if (hi2c_ == nullptr
        || buffer == nullptr
        || length == 0u
        || address_7bit_ < IST8310_Regs::I2C_ADDRESS_MIN_7BIT
        || address_7bit_ > IST8310_Regs::I2C_ADDRESS_MAX_7BIT)
    {
        return Status::InvalidArgument;
    }

    const HAL_StatusTypeDef result = HAL_I2C_Mem_Read(hi2c_,
                                                      HalDeviceAddress(),
                                                      static_cast<uint16_t>(start_reg),
                                                      I2C_MEMADD_SIZE_8BIT,
                                                      buffer,
                                                      length,
                                                      IST8310_Regs::I2C_TRANSACTION_TIMEOUT_MS);

    return result == HAL_OK ? Status::Ok : Status::I2cError;
}

IST8310::Status IST8310::ReadRawMag(RawMagData& data)
{
    Status status = StartSingleMeasurement();

    if (status != Status::Ok)
    {
        return status;
    }

    status = WaitDataReady();

    if (status != Status::Ok)
    {
        return status;
    }

    uint8_t buffer[IST8310_Regs::DATA_BURST_LENGTH]{};
    status = ReadBuffer(IST8310_Regs::Register::STAT1, buffer, IST8310_Regs::DATA_BURST_LENGTH);

    if (status != Status::Ok)
    {
        return status;
    }

    const uint8_t stat1 = buffer[IST8310_Regs::DATA_BURST_STAT1_INDEX];

    if ((stat1 & ToByte(IST8310_Regs::STAT1_BITS::DOR)) != IST8310_Regs::BitNone)
    {
        return Status::DataOverrun;
    }

    if ((stat1 & ToByte(IST8310_Regs::STAT1_BITS::DRDY)) == IST8310_Regs::BitNone)
    {
        return Status::DataNotReady;
    }

    RawMagData new_data{};
    new_data.x = CombineLittleEndian(buffer[IST8310_Regs::DATA_BURST_X_L_INDEX],
                                     buffer[IST8310_Regs::DATA_BURST_X_H_INDEX]);
    new_data.y = CombineLittleEndian(buffer[IST8310_Regs::DATA_BURST_Y_L_INDEX],
                                     buffer[IST8310_Regs::DATA_BURST_Y_H_INDEX]);
    new_data.z = CombineLittleEndian(buffer[IST8310_Regs::DATA_BURST_Z_L_INDEX],
                                     buffer[IST8310_Regs::DATA_BURST_Z_H_INDEX]);

    data = new_data;
    return Status::Ok;
}

IST8310::Status IST8310::ReadMag_uT(MagData_uT& data)
{
    RawMagData raw_data{};
    const Status status = ReadRawMag(raw_data);

    if (status != Status::Ok)
    {
        return status;
    }

    MagData_uT new_data{};
    new_data.x = static_cast<float>(raw_data.x) * scale_uT_per_lsb_;
    new_data.y = static_cast<float>(raw_data.y) * scale_uT_per_lsb_;
    new_data.z = static_cast<float>(raw_data.z) * scale_uT_per_lsb_;

    data = new_data;
    return Status::Ok;
}

IST8310::Status IST8310::ReleaseHardwareReset() const
{
    if (reset_port_ == nullptr && reset_pin_ == 0u)
    {
        return Status::Ok;
    }

    if (reset_port_ == nullptr || reset_pin_ == 0u)
    {
        return Status::InvalidArgument;
    }

    HAL_GPIO_WritePin(reset_port_, reset_pin_, GPIO_PIN_SET);
    HAL_Delay(IST8310_Regs::POR_RESET_WAIT_MS);

    return Status::Ok;
}

IST8310::Status IST8310::Configure()
{
    Status status = RegisterWrite(IST8310_Regs::Register::PDCNTL,
                                  IST8310_Regs::PDCNTL_NORMAL_RECOMMENDED);

    if (status != Status::Ok)
    {
        return status;
    }

    status = RegisterWrite(IST8310_Regs::Register::AVGCNTL,
                           IST8310_Regs::AVGCNTL_LOW_NOISE_RECOMMENDED);

    if (status != Status::Ok)
    {
        return status;
    }

    if constexpr (IST8310_Regs::DEFAULT_USE_PX4_16BIT_MODE)
    {
        status = RegisterWrite(IST8310_Regs::Register::CNTL3,
                               IST8310_Regs::CNTL3_PX4_16BIT_CONFIG);

        if (status != Status::Ok)
        {
            return status;
        }
    }

    uint8_t register_value{};
    status = RegisterRead(IST8310_Regs::Register::PDCNTL, register_value);

    if (status != Status::Ok)
    {
        return status;
    }

    if (register_value != IST8310_Regs::PDCNTL_NORMAL_RECOMMENDED)
    {
        return Status::ConfigMismatch;
    }

    status = RegisterRead(IST8310_Regs::Register::AVGCNTL, register_value);

    if (status != Status::Ok)
    {
        return status;
    }

    if (register_value != IST8310_Regs::AVGCNTL_LOW_NOISE_RECOMMENDED)
    {
        return Status::ConfigMismatch;
    }

    if constexpr (IST8310_Regs::DEFAULT_USE_PX4_16BIT_MODE)
    {
        status = RegisterRead(IST8310_Regs::Register::CNTL3, register_value);

        if (status != Status::Ok)
        {
            return status;
        }

        if (register_value != IST8310_Regs::CNTL3_PX4_16BIT_CONFIG)
        {
            return Status::ConfigMismatch;
        }
    }

    scale_uT_per_lsb_ = IST8310_Regs::DEFAULT_UT_PER_LSB;
    return Status::Ok;
}

IST8310::Status IST8310::StartSingleMeasurement() const
{
    return RegisterWrite(IST8310_Regs::Register::CNTL1,
                         ToByte(IST8310_Regs::CNTL1_VALUES::SINGLE_MEASUREMENT));
}

IST8310::Status IST8310::WaitDataReady() const
{
    const uint32_t measurement_start = HAL_GetTick();
    HAL_Delay(IST8310_Regs::SINGLE_MEASUREMENT_MIN_WAIT_MS);

    while ((HAL_GetTick() - measurement_start) <= IST8310_Regs::SINGLE_MEASUREMENT_TIMEOUT_MS)
    {
        uint8_t stat1{};
        const Status status = RegisterRead(IST8310_Regs::Register::STAT1, stat1);

        if (status != Status::Ok)
        {
            return status;
        }

        if ((stat1 & ToByte(IST8310_Regs::STAT1_BITS::DRDY)) != IST8310_Regs::BitNone)
        {
            return Status::Ok;
        }

        HAL_Delay(IST8310_Regs::DATA_READY_POLL_INTERVAL_MS);
    }

    return Status::DataNotReady;
}

uint16_t IST8310::HalDeviceAddress() const
{
    return static_cast<uint16_t>(address_7bit_) << IST8310_Regs::HAL_I2C_ADDRESS_SHIFT;
}

int16_t IST8310::CombineLittleEndian(const uint8_t low, const uint8_t high)
{
    const uint16_t combined = static_cast<uint16_t>(
        (static_cast<uint16_t>(high) << IST8310_Regs::HIGH_BYTE_SHIFT) | low);

    return static_cast<int16_t>(combined);
}
