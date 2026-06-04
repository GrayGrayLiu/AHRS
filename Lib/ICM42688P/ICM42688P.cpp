/******************************************************************************
 * @file    ICM42688P.cpp
 * @brief   ICM42688P源文件
 *
 * @details
 *
 * @author  Gray
 * @email   grayme12345@gmail.com
 * @date    2026/1/14
 *
 * @copyright
 * Copyright (c) 2026 Gray
 *
 * This software is provided "as is" without any warranties.
 *
 ******************************************************************************/

#include "ICM42688P.hpp"
#include <cstring>

using ICM42688P_Regs::RegsAdd::BANK0;
using ICM42688P_Regs::RegsAdd::BANK1;
using ICM42688P_Regs::RegsAdd::BANK2;

ICM42688P::ICM42688P(SPI_HandleTypeDef* hspi,
                     GPIO_TypeDef* cs_port, uint16_t cs_pin)
    : hspi_(hspi), cs_port_(cs_port), cs_pin_(cs_pin)
{
}

ICM42688P::Status ICM42688P::Init()
{
    initialized_ = false;

    if (!HasValidBus()) {
        ++error_count_;
        return Status::InvalidArgument;
    }

    CS_High();
    HAL_Delay(ICM42688P_Regs::POWER_ON_WAIT_MS);

    Status status = Probe();

    if (status != Status::Ok) {
        ++error_count_;
        return status;
    }

    status = Reset();

    if (status != Status::Ok) {
        ++error_count_;
        return status;
    }

    status = Probe();

    if (status != Status::Ok) {
        ++error_count_;
        return status;
    }

    initialized_ = true;
    return Status::Ok;
}

ICM42688P::Status ICM42688P::Probe()
{
    if (!HasValidBus()) {
        return Status::InvalidArgument;
    }

    const Status bank_status = SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_0, true);

    if (bank_status != Status::Ok) {
        return bank_status;
    }

    uint8_t who_am_i{};
    const Status read_status = ReadRegisterRaw(static_cast<uint8_t>(BANK0::WHO_AM_I), who_am_i);

    if (read_status != Status::Ok) {
        return read_status;
    }

    return who_am_i == ICM42688P_Regs::WHO_AM_I_EXPECTED
        ? Status::Ok
        : Status::WrongDeviceId;
}

ICM42688P::Status ICM42688P::Reset()
{
    if (!HasValidBus()) {
        return Status::InvalidArgument;
    }

    initialized_ = false;
    Status status = RegisterWrite(BANK0::DEVICE_CONFIG,
                                  static_cast<uint8_t>(ICM42688P_Regs::DEVICE_CONFIG_BITS::SOFT_RESET_CONFIG));

    if (status != Status::Ok) {
        return status;
    }

    HAL_Delay(ICM42688P_Regs::SOFT_RESET_WAIT_MS);
    bank_selected_valid_ = false;

    const uint32_t reset_start = HAL_GetTick();

    while ((HAL_GetTick() - reset_start) <= ICM42688P_Regs::SOFT_RESET_TIMEOUT_MS) {
        uint8_t device_config{};
        status = RegisterRead(BANK0::DEVICE_CONFIG, device_config);

        if (status != Status::Ok) {
            return status;
        }

        if ((device_config & static_cast<uint8_t>(ICM42688P_Regs::DEVICE_CONFIG_BITS::SOFT_RESET_CONFIG))
            == ICM42688P_Regs::BitNone) {
            return Status::Ok;
        }

        HAL_Delay(ICM42688P_Regs::RESET_POLL_INTERVAL_MS);
    }

    return Status::ResetTimeout;
}

ICM42688P::Status ICM42688P::RegisterRead(const BANK0 reg, uint8_t& value)
{
    const Status status = SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_0);
    return status == Status::Ok ? ReadRegisterRaw(static_cast<uint8_t>(reg), value) : status;
}

ICM42688P::Status ICM42688P::RegisterWrite(const BANK0 reg, const uint8_t value)
{
    const Status status = SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_0);
    return status == Status::Ok ? WriteRegisterRaw(static_cast<uint8_t>(reg), value) : status;
}

ICM42688P::Status ICM42688P::ReadBuffer(const BANK0 start_reg,
                                        uint8_t* buffer,
                                        const uint16_t length)
{
    if (buffer == nullptr
        || length == 0u
        || length > ICM42688P_Regs::MAX_READ_LENGTH) {
        return Status::InvalidArgument;
    }

    const Status status = SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_0);
    return status == Status::Ok ? ReadBufferRaw(static_cast<uint8_t>(start_reg), buffer, length) : status;
}

ICM42688P::Status ICM42688P::ReadRawAccel(RawVector& data)
{
    uint8_t buffer[ICM42688P_Regs::RAW_ACCEL_BURST_LENGTH]{};
    const Status status = ReadBuffer(BANK0::ACCEL_DATA_X1,
                                     buffer,
                                     ICM42688P_Regs::RAW_ACCEL_BURST_LENGTH);

    if (status != Status::Ok) {
        return status;
    }

    RawVector new_data{};
    new_data.x = CombineBigEndian(buffer[ICM42688P_Regs::RAW_X_HIGH_INDEX],
                                  buffer[ICM42688P_Regs::RAW_X_LOW_INDEX]);
    new_data.y = CombineBigEndian(buffer[ICM42688P_Regs::RAW_Y_HIGH_INDEX],
                                  buffer[ICM42688P_Regs::RAW_Y_LOW_INDEX]);
    new_data.z = CombineBigEndian(buffer[ICM42688P_Regs::RAW_Z_HIGH_INDEX],
                                  buffer[ICM42688P_Regs::RAW_Z_LOW_INDEX]);
    data = new_data;
    return Status::Ok;
}

ICM42688P::Status ICM42688P::ReadRawGyro(RawVector& data)
{
    uint8_t buffer[ICM42688P_Regs::RAW_GYRO_BURST_LENGTH]{};
    const Status status = ReadBuffer(BANK0::GYRO_DATA_X1,
                                     buffer,
                                     ICM42688P_Regs::RAW_GYRO_BURST_LENGTH);

    if (status != Status::Ok) {
        return status;
    }

    RawVector new_data{};
    new_data.x = CombineBigEndian(buffer[ICM42688P_Regs::RAW_X_HIGH_INDEX],
                                  buffer[ICM42688P_Regs::RAW_X_LOW_INDEX]);
    new_data.y = CombineBigEndian(buffer[ICM42688P_Regs::RAW_Y_HIGH_INDEX],
                                  buffer[ICM42688P_Regs::RAW_Y_LOW_INDEX]);
    new_data.z = CombineBigEndian(buffer[ICM42688P_Regs::RAW_Z_HIGH_INDEX],
                                  buffer[ICM42688P_Regs::RAW_Z_LOW_INDEX]);
    data = new_data;
    return Status::Ok;
}

bool ICM42688P::Update()
{
    // Stage1: no health check and no data path.
    return false;
}

bool ICM42688P::ReadLatest(int16_t accel[3], int16_t gyro[3], int16_t *temp) const
{
    (void)accel;
    (void)gyro;
    (void)temp;

    // Stage1: no sample fetch.
    return false;
}

void ICM42688P::CS_Low() const
{
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
}

void ICM42688P::CS_High() const
{
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
}

ICM42688P::Status ICM42688P::SelectBank(const ICM42688P_Regs::REG_BANK_SEL_BITS bank,
                                        const bool force)
{
    if (!force && bank_selected_valid_ && bank == current_bank_) {
        return Status::Ok;
    }

    const uint8_t bank_value = static_cast<uint8_t>(bank) &
        static_cast<uint8_t>(ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_MASK);

    const Status status = WriteRegisterRaw(static_cast<uint8_t>(BANK0::REG_BANK_SEL), bank_value);

    if (status != Status::Ok) {
        return status;
    }

    current_bank_ = bank;
    bank_selected_valid_ = true;
    return Status::Ok;
}

ICM42688P::Status ICM42688P::WriteRegister(const BANK1 reg, const uint8_t value)
{
    const Status status = SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_1);
    return status == Status::Ok ? WriteRegisterRaw(static_cast<uint8_t>(reg), value) : status;
}

ICM42688P::Status ICM42688P::WriteRegister(const BANK2 reg, const uint8_t value)
{
    const Status status = SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_2);
    return status == Status::Ok ? WriteRegisterRaw(static_cast<uint8_t>(reg), value) : status;
}

ICM42688P::Status ICM42688P::ReadRegister(const BANK1 reg, uint8_t& value)
{
    const Status status = SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_1);
    return status == Status::Ok ? ReadRegisterRaw(static_cast<uint8_t>(reg), value) : status;
}

ICM42688P::Status ICM42688P::ReadRegister(const BANK2 reg, uint8_t& value)
{
    const Status status = SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_2);
    return status == Status::Ok ? ReadRegisterRaw(static_cast<uint8_t>(reg), value) : status;
}

ICM42688P::Status ICM42688P::WriteRegisterRaw(const uint8_t reg, const uint8_t value) const
{
    if (!HasValidBus()) {
        return Status::InvalidArgument;
    }

    uint8_t tx[ICM42688P_Regs::REGISTER_TRANSACTION_LENGTH] = {
        static_cast<uint8_t>(reg & ICM42688P_Regs::SPI_WRITE_ADDRESS_MASK),
        value
    };

    CS_Low();
    const HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi_,
                                                      tx,
                                                      ICM42688P_Regs::REGISTER_TRANSACTION_LENGTH,
                                                      ICM42688P_Regs::SPI_TRANSACTION_TIMEOUT_MS);
    CS_High();

    return status == HAL_OK ? Status::Ok : Status::SpiError;
}

ICM42688P::Status ICM42688P::ReadRegisterRaw(const uint8_t reg, uint8_t& value)
{
    return ReadBufferRaw(reg, &value, ICM42688P_Regs::REGISTER_VALUE_LENGTH);
}

ICM42688P::Status ICM42688P::ReadBufferRaw(const uint8_t start_reg,
                                           uint8_t* buffer,
                                           const uint16_t length)
{
    if (!HasValidBus()
        || buffer == nullptr
        || length == 0u
        || length > ICM42688P_Regs::MAX_READ_LENGTH) {
        return Status::InvalidArgument;
    }

    tx_buf_[0] = static_cast<uint8_t>(start_reg | ICM42688P_Regs::SPI_READ_BIT);
    memset(&tx_buf_[ICM42688P_Regs::SPI_COMMAND_LENGTH],
           ICM42688P_Regs::SPI_DUMMY_BYTE,
           length);

    CS_Low();
    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        hspi_,
        tx_buf_,
        rx_buf_,
        static_cast<uint16_t>(length + ICM42688P_Regs::SPI_COMMAND_LENGTH),
        ICM42688P_Regs::SPI_TRANSACTION_TIMEOUT_MS);
    CS_High();

    if (status != HAL_OK) {
        return Status::SpiError;
    }

    memcpy(buffer, &rx_buf_[ICM42688P_Regs::SPI_COMMAND_LENGTH], length);
    return Status::Ok;
}

bool ICM42688P::HasValidBus() const
{
    return hspi_ != nullptr && cs_port_ != nullptr && cs_pin_ != 0u;
}

int16_t ICM42688P::CombineBigEndian(const uint8_t high, const uint8_t low)
{
    const uint16_t combined = static_cast<uint16_t>(
        (static_cast<uint16_t>(high) << ICM42688P_Regs::RAW_HIGH_BYTE_SHIFT) | low);
    return static_cast<int16_t>(combined);
}
