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

namespace
{
constexpr uint16_t SENSOR_DATA_BURST_LENGTH{14u};
constexpr uint8_t TEMP_HIGH_INDEX{0u};
constexpr uint8_t TEMP_LOW_INDEX{1u};
constexpr uint8_t ACCEL_X_HIGH_INDEX{2u};
constexpr uint8_t ACCEL_X_LOW_INDEX{3u};
constexpr uint8_t ACCEL_Y_HIGH_INDEX{4u};
constexpr uint8_t ACCEL_Y_LOW_INDEX{5u};
constexpr uint8_t ACCEL_Z_HIGH_INDEX{6u};
constexpr uint8_t ACCEL_Z_LOW_INDEX{7u};
constexpr uint8_t GYRO_X_HIGH_INDEX{8u};
constexpr uint8_t GYRO_X_LOW_INDEX{9u};
constexpr uint8_t GYRO_Y_HIGH_INDEX{10u};
constexpr uint8_t GYRO_Y_LOW_INDEX{11u};
constexpr uint8_t GYRO_Z_HIGH_INDEX{12u};
constexpr uint8_t GYRO_Z_LOW_INDEX{13u};

constexpr float STANDARD_GRAVITY_M_S2{9.80665F};
constexpr float ACCEL_LSB_PER_G{2048.0F};
constexpr float GYRO_FULL_SCALE_DPS{2000.0F};
constexpr float SIGNED_16BIT_HALF_RANGE{32768.0F};
constexpr float DEG_TO_RAD{0.01745329251994329577F};
constexpr float TEMPERATURE_SENSITIVITY_LSB_PER_DEG_C{132.48F};
constexpr float TEMPERATURE_OFFSET_DEG_C{25.0F};
}

ICM42688P::ICM42688P(SPI_HandleTypeDef* hspi,
                     GPIO_TypeDef* cs_port, uint16_t cs_pin)
    : hspi_(hspi), cs_port_(cs_port), cs_pin_(cs_pin)
{
}

ICM42688P::Status ICM42688P::Init()
{
    initialized_ = false;
    configured_ = false;
    state_ = DriverState::Uninitialized;
    last_status_ = Status::Ok;
    latest_.configured = false;
    latest_.data_valid = false;

    if (!HasValidBus()) {
        ++error_count_;
        ++failure_count_;
        last_status_ = Status::InvalidArgument;
        state_ = DriverState::Error;
        return Status::InvalidArgument;
    }

    CS_High();
    HAL_Delay(ICM42688P_Regs::POWER_ON_WAIT_MS);

    state_ = DriverState::Probing;
    Status status = Probe();

    if (status != Status::Ok) {
        ++error_count_;
        ++failure_count_;
        last_status_ = status;
        state_ = DriverState::Error;
        return status;
    }

    state_ = DriverState::Resetting;
    status = Reset();

    if (status != Status::Ok) {
        ++error_count_;
        ++failure_count_;
        last_status_ = status;
        state_ = DriverState::Error;
        return status;
    }

    state_ = DriverState::Probing;
    status = Probe();

    if (status != Status::Ok) {
        ++error_count_;
        ++failure_count_;
        last_status_ = status;
        state_ = DriverState::Error;
        return status;
    }

    state_ = DriverState::Configuring;
    status = Configure();

    if (status != Status::Ok) {
        ++error_count_;
        ++failure_count_;
        last_status_ = status;
        state_ = DriverState::Error;
        return status;
    }

    initialized_ = true;
    configured_ = true;
    state_ = DriverState::Running;
    last_status_ = Status::Ok;
    failure_count_ = 0u;
    latest_.configured = true;
    latest_.error_counter = error_count_;
    return Status::Ok;
}

ICM42688P::Status ICM42688P::Configure()
{
    configured_ = false;
    latest_.configured = false;

    for (const auto& config : register_bank1_cfg_) {
        const Status status = RegisterSetAndClearBits(config);

        if (status != Status::Ok) {
            return status;
        }
    }

    for (const auto& config : register_bank2_cfg_) {
        const Status status = RegisterSetAndClearBits(config);

        if (status != Status::Ok) {
            return status;
        }
    }

    const register_bank0_config_t* power_config = nullptr;

    for (const auto& config : register_bank0_cfg_) {
        if (config.reg == BANK0::PWR_MGMT0) {
            power_config = &config;
            continue;
        }

        if (!ShouldApplyPollingConfig(config.reg)) {
            continue;
        }

        const Status status = RegisterSetAndClearBits(config);

        if (status != Status::Ok) {
            return status;
        }
    }

    if (power_config == nullptr) {
        return Status::ConfigMismatch;
    }

    Status status = RegisterSetAndClearBits(*power_config);

    if (status != Status::Ok) {
        return status;
    }

    HAL_Delay(ICM42688P_Regs::SENSOR_MODE_CHANGE_WAIT_MS);
    HAL_Delay(ICM42688P_Regs::SENSOR_STARTUP_WAIT_MS);

    for (const auto& config : register_bank1_cfg_) {
        status = RegisterCheck(config);

        if (status != Status::Ok) {
            return status;
        }
    }

    for (const auto& config : register_bank2_cfg_) {
        status = RegisterCheck(config);

        if (status != Status::Ok) {
            return status;
        }
    }

    for (const auto& config : register_bank0_cfg_) {
        if (!ShouldApplyPollingConfig(config.reg)) {
            continue;
        }

        status = RegisterCheck(config);

        if (status != Status::Ok) {
            return status;
        }
    }

    configured_ = true;
    latest_.configured = true;
    return Status::Ok;
}

ICM42688P::Status ICM42688P::RegisterSetAndClearBits(const register_bank0_config_t& config)
{
    uint8_t old_value{};
    Status status = RegisterRead(config.reg, old_value);

    if (status != Status::Ok) {
        return status;
    }

    const uint8_t new_value = ComposeRegisterValue(old_value, config.setBits, config.mask);
    return new_value == old_value ? Status::Ok : RegisterWrite(config.reg, new_value);
}

ICM42688P::Status ICM42688P::RegisterSetAndClearBits(const register_bank1_config_t& config)
{
    uint8_t old_value{};
    Status status = ReadRegister(config.reg, old_value);

    if (status != Status::Ok) {
        return status;
    }

    const uint8_t new_value = ComposeRegisterValue(old_value, config.setBits, config.mask);
    return new_value == old_value ? Status::Ok : WriteRegister(config.reg, new_value);
}

ICM42688P::Status ICM42688P::RegisterSetAndClearBits(const register_bank2_config_t& config)
{
    uint8_t old_value{};
    Status status = ReadRegister(config.reg, old_value);

    if (status != Status::Ok) {
        return status;
    }

    const uint8_t new_value = ComposeRegisterValue(old_value, config.setBits, config.mask);
    return new_value == old_value ? Status::Ok : WriteRegister(config.reg, new_value);
}

ICM42688P::Status ICM42688P::RegisterCheck(const register_bank0_config_t& config)
{
    uint8_t value{};
    const Status status = RegisterRead(config.reg, value);

    if (status != Status::Ok) {
        return status;
    }

    return RegisterValueMatches(value, config.setBits, config.mask)
        ? Status::Ok
        : Status::ConfigMismatch;
}

ICM42688P::Status ICM42688P::RegisterCheck(const register_bank1_config_t& config)
{
    uint8_t value{};
    const Status status = ReadRegister(config.reg, value);

    if (status != Status::Ok) {
        return status;
    }

    return RegisterValueMatches(value, config.setBits, config.mask)
        ? Status::Ok
        : Status::ConfigMismatch;
}

ICM42688P::Status ICM42688P::RegisterCheck(const register_bank2_config_t& config)
{
    uint8_t value{};
    const Status status = ReadRegister(config.reg, value);

    if (status != Status::Ok) {
        return status;
    }

    return RegisterValueMatches(value, config.setBits, config.mask)
        ? Status::Ok
        : Status::ConfigMismatch;
}

ICM42688P::Status ICM42688P::VerifyRegister(const BANK0 reg, const uint8_t expected_value)
{
    uint8_t readback{};
    const Status status = RegisterRead(reg, readback);

    if (status != Status::Ok) {
        return status;
    }

    return readback == expected_value ? Status::Ok : Status::ConfigMismatch;
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
    configured_ = false;
    latest_.configured = false;
    latest_.data_valid = false;
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

ICM42688P::Status ICM42688P::Update()
{
    if (!initialized_ || !configured_) {
        return Status::InvalidArgument;
    }

    uint8_t buffer[SENSOR_DATA_BURST_LENGTH]{};
    const Status status = ReadBuffer(BANK0::TEMP_DATA1, buffer, SENSOR_DATA_BURST_LENGTH);

    if (status != Status::Ok) {
        ++error_count_;
        latest_.error_counter = error_count_;
        latest_.data_valid = false;
        return status;
    }

    Sample sample{};
    sample.temp_raw = CombineBigEndian(buffer[TEMP_HIGH_INDEX], buffer[TEMP_LOW_INDEX]);
    sample.accel_raw[0] = CombineBigEndian(buffer[ACCEL_X_HIGH_INDEX], buffer[ACCEL_X_LOW_INDEX]);
    sample.accel_raw[1] = CombineBigEndian(buffer[ACCEL_Y_HIGH_INDEX], buffer[ACCEL_Y_LOW_INDEX]);
    sample.accel_raw[2] = CombineBigEndian(buffer[ACCEL_Z_HIGH_INDEX], buffer[ACCEL_Z_LOW_INDEX]);
    sample.gyro_raw[0] = CombineBigEndian(buffer[GYRO_X_HIGH_INDEX], buffer[GYRO_X_LOW_INDEX]);
    sample.gyro_raw[1] = CombineBigEndian(buffer[GYRO_Y_HIGH_INDEX], buffer[GYRO_Y_LOW_INDEX]);
    sample.gyro_raw[2] = CombineBigEndian(buffer[GYRO_Z_HIGH_INDEX], buffer[GYRO_Z_LOW_INDEX]);

    if (sample.accel_raw[0] == INT16_MIN
        && sample.accel_raw[1] == INT16_MIN
        && sample.accel_raw[2] == INT16_MIN
        && sample.gyro_raw[0] == INT16_MIN
        && sample.gyro_raw[1] == INT16_MIN
        && sample.gyro_raw[2] == INT16_MIN) {
        ++error_count_;
        latest_.error_counter = error_count_;
        latest_.data_valid = false;
        return Status::ConfigMismatch;
    }

    constexpr float accel_scale = STANDARD_GRAVITY_M_S2 / ACCEL_LSB_PER_G;
    constexpr float gyro_scale = (GYRO_FULL_SCALE_DPS / SIGNED_16BIT_HALF_RANGE) * DEG_TO_RAD;

    for (uint8_t axis = 0u; axis < 3u; ++axis) {
        sample.accel_m_s2[axis] = static_cast<float>(sample.accel_raw[axis]) * accel_scale;
        sample.gyro_rad_s[axis] = static_cast<float>(sample.gyro_raw[axis]) * gyro_scale;
    }

    sample.temperature_deg_c = static_cast<float>(sample.temp_raw)
                               / TEMPERATURE_SENSITIVITY_LSB_PER_DEG_C
                               + TEMPERATURE_OFFSET_DEG_C;
    sample.timestamp_ms = HAL_GetTick();
    sample.sample_counter = ++sample_count_;
    sample.error_counter = error_count_;
    sample.configured = configured_;
    sample.data_valid = true;

    last_update_ms_ = sample.timestamp_ms;
    latest_ = sample;
    return Status::Ok;
}

ICM42688P::Status ICM42688P::GetLatest(Sample& sample) const
{
    if (!initialized_) {
        return Status::InvalidArgument;
    }

    sample = latest_;
    return Status::Ok;
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
