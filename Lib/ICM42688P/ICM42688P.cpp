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
constexpr float STANDARD_GRAVITY_M_S2{9.80665F};
constexpr float DEG_TO_RAD{0.01745329251994329577F};
constexpr float TEMPERATURE_SENSITIVITY_LSB_PER_DEG_C{132.48F};
constexpr float TEMPERATURE_OFFSET_DEG_C{25.0F};
constexpr uint32_t RESET_CHECK_INTERVAL_MS{10u};
constexpr uint32_t RESET_RETRY_DELAY_MS{100u};
constexpr uint32_t CONFIGURE_RETRY_DELAY_MS{100u};
constexpr uint32_t DRIVER_RESET_TIMEOUT_MS{1000u};
constexpr uint32_t FIFO_WATCHDOG_INTERVAL_MS{100u};
constexpr uint32_t FAILURE_RESET_THRESHOLD{10u};
}

ICM42688P::ICM42688P(SPI_HandleTypeDef* hspi,
                     GPIO_TypeDef* cs_port, uint16_t cs_pin)
    : hspi_(hspi), cs_port_(cs_port), cs_pin_(cs_pin)
{
}

ICM42688P::Status ICM42688P::Init()
{
    if (!HasValidBus()) {
        ++error_count_;
        last_status_ = Status::InvalidArgument;
        return Status::InvalidArgument;
    }

    CS_High();
    latest_ = Sample{};
    fifo_last_decoded_ = FifoDecodedSample{};
    fifo_decoded_count_ = 0u;
    accel_last_effective_valid_ = false;
    gyro_last_effective_valid_ = false;
    data_ready_pending_ = 0u;
    data_ready_interrupt_count_ = 0u;
    last_data_ready_timestamp_ms_ = 0u;
    bank_selected_valid_ = false;
    failure_count_ = 0u;
    reset_timestamp_ms_ = 0u;
    initialized_ = true;
    configured_ = false;
    state_ = DriverState::RESET;
    last_status_ = Status::Ok;
    latest_.error_counter = error_count_;
    ScheduleNow(HAL_GetTick());
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

    configured_ = false;
    latest_.configured = false;
    latest_.data_valid = false;
    initialized_ = true;
    state_ = DriverState::RESET;
    ScheduleNow(HAL_GetTick());
    return Status::Ok;
}

ICM42688P::Status ICM42688P::WaitForReset()
{
    uint8_t who_am_i{};
    uint8_t device_config{};
    uint8_t int_status{};
    Status status = RegisterRead(BANK0::WHO_AM_I, who_am_i);

    if (status != Status::Ok) {
        return status;
    }

    status = RegisterRead(BANK0::DEVICE_CONFIG, device_config);

    if (status != Status::Ok) {
        return status;
    }

    status = RegisterRead(BANK0::INT_STATUS, int_status);

    if (status != Status::Ok) {
        return status;
    }

    if (who_am_i != ICM42688P_Regs::WHO_AM_I_EXPECTED) {
        return Status::WrongDeviceId;
    }

    const bool reset_bit_clear =
        (device_config
         & static_cast<uint8_t>(ICM42688P_Regs::DEVICE_CONFIG_BITS::SOFT_RESET_CONFIG))
        == ICM42688P_Regs::BitNone;
    const bool reset_done =
        (int_status
         & static_cast<uint8_t>(ICM42688P_Regs::INT_STATUS_BITS::RESET_DONE_INT))
        != ICM42688P_Regs::BitNone;

    return reset_bit_clear && reset_done ? Status::Ok : Status::NoData;
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

    const register_bank0_config_t* fifo_config1 = nullptr;
    const register_bank0_config_t* power_config = nullptr;

    for (const auto& config : register_bank0_cfg_) {
        if (config.reg == BANK0::PWR_MGMT0) {
            power_config = &config;
            continue;
        }

        if (config.reg == BANK0::FIFO_CONFIG1) {
            fifo_config1 = &config;
            continue;
        }

        if (!ShouldApplyFifoInterruptConfig(config.reg)) {
            continue;
        }

        const Status status = RegisterSetAndClearBits(config);

        if (status != Status::Ok) {
            return status;
        }
    }

    if (fifo_config1 == nullptr || power_config == nullptr) {
        return Status::ConfigMismatch;
    }

    Status status = ConfigureFIFOWatermark(FIFO_WATERMARK_BYTES);

    if (status != Status::Ok) {
        return status;
    }

    status = RegisterSetAndClearBits(*fifo_config1);

    if (status != Status::Ok) {
        return status;
    }

    status = RegisterSetAndClearBits(*power_config);

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
        if (!ShouldApplyFifoInterruptConfig(config.reg)) {
            continue;
        }

        status = RegisterCheck(config);

        if (status != Status::Ok) {
            return status;
        }
    }

    return Status::Ok;
}

ICM42688P::Status ICM42688P::ConfigureFIFOWatermark(const uint16_t watermark_bytes)
{
    constexpr uint16_t maximum_watermark{0x0FFFu};

    if (watermark_bytes == 0u || watermark_bytes > maximum_watermark) {
        return Status::InvalidArgument;
    }

    const register_bank0_config_t watermark_low{
        BANK0::FIFO_CONFIG2,
        static_cast<uint8_t>(watermark_bytes & 0x00FFu),
        static_cast<uint8_t>(ICM42688P_Regs::FIFO_CONFIG2_BITS::FIFO_WM_7_0_MASK)
    };
    const register_bank0_config_t watermark_high{
        BANK0::FIFO_CONFIG3,
        static_cast<uint8_t>((watermark_bytes >> 8u) & 0x0Fu),
        static_cast<uint8_t>(ICM42688P_Regs::FIFO_CONFIG3_BITS::FIFO_WM_11_8_MASK)
    };

    Status status = RegisterSetAndClearBits(watermark_low);

    if (status != Status::Ok) {
        return status;
    }

    status = RegisterSetAndClearBits(watermark_high);

    if (status != Status::Ok) {
        return status;
    }

    status = RegisterCheck(watermark_low);

    if (status != Status::Ok) {
        return status;
    }

    return RegisterCheck(watermark_high);
}

ICM42688P::Status ICM42688P::ConfigureInterrupt()
{
    // CubeMX/HAL owns the EXTI edge configuration. This verifies the sensor-side
    // INT register configuration and resets the software event latch.
    for (const auto& config : register_bank0_cfg_) {
        if (!IsInterruptConfigRegister(config.reg)) {
            continue;
        }

        const Status status = RegisterCheck(config);

        if (status != Status::Ok) {
            return status;
        }
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    data_ready_pending_ = 0u;
    data_ready_interrupt_count_ = 0u;
    last_data_ready_timestamp_ms_ = 0u;

    if (primask == 0u) {
        __enable_irq();
    }

    return Status::Ok;
}

ICM42688P::Status ICM42688P::FIFOReset()
{
    const Status status = RegisterWrite(
        BANK0::SIGNAL_PATH_RESET,
        static_cast<uint8_t>(ICM42688P_Regs::SIGNAL_PATH_RESET_BITS::FIFO_FLUSH));

    if (status != Status::Ok) {
        return status;
    }

    HAL_Delay(1u);
    last_fifo_count_bytes_ = 0u;
    last_fifo_valid_packets_ = 0u;
    fifo_decoded_count_ = 0u;
    accel_last_effective_valid_ = false;
    gyro_last_effective_valid_ = false;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    data_ready_pending_ = 0u;

    if (primask == 0u) {
        __enable_irq();
    }

    return Status::Ok;
}

ICM42688P::Status ICM42688P::Update()
{
    return RunImpl();
}

ICM42688P::Status ICM42688P::RunImpl()
{
    if (!initialized_) {
        return Status::InvalidArgument;
    }

    const uint32_t now = HAL_GetTick();
    const bool data_ready_scheduled =
        state_ == DriverState::FIFO_READ && data_ready_pending_ != 0u;

    // A pending INT1 event is the bare-metal ScheduleNow equivalent. The ISR
    // only records the event; RunImpl remains responsible for scheduling work.
    if (!data_ready_scheduled && !TickReached(now, next_run_ms_)) {
        return Status::NoData;
    }

    // Bare-metal equivalent of PX4 RunImpl(): next_run_ms_ replaces the work
    // queue's ScheduleNow/ScheduleDelayed timing while Service calls us at 1 kHz.
    switch (state_) {
    case DriverState::RESET: {
        configured_ = false;
        latest_.configured = false;
        latest_.data_valid = false;
        failure_count_ = 0u;

        const Status status = RegisterWrite(
            BANK0::DEVICE_CONFIG,
            static_cast<uint8_t>(ICM42688P_Regs::DEVICE_CONFIG_BITS::SOFT_RESET_CONFIG));

        if (status != Status::Ok) {
            ++error_count_;
            ++failure_count_;
            last_status_ = status;
            ScheduleDelayed(now, RESET_RETRY_DELAY_MS);
            return status;
        }

        bank_selected_valid_ = false;
        reset_timestamp_ms_ = now;
        state_ = DriverState::WAIT_FOR_RESET;
        last_status_ = Status::NoData;
        ScheduleDelayed(now, ICM42688P_Regs::SOFT_RESET_WAIT_MS);
        return Status::NoData;
    }

    case DriverState::WAIT_FOR_RESET: {
        const Status status = WaitForReset();

        if (status == Status::Ok) {
            state_ = DriverState::CONFIGURE;
            last_status_ = Status::NoData;
            ScheduleNow(now);
            return Status::NoData;
        }

        if ((now - reset_timestamp_ms_) > DRIVER_RESET_TIMEOUT_MS) {
            ++error_count_;
            ++failure_count_;
            last_status_ = status == Status::WrongDeviceId
                ? Status::WrongDeviceId
                : Status::ResetTimeout;
            state_ = DriverState::RESET;
            ScheduleDelayed(now, RESET_RETRY_DELAY_MS);
            return last_status_;
        }

        if (status != Status::NoData && status != Status::WrongDeviceId) {
            ++error_count_;
        }

        last_status_ = status;
        ScheduleDelayed(now, RESET_CHECK_INTERVAL_MS);
        return Status::NoData;
    }

    case DriverState::CONFIGURE: {
        Status status = Configure();

        if (status == Status::Ok) {
            status = ConfigureInterrupt();
        }

        if (status == Status::Ok) {
            state_ = DriverState::FIFO_RESET;
            last_status_ = Status::NoData;
            ScheduleDelayed(now, ICM42688P_Regs::SOFT_RESET_WAIT_MS);
            return Status::NoData;
        }

        ++error_count_;
        ++failure_count_;
        configured_ = false;
        latest_.configured = false;
        last_status_ = status;

        if ((now - reset_timestamp_ms_) > DRIVER_RESET_TIMEOUT_MS
            || failure_count_ > FAILURE_RESET_THRESHOLD) {
            state_ = DriverState::RESET;
        }

        ScheduleDelayed(now, CONFIGURE_RETRY_DELAY_MS);
        return status;
    }

    case DriverState::FIFO_RESET: {
        const Status status = FIFOReset();

        if (status != Status::Ok) {
            ++error_count_;
            ++failure_count_;
            last_status_ = status;

            if (failure_count_ > FAILURE_RESET_THRESHOLD) {
                state_ = DriverState::RESET;
            }

            ScheduleDelayed(now, RESET_RETRY_DELAY_MS);
            return status;
        }

        configured_ = true;
        latest_.configured = true;
        failure_count_ = 0u;
        state_ = DriverState::FIFO_READ;
        last_status_ = Status::NoData;
        ScheduleDelayed(now, FIFO_WATCHDOG_INTERVAL_MS);
        return Status::NoData;
    }

    case DriverState::FIFO_READ: {
        uint32_t timestamp_sample_ms{};

        // INT1 only records a timestamp/pending flag. SPI FIFO access remains
        // in normal context and is skipped when no data-ready event is pending.
        if (!ConsumeDataReady(timestamp_sample_ms)) {
            ScheduleDelayed(now, FIFO_WATCHDOG_INTERVAL_MS);
            return Status::NoData;
        }

        ScheduleDelayed(now, FIFO_WATCHDOG_INTERVAL_MS);
        const Status status = FIFORead(timestamp_sample_ms);

        if (status == Status::Ok) {
            if (failure_count_ > 0u) {
                --failure_count_;
            }

        } else {
            ++failure_count_;

            if (failure_count_ > FAILURE_RESET_THRESHOLD) {
                (void)Reset();
            }
        }

        // TODO: add PX4-style periodic configuration checking without moving
        // register recovery policy into the Service or scheduler layers.
        last_status_ = status;
        return status;
    }
    }

    return Status::InvalidArgument;
}

ICM42688P::Status ICM42688P::FIFORead(const uint32_t timestamp_sample_ms)
{
    uint16_t fifo_count_bytes{};
    Status status = FIFOReadCount(fifo_count_bytes);

    if (status == Status::NoData) {
        latest_.configured = configured_;
        latest_.error_counter = error_count_;
        latest_.interrupt_counter = data_ready_interrupt_count_;
        latest_.last_interrupt_timestamp_ms = timestamp_sample_ms;
        last_status_ = status;
        return status;
    }

    if (status != Status::Ok) {
        ++error_count_;
        latest_.error_counter = error_count_;
        latest_.data_valid = false;
        last_status_ = status;
        return status;
    }

    const uint16_t requested_packets = static_cast<uint16_t>(fifo_count_bytes / FIFO_PACKET_SIZE);
    uint16_t valid_packets{};
    status = FIFOReadData(requested_packets, valid_packets);

    if (status == Status::NoData) {
        latest_.configured = configured_;
        latest_.error_counter = error_count_;
        latest_.interrupt_counter = data_ready_interrupt_count_;
        latest_.last_interrupt_timestamp_ms = timestamp_sample_ms;
        last_status_ = status;
        return status;
    }

    if (status != Status::Ok) {
        ++error_count_;
        latest_.error_counter = error_count_;
        latest_.data_valid = false;
        last_status_ = status;
        return status;
    }

    if (valid_packets == 0u || valid_packets != fifo_decoded_count_) {
        ++error_count_;
        latest_.error_counter = error_count_;
        latest_.data_valid = false;
        last_status_ = Status::BadFifoPacket;
        return Status::BadFifoPacket;
    }

    Sample sample{};
    sample.delta_samples = valid_packets;
    sample.delta_time_s = static_cast<float>(valid_packets) * FIFO_SAMPLE_DT_S;

    status = ProcessTemperature(fifo_decoded_batch_, valid_packets, sample);

    if (status == Status::Ok) {
        status = ProcessGyro(
            fifo_decoded_batch_, valid_packets, FIFO_SAMPLE_DT_S, sample);
    }

    if (status == Status::Ok) {
        status = ProcessAccel(
            fifo_decoded_batch_, valid_packets, FIFO_SAMPLE_DT_S, sample);
    }

    if (status != Status::Ok) {
        ++error_count_;
        latest_.error_counter = error_count_;
        latest_.data_valid = false;
        last_status_ = status;
        return status;
    }

    sample.timestamp_ms = timestamp_sample_ms;
    sample.sample_counter = sample_count_ + valid_packets;
    sample.error_counter = error_count_;
    sample.configured = configured_;
    sample.data_valid = true;
    sample.fifo_count_bytes = last_fifo_count_bytes_;
    sample.fifo_valid_packets = valid_packets;
    sample.fifo_timestamp = fifo_last_decoded_.timestamp_fifo;
    sample.fifo_header = fifo_last_decoded_.header;
    sample.data_source = DATA_SOURCE_FIFO;
    sample.interrupt_counter = data_ready_interrupt_count_;
    sample.last_interrupt_timestamp_ms = timestamp_sample_ms;

    sample_count_ = sample.sample_counter;
    latest_ = sample;
    last_status_ = Status::Ok;
    return Status::Ok;
}

ICM42688P::Status ICM42688P::FIFOReadCount(uint16_t& count_bytes)
{
    uint8_t count_buffer[2]{};
    const Status status = ReadBuffer(BANK0::FIFO_COUNTH, count_buffer, sizeof(count_buffer));

    if (status != Status::Ok) {
        return status;
    }

    count_bytes = static_cast<uint16_t>(
        (static_cast<uint16_t>(count_buffer[0]) << 8u) | count_buffer[1]);
    last_fifo_count_bytes_ = count_bytes;

    if (count_bytes >= ICM42688P_Regs::FIFO::SIZE) {
        const Status reset_status = FIFOReset();
        return reset_status == Status::Ok ? Status::FifoOverflow : reset_status;
    }

    return count_bytes < FIFO_PACKET_SIZE ? Status::NoData : Status::Ok;
}

ICM42688P::Status ICM42688P::FIFOReadData(const uint16_t requested_packets,
                                          uint16_t& valid_packets)
{
    valid_packets = 0u;
    fifo_decoded_count_ = 0u;

    if (requested_packets == 0u) {
        return Status::NoData;
    }

    if (requested_packets > FIFO_MAX_PACKETS_PER_UPDATE) {
        const Status reset_status = FIFOReset();
        return reset_status == Status::Ok ? Status::FifoOverflow : reset_status;
    }

    const uint16_t data_bytes = static_cast<uint16_t>(requested_packets * FIFO_PACKET_SIZE);
    const uint16_t transfer_length = static_cast<uint16_t>(
        ICM42688P_Regs::SPI_COMMAND_LENGTH + FIFO_TRANSFER_PREFIX_BYTES + data_bytes);
    const Status bank_status = SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_0);

    if (bank_status != Status::Ok) {
        return bank_status;
    }

    fifo_tx_buf_[0] = static_cast<uint8_t>(BANK0::INT_STATUS)
                    | ICM42688P_Regs::SPI_READ_BIT;
    memset(&fifo_tx_buf_[ICM42688P_Regs::SPI_COMMAND_LENGTH],
           ICM42688P_Regs::SPI_DUMMY_BYTE,
           static_cast<size_t>(transfer_length - ICM42688P_Regs::SPI_COMMAND_LENGTH));

    CS_Low();
    const HAL_StatusTypeDef hal_status = HAL_SPI_TransmitReceive(
        hspi_,
        fifo_tx_buf_,
        fifo_rx_buf_,
        transfer_length,
        ICM42688P_Regs::SPI_TRANSACTION_TIMEOUT_MS);
    CS_High();

    if (hal_status != HAL_OK) {
        return Status::SpiError;
    }

    const uint8_t int_status = fifo_rx_buf_[FIFO_RX_INT_STATUS_INDEX];
    const uint16_t fifo_count_bytes = static_cast<uint16_t>(
        (static_cast<uint16_t>(fifo_rx_buf_[FIFO_RX_COUNT_HIGH_INDEX]) << 8u)
        | fifo_rx_buf_[FIFO_RX_COUNT_LOW_INDEX]);
    last_fifo_count_bytes_ = fifo_count_bytes;

    if ((int_status & static_cast<uint8_t>(ICM42688P_Regs::INT_STATUS_BITS::FIFO_FULL_INT)) != 0u
        || fifo_count_bytes >= ICM42688P_Regs::FIFO::SIZE) {
        const Status reset_status = FIFOReset();
        return reset_status == Status::Ok ? Status::FifoOverflow : reset_status;
    }

    const uint16_t available_packets = static_cast<uint16_t>(fifo_count_bytes / FIFO_PACKET_SIZE);

    if (available_packets == 0u) {
        return Status::NoData;
    }

    if (available_packets > FIFO_MAX_PACKETS_PER_UPDATE) {
        const Status reset_status = FIFOReset();
        return reset_status == Status::Ok ? Status::FifoOverflow : reset_status;
    }

    const uint16_t packets_to_process = requested_packets < available_packets
        ? requested_packets
        : available_packets;

    for (uint16_t index = 0u; index < packets_to_process; ++index) {
        ICM42688P_Regs::FIFO::DATA packet{};
        const size_t packet_offset = static_cast<size_t>(FIFO_RX_DATA_INDEX)
                                   + static_cast<size_t>(index * FIFO_PACKET_SIZE);
        memcpy(&packet, &fifo_rx_buf_[packet_offset], sizeof(packet));

        if (!IsValidFifoHeader(packet.FIFO_Header)) {
            const Status reset_status = FIFOReset();
            return reset_status == Status::Ok ? Status::BadFifoPacket : reset_status;
        }

        FifoDecodedSample decoded{};
        const Status decode_status = DecodeFifoPacket(packet, decoded);

        if (decode_status != Status::Ok) {
            const Status reset_status = FIFOReset();
            return reset_status == Status::Ok ? decode_status : reset_status;
        }

        fifo_decoded_batch_[valid_packets] = decoded;
        fifo_last_decoded_ = decoded;
        ++valid_packets;
    }

    fifo_decoded_count_ = valid_packets;
    last_fifo_valid_packets_ = valid_packets;
    return valid_packets == 0u ? Status::NoData : Status::Ok;
}

ICM42688P::Status ICM42688P::DecodeFifoPacket(
    const ICM42688P_Regs::FIFO::DATA& packet,
    FifoDecodedSample& decoded) const
{
    if (!IsValidFifoHeader(packet.FIFO_Header)) {
        return Status::BadFifoPacket;
    }

    const uint8_t accel_extension[3]{
        static_cast<uint8_t>((packet.ACCEL_X_3_0_GYRO_X_3_0 & 0xF0u) >> 4u),
        static_cast<uint8_t>((packet.ACCEL_Y_3_0_GYRO_Y_3_0 & 0xF0u) >> 4u),
        static_cast<uint8_t>((packet.ACCEL_Z_3_0_GYRO_Z_3_0 & 0xF0u) >> 4u),
    };
    const uint8_t gyro_extension[3]{
        static_cast<uint8_t>(packet.ACCEL_X_3_0_GYRO_X_3_0 & 0x0Fu),
        static_cast<uint8_t>(packet.ACCEL_Y_3_0_GYRO_Y_3_0 & 0x0Fu),
        static_cast<uint8_t>(packet.ACCEL_Z_3_0_GYRO_Z_3_0 & 0x0Fu),
    };
    const uint8_t accel_high[3]{
        packet.ACCEL_X_19_12,
        packet.ACCEL_Y_19_12,
        packet.ACCEL_Z_19_12,
    };
    const uint8_t accel_low[3]{
        packet.ACCEL_X_11_4,
        packet.ACCEL_Y_11_4,
        packet.ACCEL_Z_11_4,
    };
    const uint8_t gyro_high[3]{
        packet.GYRO_X_19_12,
        packet.GYRO_Y_19_12,
        packet.GYRO_Z_19_12,
    };
    const uint8_t gyro_low[3]{
        packet.GYRO_X_11_4,
        packet.GYRO_Y_11_4,
        packet.GYRO_Z_11_4,
    };

    constexpr float accel_scale = STANDARD_GRAVITY_M_S2 / 8192.0F;
    constexpr float gyro_scale = (1.0F / 131.0F) * DEG_TO_RAD;

    for (uint8_t axis = 0u; axis < 3u; ++axis) {
        decoded.accel_raw20[axis] = Reassemble20Bit(
            accel_high[axis], accel_low[axis], accel_extension[axis]);
        decoded.gyro_raw20[axis] = Reassemble20Bit(
            gyro_high[axis], gyro_low[axis], gyro_extension[axis]);
        decoded.accel_effective[axis] = decoded.accel_raw20[axis] / 4;
        decoded.gyro_effective[axis] = decoded.gyro_raw20[axis] / 2;
        decoded.accel_raw16[axis] = SaturateToInt16(decoded.accel_effective[axis]);
        decoded.gyro_raw16[axis] = SaturateToInt16(decoded.gyro_effective[axis]);
        decoded.accel_m_s2[axis] = static_cast<float>(decoded.accel_effective[axis])
                                  * accel_scale;
        decoded.gyro_rad_s[axis] = static_cast<float>(decoded.gyro_effective[axis])
                                  * gyro_scale;
    }

    decoded.temp_raw = CombineBigEndian(packet.TEMPERATURE_15_8, packet.TEMPERATURE_7_0);
    decoded.temperature_deg_c = static_cast<float>(decoded.temp_raw)
                                / TEMPERATURE_SENSITIVITY_LSB_PER_DEG_C
                                + TEMPERATURE_OFFSET_DEG_C;
    decoded.timestamp_fifo = static_cast<uint16_t>(
        (static_cast<uint16_t>(packet.TIME_STAMP_15_8) << 8u)
        | packet.TIME_STAMP_7_0);
    decoded.header = packet.FIFO_Header;
    return Status::Ok;
}

ICM42688P::Status ICM42688P::ProcessTemperature(
    const FifoDecodedSample samples[],
    const uint16_t sample_count,
    Sample& output) const
{
    if (samples == nullptr || sample_count == 0u
        || sample_count > FIFO_MAX_PACKETS_PER_UPDATE) {
        return Status::InvalidArgument;
    }

    int32_t temperature_sum{0};

    for (uint16_t index = 0u; index < sample_count; ++index) {
        temperature_sum += samples[index].temp_raw;
    }

    const int32_t temperature_average = temperature_sum / static_cast<int32_t>(sample_count);
    output.temp_raw = SaturateToInt16(temperature_average);
    output.temperature_deg_c = static_cast<float>(output.temp_raw)
                               / TEMPERATURE_SENSITIVITY_LSB_PER_DEG_C
                               + TEMPERATURE_OFFSET_DEG_C;
    return Status::Ok;
}

ICM42688P::Status ICM42688P::ProcessGyro(
    const FifoDecodedSample samples[],
    const uint16_t sample_count,
    const float sample_dt_s,
    Sample& output)
{
    if (samples == nullptr || sample_count == 0u
        || sample_count > FIFO_MAX_PACKETS_PER_UPDATE
        || sample_dt_s <= 0.0F) {
        return Status::InvalidArgument;
    }

    constexpr float gyro_scale = (1.0F / 131.0F) * DEG_TO_RAD;
    output.delta_time_s = static_cast<float>(sample_count) * sample_dt_s;

    for (uint8_t axis = 0u; axis < 3u; ++axis) {
        float integral_raw{0.0F};

        if (gyro_last_effective_valid_) {
            integral_raw = 0.5F * static_cast<float>(gyro_last_effective_[axis])
                         + 0.5F * static_cast<float>(
                               samples[sample_count - 1u].gyro_effective[axis]);

            for (uint16_t index = 0u; index + 1u < sample_count; ++index) {
                integral_raw += static_cast<float>(samples[index].gyro_effective[axis]);
            }

        } else {
            for (uint16_t index = 0u; index < sample_count; ++index) {
                integral_raw += static_cast<float>(samples[index].gyro_effective[axis]);
            }
        }

        output.delta_angle_rad[axis] = integral_raw * gyro_scale * sample_dt_s;
        output.gyro_rad_s[axis] = output.delta_angle_rad[axis] / output.delta_time_s;
        output.gyro_effective[axis] = static_cast<int32_t>(
            integral_raw / static_cast<float>(sample_count));
        output.gyro_raw[axis] = SaturateToInt16(output.gyro_effective[axis]);
        output.gyro_raw20[axis] = samples[sample_count - 1u].gyro_raw20[axis];
        gyro_last_effective_[axis] = samples[sample_count - 1u].gyro_effective[axis];
    }

    gyro_last_effective_valid_ = true;
    return Status::Ok;
}

ICM42688P::Status ICM42688P::ProcessAccel(
    const FifoDecodedSample samples[],
    const uint16_t sample_count,
    const float sample_dt_s,
    Sample& output)
{
    if (samples == nullptr || sample_count == 0u
        || sample_count > FIFO_MAX_PACKETS_PER_UPDATE
        || sample_dt_s <= 0.0F) {
        return Status::InvalidArgument;
    }

    constexpr float accel_scale = STANDARD_GRAVITY_M_S2 / 8192.0F;
    output.delta_time_s = static_cast<float>(sample_count) * sample_dt_s;

    for (uint8_t axis = 0u; axis < 3u; ++axis) {
        float integral_raw{0.0F};

        if (accel_last_effective_valid_) {
            integral_raw = 0.5F * static_cast<float>(accel_last_effective_[axis])
                         + 0.5F * static_cast<float>(
                               samples[sample_count - 1u].accel_effective[axis]);

            for (uint16_t index = 0u; index + 1u < sample_count; ++index) {
                integral_raw += static_cast<float>(samples[index].accel_effective[axis]);
            }

        } else {
            for (uint16_t index = 0u; index < sample_count; ++index) {
                integral_raw += static_cast<float>(samples[index].accel_effective[axis]);
            }
        }

        output.delta_velocity_m_s[axis] = integral_raw * accel_scale * sample_dt_s;
        output.accel_m_s2[axis] = output.delta_velocity_m_s[axis] / output.delta_time_s;
        output.accel_effective[axis] = static_cast<int32_t>(
            integral_raw / static_cast<float>(sample_count));
        output.accel_raw[axis] = SaturateToInt16(output.accel_effective[axis]);
        output.accel_raw20[axis] = samples[sample_count - 1u].accel_raw20[axis];
        accel_last_effective_[axis] = samples[sample_count - 1u].accel_effective[axis];
    }

    accel_last_effective_valid_ = true;
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

// Bank-aware register and buffer access helpers.
// The C API also exposes selected accessors for debug/bring-up use.
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

void ICM42688P::DataReady(const uint32_t timestamp_ms)
{
    // EXTI context only records timing and pending state. SPI access is deferred
    // to RunImpl() in normal scheduler context, matching PX4 DataReady().
    last_data_ready_timestamp_ms_ = timestamp_ms;
    ++data_ready_interrupt_count_;
    data_ready_pending_ = 1u;
}

bool ICM42688P::ConsumeDataReady(uint32_t& timestamp_sample_ms)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool pending = data_ready_pending_ != 0u;

    if (pending) {
        timestamp_sample_ms = last_data_ready_timestamp_ms_;
        data_ready_pending_ = 0u;
    }

    if (primask == 0u) {
        __enable_irq();
    }

    return pending;
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
