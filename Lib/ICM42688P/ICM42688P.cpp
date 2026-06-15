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
#include "TimeBase.h"
#include <cstring>

using ICM42688P_Regs::RegsAdd::BANK0;
using ICM42688P_Regs::RegsAdd::BANK1;
using ICM42688P_Regs::RegsAdd::BANK2;

namespace
{
// ============================================================================
// 物理量换算、状态机节拍和实测 batch dt 合理性范围
// ============================================================================

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
constexpr float MICROSECONDS_TO_SECONDS{1.0e-6F};
constexpr float MEASURED_BATCH_DT_MIN_RATIO{0.5F};
constexpr float MEASURED_BATCH_DT_MAX_RATIO{2.0F};

/**
 * @brief  根据名义 batch 时间和相邻 data-ready 时间戳选择本批次积分时间
 * @param  nominal_batch_dt_s 名义 batch 时间，单位 s
 * @param  timestamp_sample_us 当前成功 FIFO batch 对应的 data-ready 时间戳，单位 us
 * @param  last_timestamp_valid 上一批成功时间戳是否有效
 * @param  last_timestamp_us 上一批成功 FIFO batch 对应的 data-ready 时间戳，单位 us
 * @retval 本批次采用的 batch delta time，单位 s
 */
float ComputeBatchDeltaTimeSeconds(const float nominal_batch_dt_s,
                                   const uint64_t timestamp_sample_us,
                                   const bool last_timestamp_valid,
                                   const uint64_t last_timestamp_us)
{
    // 首个成功 batch 无历史时间戳，直接使用名义 dt。
    if (!last_timestamp_valid) {
        return nominal_batch_dt_s;
    }

    // 相邻成功 data-ready 的 MCU 时间差作为实测 batch 时间。
    const uint64_t measured_batch_dt_us =
        timestamp_sample_us - last_timestamp_us;
    const float measured_batch_dt_s =
        static_cast<float>(measured_batch_dt_us) * MICROSECONDS_TO_SECONDS;

    // 实测值必须在名义值的 0.5~2.0 倍范围内，否则回退名义 dt 以避免异常
    // 中断间隔污染积分。FIFO 内部 timestamp 不参与这里的主时间计算。
    if (measured_batch_dt_s >= nominal_batch_dt_s * MEASURED_BATCH_DT_MIN_RATIO
        && measured_batch_dt_s <= nominal_batch_dt_s * MEASURED_BATCH_DT_MAX_RATIO) {
        return measured_batch_dt_s;
    }

    return nominal_batch_dt_s;
}
}

// ============================================================================
// 对象构造与生命周期入口
// ============================================================================

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
    last_fifo_timestamp_sample_us_ = 0u;
    last_fifo_timestamp_sample_valid_ = false;
    data_ready_pending_ = 0u;
    data_ready_interrupt_count_ = 0u;
    last_data_ready_timestamp_us_ = 0u;
    bank_selected_valid_ = false;
    failure_count_ = 0u;
    reset_timestamp_ms_ = 0u;
    initialized_ = true;
    configured_ = false;
    state_ = DriverState::RESET;
    last_status_ = Status::Ok;
    latest_.error_counter = error_count_;
    ScheduleNow(TimeBase_Millis());
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
    last_fifo_timestamp_sample_us_ = 0u;
    last_fifo_timestamp_sample_valid_ = false;
    initialized_ = true;
    state_ = DriverState::RESET;
    ScheduleNow(TimeBase_Millis());
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
    // EXTI 边沿触发由 CubeMX/HAL 配置；这里仅校验传感器侧 INT 寄存器配置，
    // 并清空软件侧 data-ready 事件锁存状态。
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
    last_data_ready_timestamp_us_ = 0u;
    __set_PRIMASK(primask);

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
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    data_ready_pending_ = 0u;
    __set_PRIMASK(primask);

    last_fifo_count_bytes_ = 0u;
    last_fifo_valid_packets_ = 0u;
    fifo_decoded_count_ = 0u;
    accel_last_effective_valid_ = false;
    gyro_last_effective_valid_ = false;
    last_fifo_timestamp_sample_us_ = 0u;
    last_fifo_timestamp_sample_valid_ = false;

    return Status::Ok;
}

ICM42688P::Status ICM42688P::Update()
{
    return RunImpl();
}

// 裸机版 PX4 风格驱动生命周期中心。Service 的高优先级事件路径和 1 kHz
// 周期兜底路径都只负责提供执行机会，具体状态转换统一在这里完成。
ICM42688P::Status ICM42688P::RunImpl()
{
    if (!initialized_) {
        return Status::InvalidArgument;
    }

    const uint32_t now = TimeBase_Millis();
    const bool data_ready_scheduled =
        state_ == DriverState::FIFO_READ && data_ready_pending_ != 0u;

    if (!data_ready_scheduled && !TickReached(now, next_run_ms_)) {
        return Status::NoData;
    }

    // next_run_ms_ 只表达“下一次允许运行的时间”，并不主动唤醒代码；实际调用
    // 仍来自 Service。data-ready pending 可使 FIFO_READ 跳过该毫秒延时门限。
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
        uint64_t timestamp_sample_us{};

        // 中断只锁存事件和时间戳。SPI 访问及 FIFO 处理始终留在普通上下文的
        // FIFO_READ 状态中执行。
        if (!ConsumeDataReady(timestamp_sample_us)) {
            ScheduleDelayed(now, FIFO_WATCHDOG_INTERVAL_MS);
            return Status::NoData;
        }

        ScheduleDelayed(now, FIFO_WATCHDOG_INTERVAL_MS);
        const Status status = FIFORead(timestamp_sample_us);

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

        // TODO: 后续可增加类似 PX4 的周期性配置检查，但寄存器恢复策略仍应留在
        // driver 层，不应移动到 Service 或 Scheduler。
        last_status_ = status;
        return status;
    }
    }

    return Status::InvalidArgument;
}

ICM42688P::Status ICM42688P::FIFORead(const uint64_t timestamp_sample_us)
{
    // 一次 FIFORead 读取并解码完整 batch，处理温度、角增量和速度增量，最后
    // 更新 latest_。任何失败路径都不会推进成功 batch 的 dt 基准时间戳。
    const uint32_t timestamp_sample_ms =
        static_cast<uint32_t>(timestamp_sample_us / 1000u);
    uint16_t fifo_count_bytes{};
    Status status = FIFOReadCount(fifo_count_bytes);

    if (status == Status::NoData) {
        latest_.configured = configured_;
        latest_.error_counter = error_count_;
        latest_.interrupt_counter = data_ready_interrupt_count_;
        latest_.last_interrupt_timestamp_us = timestamp_sample_us;
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
        latest_.last_interrupt_timestamp_us = timestamp_sample_us;
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

    // 本段只选择 batch 积分时间：优先使用相邻成功 data-ready 的 MCU 时间差；
    // 如果实测值超出名义值 0.5~2.0 倍，则回退名义 dt。FIFO 内部 timestamp
    // 不参与主积分时间计算。
    const float nominal_batch_dt_s = static_cast<float>(valid_packets) * FIFO_SAMPLE_DT_S;
    const float batch_dt_s = ComputeBatchDeltaTimeSeconds(
        nominal_batch_dt_s,
        timestamp_sample_us,
        last_fifo_timestamp_sample_valid_,
        last_fifo_timestamp_sample_us_);
    const float sample_dt_s = batch_dt_s / static_cast<float>(valid_packets);

    status = ProcessTemperature(fifo_decoded_batch_, valid_packets, sample);

    if (status == Status::Ok) {
        status = ProcessGyro(
            fifo_decoded_batch_, valid_packets, sample_dt_s, sample);
    }

    if (status == Status::Ok) {
        status = ProcessAccel(
            fifo_decoded_batch_, valid_packets, sample_dt_s, sample);
    }

    if (status != Status::Ok) {
        ++error_count_;
        latest_.error_counter = error_count_;
        latest_.data_valid = false;
        last_status_ = status;
        return status;
    }

    sample.timestamp_us = timestamp_sample_us;
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
    sample.last_interrupt_timestamp_us = timestamp_sample_us;
    sample.last_interrupt_timestamp_ms = timestamp_sample_ms;

    sample_count_ = sample.sample_counter;
    last_fifo_timestamp_sample_us_ = timestamp_sample_us;
    last_fifo_timestamp_sample_valid_ = true;
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
        // 将芯片默认坐标统一转换到目标机体系：X、Z 取反，Y 保持不变。
        const int32_t axis_sign = AxisSign(axis);
        decoded.accel_raw20[axis] = axis_sign * Reassemble20Bit(
            accel_high[axis], accel_low[axis], accel_extension[axis]);
        decoded.gyro_raw20[axis] = axis_sign * Reassemble20Bit(
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
    // 对整个 FIFO batch 做梯形积分，主输出为 delta_angle_rad；平均角速度由
    // delta_angle / delta_time 派生，不改变原始积分公式。
    //
    // 该积分结构与 PX4Gyroscope::updateFIFO() 的 FIFO batch 处理思路一致：
    // 使用上一批末尾 sample 和当前批末尾 sample 构成跨 batch 梯形积分边界，
    // 中间 sample 完整累加，最终得到本批次 delta_angle。
    // 当前裸机工程没有 PX4Gyroscope/uORB，因此直接把积分结果写入 Sample。
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
    // 对整个 FIFO batch 做梯形积分，主输出为机体系比力积分
    // delta_velocity_m_s；不包含导航系重力补偿或坐标系二次变换。
    //
    // 该积分结构与 PX4Accelerometer::updateFIFO() 的 FIFO batch 处理思路一致：
    // 使用上一批末尾 sample 和当前批末尾 sample 构成跨 batch 梯形积分边界，
    // 中间 sample 完整累加，最终得到本批次 delta_velocity。
    // 当前裸机工程没有 PX4Accelerometer/uORB，因此直接把积分结果写入 Sample。
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

// 带 bank 选择的寄存器和缓冲区访问辅助函数。
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

ICM42688P::Status ICM42688P::GetLatest(Sample& sample) const
{
    if (!initialized_) {
        return Status::InvalidArgument;
    }

    sample = latest_;
    return Status::Ok;
}

void ICM42688P::DataReady(const uint64_t timestamp_us)
{
    // ISR 路径只保存最后一次 MCU 时间戳、置 pending 并累计中断次数。
    // 实际 SPI/FIFO 读取由 RunImpl() 的 FIFO_READ 状态完成。
    last_data_ready_timestamp_us_ = timestamp_us;
    ++data_ready_interrupt_count_;
    data_ready_pending_ = 1u;
}

bool ICM42688P::ConsumeDataReady(uint64_t& timestamp_sample_us)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const bool pending = data_ready_pending_ != 0u;

    if (pending) {
        timestamp_sample_us = last_data_ready_timestamp_us_;
        data_ready_pending_ = 0u;
    }

    __set_PRIMASK(primask);
    return pending;
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
