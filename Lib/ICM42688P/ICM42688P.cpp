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

/**
 * @brief  初始化 driver 软件状态，进入 RESET 等待硬件初始化
 * @retval Status::Ok              driver 已准备好由 RunImpl() 推进状态机
 * @retval Status::InvalidArgument  SPI 总线无效
 */
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

/**
 * @brief  读取 WHO_AM_I 寄存器验证芯片型号
 * @retval Status::Ok              芯片型号匹配 ICM42688P
 * @retval 其他状态表示 SPI、寄存器或设备 ID 错误
 */
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

/**
 * @brief  将 driver 状态机重新置于 RESET，真正的 SPI soft reset 由 RunImpl() 执行
 * @retval Status::Ok              状态机已复位，等待下次 RunImpl() 调度
 * @retval Status::InvalidArgument  SPI 总线无效
 */
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

/**
 * @brief  轮询 WHO_AM_I、DEVICE_CONFIG 和 INT_STATUS，确认 soft reset 完成
 * @retval Status::Ok reset 完成
 * @retval Status::NoData 仍需等待
 * @retval 其他状态表示 SPI 访问失败或 WHO_AM_I 不匹配
 */
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

    // 1. 先配置 Bank1 中的陀螺仪 AAF / notch 等高级滤波参数。
    for (const auto& config : register_bank1_cfg_) {
        const Status status = RegisterSetAndClearBits(config);

        if (status != Status::Ok) {
            return status;
        }
    }

    // 2. 再配置 Bank2 中的加速度计 AAF 参数。
    for (const auto& config : register_bank2_cfg_) {
        const Status status = RegisterSetAndClearBits(config);

        if (status != Status::Ok) {
            return status;
        }
    }

    // 3. 配置 Bank0 中除 FIFO_CONFIG1 和 PWR_MGMT0 以外的控制寄存器。
    //    FIFO_CONFIG1 需要在 watermark 配置完成后写入，PWR_MGMT0 需要最后写入以启动传感器。
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

        const Status status = RegisterSetAndClearBits(config);

        if (status != Status::Ok) {
            return status;
        }
    }

    if (fifo_config1 == nullptr || power_config == nullptr) {
        return Status::ConfigMismatch;
    }

    // 4. 单独配置 FIFO watermark。当前 watermark 已通过实板验证。
    Status status = ConfigureFIFOWatermark(FIFO_WATERMARK_BYTES);

    if (status != Status::Ok) {
        return status;
    }

    // 5. 启用 FIFO 数据包类型。在 watermark 之后写入，避免 FIFO 先启动而 watermark 尚未就绪。
    status = RegisterSetAndClearBits(*fifo_config1);

    if (status != Status::Ok) {
        return status;
    }

    // 6. 最后写入 PWR_MGMT0，使能 gyro/accel 工作模式；这是传感器真正启动的边界。
    status = RegisterSetAndClearBits(*power_config);

    if (status != Status::Ok) {
        return status;
    }

    // 7. 等待传感器完成模式切换和启动。
    HAL_Delay(ICM42688P_Regs::SENSOR_MODE_CHANGE_WAIT_MS);
    HAL_Delay(ICM42688P_Regs::SENSOR_STARTUP_WAIT_MS);

    // 8. 按配置表回读校验寄存器，与 PX4 Configure() 的配置后检查思路一致。
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
        status = RegisterCheck(config);

        if (status != Status::Ok) {
            return status;
        }
    }

    return Status::Ok;
}

/**
 * @brief  配置 FIFO watermark 阈值并回读校验
 * @param  watermark_bytes FIFO watermark 字节数
 * @retval Status::Ok  watermark 配置成功
 * @retval 其他状态表示参数无效或 SPI/配置校验失败
 */
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

/**
 * @brief  发送 FIFO FLUSH 指令并清空软件侧 FIFO 相关缓存和积分状态
 * @retval Status::Ok  FIFO 已刷新
 * @retval 其他状态表示 SPI 写入失败
 */
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

/**
 * @brief  执行 ICM42688P driver 内部状态机（PX4 风格 5 态）
 * @retval Status::Ok          成功读取并处理一次 FIFO batch
 * @retval Status::NoData      当前状态无需继续处理或暂无 data-ready 事件
 * @retval Status::InvalidArgument driver 尚未初始化
 * @retval 其他状态表示 reset、配置、FIFO 或 SPI 访问阶段失败
 *
 * @details
 * Service 层只提供高优先级事件路径和 1 kHz 周期兜底执行机会；
 * 具体 reset、configure、FIFO reset 和 FIFO read 状态转换均在本函数内完成。
 * next_run_ms_ 只表达“下一次允许运行的时间”，并不主动唤醒代码；
 * data-ready pending 可使 FIFO_READ 跳过该毫秒延时门限。
 */
ICM42688P::Status ICM42688P::RunImpl()
{
    // 1. 检查 driver 是否已初始化。
    if (!initialized_) {
        return Status::InvalidArgument;
    }

    // 2. 判断当前是否由 data-ready 事件触发，或是否到达调度时间。
    const uint32_t now = TimeBase_Millis();
    const bool data_ready_scheduled =
        state_ == DriverState::FIFO_READ && data_ready_pending_ != 0u;

    if (!data_ready_scheduled && !TickReached(now, next_run_ms_)) {
        return Status::NoData;
    }

    switch (state_) {

    // ========================================================================
    // RESET：清空运行状态并发送 soft reset 命令。
    // ========================================================================
    case DriverState::RESET: {
        configured_ = false;
        latest_.configured = false;
        latest_.data_valid = false;
        failure_count_ = 0u;

        const Status status = RegisterWrite(
            BANK0::DEVICE_CONFIG,
            static_cast<uint8_t>(ICM42688P_Regs::DEVICE_CONFIG_BITS::SOFT_RESET_CONFIG));

        // reset 命令发送失败时，延时后重试。
        if (status != Status::Ok) {
            ++error_count_;
            ++failure_count_;
            last_status_ = status;
            ScheduleDelayed(now, RESET_RETRY_DELAY_MS);
            return status;
        }

        // reset 命令已发出，转入 WAIT_FOR_RESET 等待复位完成。
        bank_selected_valid_ = false;
        reset_timestamp_ms_ = now;
        state_ = DriverState::WAIT_FOR_RESET;
        last_status_ = Status::NoData;
        ScheduleDelayed(now, ICM42688P_Regs::SOFT_RESET_WAIT_MS);
        return Status::NoData;
    }

    // ========================================================================
    // WAIT_FOR_RESET：轮询 reset done 和 WHO_AM_I 直到复位完成或超时。
    // ========================================================================
    case DriverState::WAIT_FOR_RESET: {
        const Status status = WaitForReset();

        // reset 完成后进入配置阶段。
        if (status == Status::Ok) {
            state_ = DriverState::CONFIGURE;
            last_status_ = Status::NoData;
            ScheduleNow(now);
            return Status::NoData;
        }

        // reset 超时或 WHO_AM_I 错误时回到 RESET。
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

        // 未完成复位，安排下一次检查。
        last_status_ = status;
        ScheduleDelayed(now, RESET_CHECK_INTERVAL_MS);
        return Status::NoData;
    }

    // ========================================================================
    // CONFIGURE：写入传感器配置并校验中断寄存器。
    // ========================================================================
    case DriverState::CONFIGURE: {
        Status status = Configure();

        if (status == Status::Ok) {
            status = ConfigureInterrupt();
        }

        // 配置成功后进入 FIFO_RESET，准备清空 FIFO。
        if (status == Status::Ok) {
            state_ = DriverState::FIFO_RESET;
            last_status_ = Status::NoData;
            ScheduleDelayed(now, ICM42688P_Regs::SOFT_RESET_WAIT_MS);
            return Status::NoData;
        }

        // 配置失败时按失败次数或超时策略决定是否重新 reset。
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

    // ========================================================================
    // FIFO_RESET：刷新 FIFO 并清空跨 batch 积分状态。
    // ========================================================================
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

        // FIFO reset 成功后进入正常 FIFO_READ 状态。
        configured_ = true;
        latest_.configured = true;
        failure_count_ = 0u;
        state_ = DriverState::FIFO_READ;
        last_status_ = Status::NoData;
        ScheduleDelayed(now, FIFO_WATCHDOG_INTERVAL_MS);
        return Status::NoData;
    }

    // ========================================================================
    // FIFO_READ：等待 data-ready 事件并处理一次 FIFO batch。
    // ========================================================================
    case DriverState::FIFO_READ: {
        uint64_t timestamp_sample_us{};

        // 中断只锁存事件和时间戳。SPI 访问及 FIFO 处理始终留在普通上下文的
        // FIFO_READ 状态中执行。
        // 没有 data-ready 事件时，只安排 watchdog 兜底调度。
        if (!ConsumeDataReady(timestamp_sample_us)) {
            ScheduleDelayed(now, FIFO_WATCHDOG_INTERVAL_MS);
            return Status::NoData;
        }

        ScheduleDelayed(now, FIFO_WATCHDOG_INTERVAL_MS);
        const Status status = FIFORead(timestamp_sample_us);

        // 读取成功时降低失败计数，失败过多时触发 Reset()。
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

    // 理论上不会到达；保留兜底错误返回。
    return Status::InvalidArgument;
}

/**
 * @brief  读取并处理一次完整 FIFO batch
 * @param  timestamp_sample_us 当前 FIFO batch 对应的 data-ready MCU 时间戳，单位 us
 * @retval Status::Ok          成功更新 latest_ 样本
 * @retval Status::NoData      FIFO 中暂无可处理数据
 * @retval Status::BadFifoPacket 有效 packet 数校验失败
 * @retval 其他状态表示 FIFO 读取、packet 解码或数据处理失败
 */
ICM42688P::Status ICM42688P::FIFORead(const uint64_t timestamp_sample_us)
{
    // 1. 将 data-ready 时间戳转换为毫秒时间戳，并读取 FIFO count。
    const uint32_t timestamp_sample_ms =
        static_cast<uint32_t>(timestamp_sample_us / 1000u);
    uint16_t fifo_count_bytes{};
    Status status = FIFOReadCount(fifo_count_bytes);

    // 2. 处理 FIFO count 阶段的无数据和错误返回。
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

    // 3. 根据 FIFO count 计算期望读取的 packet 数，并读取完整 FIFO batch。
    const uint16_t requested_packets = static_cast<uint16_t>(fifo_count_bytes / FIFO_PACKET_SIZE);
    uint16_t valid_packets{};
    status = FIFOReadData(requested_packets, valid_packets);

    // 4. 处理 FIFO 数据读取阶段的无数据和错误返回。
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

    // 5. 校验本次 batch 的有效 packet 数。
    if (valid_packets == 0u || valid_packets != fifo_decoded_count_) {
        ++error_count_;
        latest_.error_counter = error_count_;
        latest_.data_valid = false;
        last_status_ = Status::BadFifoPacket;
        return Status::BadFifoPacket;
    }

    // 6. 初始化输出 Sample，并计算本次 batch 的积分时间。
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

    // 7. 依次处理温度、陀螺仪和加速度计数据。
    status = ProcessTemperature(fifo_decoded_batch_, valid_packets, sample);

    if (status == Status::Ok) {
        status = ProcessGyro(
            fifo_decoded_batch_, valid_packets, sample_dt_s, sample);
    }

    if (status == Status::Ok) {
        status = ProcessAccel(
            fifo_decoded_batch_, valid_packets, sample_dt_s, sample);
    }

    // 8. 处理数据转换失败路径。
    if (status != Status::Ok) {
        ++error_count_;
        latest_.error_counter = error_count_;
        latest_.data_valid = false;
        last_status_ = status;
        return status;
    }

    // 任何失败路径都不会推进成功 batch 的 sample 计数和 dt 基准时间戳。
    // 9. 填充 Sample 元信息，并统一推进 driver 成功状态。
    PopulateFifoSampleMetadata(sample, timestamp_sample_us, timestamp_sample_ms, valid_packets);

    sample_count_ = sample.sample_counter;
    last_fifo_timestamp_sample_us_ = timestamp_sample_us;
    last_fifo_timestamp_sample_valid_ = true;
    latest_ = sample;
    last_status_ = Status::Ok;
    return Status::Ok;
}

/**
 * @brief  填充一次成功 FIFO batch 对应的 Sample 元信息
 * @param  sample 已完成温度、陀螺、加计处理的输出样本
 * @param  timestamp_sample_us 当前 FIFO batch 对应的 data-ready 时间戳，单位 us
 * @param  timestamp_sample_ms 当前 FIFO batch 对应的 data-ready 时间戳，单位 ms
 * @param  valid_packets 本次 FIFO batch 中有效 packet 数
 * @retval 无
 */
void ICM42688P::PopulateFifoSampleMetadata(Sample& sample,
                                           const uint64_t timestamp_sample_us,
                                           const uint32_t timestamp_sample_ms,
                                           const uint16_t valid_packets) const
{
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
}

/**
 * @brief  读取 FIFO 字节计数，溢出时自动执行 FIFOReset
 * @param  count_bytes FIFO 中当前字节数（输出参数）
 * @retval Status::Ok      FIFO 中至少有一个完整 packet
 * @retval Status::NoData  FIFO 数据不足一个 packet
 * @retval 其他状态表示 SPI 读取错误或 FIFO 溢出
 */
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

/**
 * @brief  读取 FIFO 数据并解码为当前 batch
 * @param  requested_packets 调用方期望读取的 FIFO packet 数
 * @param  valid_packets 成功解码的有效 FIFO packet 数（输出参数）
 * @retval Status::Ok          成功读取并至少解码一个 packet
 * @retval Status::NoData      FIFO 中无可处理 packet
 * @retval Status::SpiError    SPI 传输失败
 * @retval Status::FifoOverflow FIFO 溢出，已执行 FIFOReset
 * @retval Status::BadFifoPacket FIFO header 校验或解码失败，已执行 FIFOReset
 */
ICM42688P::Status ICM42688P::FIFOReadData(const uint16_t requested_packets,
                                          uint16_t& valid_packets)
{
    // 1. 清空本次 batch 的输出计数和解码计数。
    valid_packets = 0u;
    fifo_decoded_count_ = 0u;

    // 2. 检查请求 packet 数，过大时刷新 FIFO 并返回溢出状态。
    if (requested_packets == 0u) {
        return Status::NoData;
    }

    if (requested_packets > FIFO_MAX_PACKETS_PER_UPDATE) {
        const Status reset_status = FIFOReset();
        return reset_status == Status::Ok ? Status::FifoOverflow : reset_status;
    }

    // 3. 计算 SPI burst 传输长度，并确保当前处于 Bank0。
    const uint16_t data_bytes = static_cast<uint16_t>(requested_packets * FIFO_PACKET_SIZE);
    const uint16_t transfer_length = static_cast<uint16_t>(
        ICM42688P_Regs::SPI_COMMAND_LENGTH + FIFO_TRANSFER_PREFIX_BYTES + data_bytes);
    const Status bank_status = SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_0);

    if (bank_status != Status::Ok) {
        return bank_status;
    }

    // 4. 组装 FIFO burst read 传输缓冲区。
    fifo_tx_buf_[0] = static_cast<uint8_t>(BANK0::INT_STATUS)
                    | ICM42688P_Regs::SPI_READ_BIT;
    memset(&fifo_tx_buf_[ICM42688P_Regs::SPI_COMMAND_LENGTH],
           ICM42688P_Regs::SPI_DUMMY_BYTE,
           static_cast<size_t>(transfer_length - ICM42688P_Regs::SPI_COMMAND_LENGTH));

    // 5. 执行一次阻塞式 SPI burst 读，读取 INT_STATUS、FIFO count 和 FIFO data。
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

    // 6. 根据 INT_STATUS 和 FIFO count 检查 FIFO 溢出或空数据。
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

    // 7. 限制本次实际处理的 packet 数。
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

    // 8. 逐 packet 拷贝、校验 header、解码并写入 batch 缓存。
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

    // 9. 更新本次 batch 的解码统计。
    fifo_decoded_count_ = valid_packets;
    last_fifo_valid_packets_ = valid_packets;
    return valid_packets == 0u ? Status::NoData : Status::Ok;
}

/**
 * @brief  对单个 FIFO packet 做 20-bit 重组、坐标变换、标定和温度提取
 * @param  packet  FIFO RAW 20 字节 packet
 * @param  decoded 输出解码结果（20-bit raw、effective、16-bit、物理量、温度等）
 * @retval Status::Ok            解码成功
 * @retval Status::BadFifoPacket  FIFO header 校验失败
 *
 * @note   坐标变换：X、Z 取反，Y 保持不变，将芯片坐标系转换到目标机体系。
 */
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

/**
 * @brief  对 FIFO batch 中多个 sample 的温度做平均并填入输出 Sample
 * @param  samples     已解码的 FIFO sample 数组
 * @param  sample_count sample 数量
 * @param  output      输出 Sample，仅写入 temperature_deg_c 和 temp_raw
 * @retval Status::Ok               温度计算成功
 * @retval Status::InvalidArgument  参数无效
 */
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

/**
 * @brief  ISR 通知入口：锁存 MCU 时间戳、置 pending、累计中断次数
 * @param  timestamp_us 当前中断时刻的 MCU 微秒时间戳
 *
 * @note   本函数在 ISR 上下文中执行，不访问 SPI、不读 FIFO。
 *         实际处理由 RunImpl() 的 FIFO_READ 状态在普通上下文中完成。
 */
void ICM42688P::DataReady(const uint64_t timestamp_us)
{
    last_data_ready_timestamp_us_ = timestamp_us;
    ++data_ready_interrupt_count_;
    data_ready_pending_ = 1u;
}

/**
 * @brief  在普通上下文中原子消费 ISR 置位的 data-ready pending 标志
 * @param  timestamp_sample_us 最近一次 data-ready 的 MCU 时间戳（输出参数）
 * @retval true  有 pending 事件已消费
 * @retval false 无 pending 事件
 */
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
