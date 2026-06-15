/******************************************************************************
 * @file    ICM42688P.hpp
 * @brief   ICM42688P头文件
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

#pragma once

#include "spi.h"
#include "ICM42688P_Registers.hpp"

using namespace ICM42688P_Regs;

struct register_bank0_config_t
{
    ICM42688P_Regs::RegsAdd::BANK0 reg{0};
    uint8_t setBits{0};
    uint8_t mask{0};
};

struct register_bank1_config_t
{
    ICM42688P_Regs::RegsAdd::BANK1 reg{0};
    uint8_t setBits{0};
    uint8_t mask{0};
};

struct register_bank2_config_t
{
    ICM42688P_Regs::RegsAdd::BANK2 reg{0};
    uint8_t setBits{0};
    uint8_t mask{0};
};

// ICM42688P 底层驱动类。
//
// 本类负责 SPI 寄存器访问、PX4 风格生命周期状态机、FIFO batch 解码与惯性
// 增量计算；scheduler 调度、应用层 LED 和调试输出由 ICM42688 Service 负责。
class ICM42688P
{
public:
    // 驱动内部及对外统一使用的状态码。
    enum class Status : int32_t
    {
        Ok = 0,
        InvalidArgument = -1,
        SpiError = -2,
        WrongDeviceId = -3,
        ResetTimeout = -4,
        Unsupported = -5,
        ConfigMismatch = -6,
        NoData = -7,
        FifoOverflow = -8,
        BadFifoPacket = -9,
    };

    struct RawVector
    {
        int16_t x{0};
        int16_t y{0};
        int16_t z{0};
    };

    struct Sample
    {
        uint32_t timestamp_ms{0};
        int16_t accel_raw[3]{};
        int16_t gyro_raw[3]{};
        int16_t temp_raw{0};
        float accel_m_s2[3]{};
        float gyro_rad_s[3]{};
        float temperature_deg_c{0.0F};
        uint32_t sample_counter{0};
        uint32_t error_counter{0};
        bool configured{false};
        bool data_valid{false};
        int32_t accel_raw20[3]{};
        int32_t gyro_raw20[3]{};
        int32_t accel_effective[3]{};
        int32_t gyro_effective[3]{};
        uint16_t fifo_count_bytes{0};
        uint16_t fifo_valid_packets{0};
        uint16_t fifo_timestamp{0};
        uint8_t fifo_header{0};
        uint8_t data_source{0};
        float delta_angle_rad[3]{};
        float delta_velocity_m_s[3]{};
        float delta_time_s{0.0F};
        uint16_t delta_samples{0};
        uint32_t interrupt_counter{0};
        uint32_t last_interrupt_timestamp_ms{0};
        uint64_t timestamp_us{0};
        uint64_t last_interrupt_timestamp_us{0};
    };

    ICM42688P(SPI_HandleTypeDef* hspi,
              GPIO_TypeDef* cs_port, uint16_t cs_pin);

    // 启动裸机驱动状态机。Init() 只初始化软件状态并进入 RESET，硬件 reset、
    // configure 和 FIFO 启动由外部后续调用 Update() 时通过 RunImpl() 推进。
    [[nodiscard]] Status Init();
    [[nodiscard]] Status Probe();
    // 重新把异步生命周期置于 RESET；真正的 SPI soft reset 仍由 RunImpl() 执行。
    [[nodiscard]] Status Reset();
    // 唯一运行入口，Service 的响应式事件和周期兜底路径最终都调用该函数。
    [[nodiscard]] Status Update();
    // ISR 通知入口：只记录 data-ready 时间戳和 pending，不执行 SPI/FIFO 读取。
    void DataReady(uint64_t timestamp_us);
    // 拷贝最新有效缓存，不访问 SPI，也不消费 sample。
    [[nodiscard]] Status GetLatest(Sample& sample) const;

    // 硬件调试和初期联调访问接口，不属于正常 FIFO 主运行链路。
    [[nodiscard]] Status RegisterRead(ICM42688P_Regs::RegsAdd::BANK0 reg, uint8_t& value);
    [[nodiscard]] Status RegisterWrite(ICM42688P_Regs::RegsAdd::BANK0 reg, uint8_t value);
    [[nodiscard]] Status ReadBuffer(ICM42688P_Regs::RegsAdd::BANK0 start_reg,
                                    uint8_t* buffer,
                                    uint16_t length);
    [[nodiscard]] Status ReadRawAccel(RawVector& data);
    [[nodiscard]] Status ReadRawGyro(RawVector& data);

private:
    // 裸机版 PX4 生命周期状态：RunImpl() 在外部提供运行机会时逐步推进这些状态。
    enum class DriverState : uint8_t
    {
        RESET,
        WAIT_FOR_RESET,
        CONFIGURE,
        FIFO_RESET,
        FIFO_READ,
    };

    struct FifoDecodedSample
    {
        int32_t accel_raw20[3]{};
        int32_t gyro_raw20[3]{};
        int32_t accel_effective[3]{};
        int32_t gyro_effective[3]{};
        int16_t accel_raw16[3]{};
        int16_t gyro_raw16[3]{};
        int16_t temp_raw{0};
        uint16_t timestamp_fifo{0};
        uint8_t header{0};
        float accel_m_s2[3]{};
        float gyro_rad_s[3]{};
        float temperature_deg_c{0.0F};
    };

    [[nodiscard]] Status WaitForReset();
    [[nodiscard]] Status Configure();
    [[nodiscard]] Status ConfigureFIFOWatermark(uint16_t watermark_bytes);
    [[nodiscard]] Status ConfigureInterrupt();
    [[nodiscard]] Status FIFOReset();
    [[nodiscard]] Status RunImpl();
    [[nodiscard]] bool ConsumeDataReady(uint64_t& timestamp_sample_us);
    [[nodiscard]] Status FIFORead(uint64_t timestamp_sample_us);
    [[nodiscard]] Status FIFOReadCount(uint16_t& count_bytes);
    [[nodiscard]] Status FIFOReadData(uint16_t requested_packets, uint16_t& valid_packets);
    [[nodiscard]] Status ProcessTemperature(const FifoDecodedSample samples[],
                                            uint16_t sample_count,
                                            Sample& output) const;
    [[nodiscard]] Status ProcessGyro(const FifoDecodedSample samples[],
                                     uint16_t sample_count,
                                     float sample_dt_s,
                                     Sample& output);
    [[nodiscard]] Status ProcessAccel(const FifoDecodedSample samples[],
                                      uint16_t sample_count,
                                      float sample_dt_s,
                                      Sample& output);
    // 只负责填充一次成功 FIFO batch 对应 Sample 的元信息字段，不参与积分计算。
    void PopulateFifoSampleMetadata(Sample& sample,
                                    uint64_t timestamp_sample_us,
                                    uint32_t timestamp_sample_ms,
                                    uint16_t valid_packets) const;
    [[nodiscard]] Status DecodeFifoPacket(const ICM42688P_Regs::FIFO::DATA& packet,
                                          FifoDecodedSample& decoded) const;
    [[nodiscard]] Status RegisterSetAndClearBits(const register_bank0_config_t& config);
    [[nodiscard]] Status RegisterSetAndClearBits(const register_bank1_config_t& config);
    [[nodiscard]] Status RegisterSetAndClearBits(const register_bank2_config_t& config);
    [[nodiscard]] Status RegisterCheck(const register_bank0_config_t& config);
    [[nodiscard]] Status RegisterCheck(const register_bank1_config_t& config);
    [[nodiscard]] Status RegisterCheck(const register_bank2_config_t& config);
    [[nodiscard]] Status SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS bank, bool force = false);

    [[nodiscard]] Status WriteRegister(ICM42688P_Regs::RegsAdd::BANK1 reg, uint8_t value);
    [[nodiscard]] Status WriteRegister(ICM42688P_Regs::RegsAdd::BANK2 reg, uint8_t value);

    [[nodiscard]] Status ReadRegister(ICM42688P_Regs::RegsAdd::BANK1 reg, uint8_t& value);
    [[nodiscard]] Status ReadRegister(ICM42688P_Regs::RegsAdd::BANK2 reg, uint8_t& value);

    [[nodiscard]] Status WriteRegisterRaw(uint8_t reg, uint8_t value) const;
    [[nodiscard]] Status ReadRegisterRaw(uint8_t reg, uint8_t& value);
    [[nodiscard]] Status ReadBufferRaw(uint8_t start_reg, uint8_t* buffer, uint16_t length);
    [[nodiscard]] bool HasValidBus() const;
    [[nodiscard]] static int16_t CombineBigEndian(uint8_t high, uint8_t low);
    [[nodiscard]] static constexpr bool TickReached(uint32_t now, uint32_t target)
    {
        return static_cast<int32_t>(now - target) >= 0;
    }
    // ScheduleNow/ScheduleDelayed 不是 RTOS 或工作队列调度，只更新驱动内部
    // 下一次允许执行的毫秒时间。真正的运行机会始终由 Service::Run() 提供。
    void ScheduleNow(uint32_t now)
    {
        next_run_ms_ = now;
    }
    void ScheduleDelayed(uint32_t now, uint32_t delay_ms)
    {
        next_run_ms_ = now + delay_ms;
    }
    [[nodiscard]] static constexpr uint8_t ComposeRegisterValue(uint8_t old_value,
                                                                 uint8_t set_bits,
                                                                 uint8_t mask)
    {
        return static_cast<uint8_t>((old_value & static_cast<uint8_t>(~mask))
                                    | (set_bits & mask));
    }
    [[nodiscard]] static constexpr bool RegisterValueMatches(uint8_t value,
                                                              uint8_t set_bits,
                                                              uint8_t mask)
    {
        return (value & mask) == (set_bits & mask);
    }
    [[nodiscard]] static constexpr bool IsInterruptConfigRegister(
        ICM42688P_Regs::RegsAdd::BANK0 reg)
    {
        using InterruptBank0 = ICM42688P_Regs::RegsAdd::BANK0;

        switch (reg) {
        case InterruptBank0::INT_CONFIG:
        case InterruptBank0::INT_CONFIG0:
        case InterruptBank0::INT_CONFIG1:
        case InterruptBank0::INT_SOURCE0:
            return true;

        default:
            return false;
        }
    }
    [[nodiscard]] static constexpr bool IsValidFifoHeader(uint8_t header)
    {
        const uint8_t message = static_cast<uint8_t>(ICM42688P_Regs::FIFO::FIFO_HEADER_BIT::HEADER_MSG);
        const uint8_t accel = static_cast<uint8_t>(ICM42688P_Regs::FIFO::FIFO_HEADER_BIT::HEADER_ACCEL);
        const uint8_t gyro = static_cast<uint8_t>(ICM42688P_Regs::FIFO::FIFO_HEADER_BIT::HEADER_GYRO);
        const uint8_t high_resolution = static_cast<uint8_t>(ICM42688P_Regs::FIFO::FIFO_HEADER_BIT::HEADER_20);
        const uint8_t timestamp_mask = static_cast<uint8_t>(
            ICM42688P_Regs::FIFO::FIFO_HEADER_BIT::HEADER_TIMESTAMP_FSYNC);
        const uint8_t accel_odr_change = static_cast<uint8_t>(
            ICM42688P_Regs::FIFO::FIFO_HEADER_BIT::HEADER_ODR_ACCEL);
        const uint8_t gyro_odr_change = static_cast<uint8_t>(
            ICM42688P_Regs::FIFO::FIFO_HEADER_BIT::HEADER_ODR_GYRO);

        return (header & message) == 0u
            && (header & accel) != 0u
            && (header & gyro) != 0u
            && (header & high_resolution) != 0u
            && (header & timestamp_mask) == ICM42688P_Regs::Bit3
            && (header & accel_odr_change) == 0u
            && (header & gyro_odr_change) == 0u;
    }
    [[nodiscard]] static constexpr int32_t Reassemble20Bit(uint8_t high,
                                                            uint8_t low,
                                                            uint8_t extension_low_nibble)
    {
        uint32_t value = (static_cast<uint32_t>(high) << 12u)
                       | (static_cast<uint32_t>(low) << 4u)
                       | (static_cast<uint32_t>(extension_low_nibble) & 0x0Fu);

        if ((high & ICM42688P_Regs::Bit7) != 0u) {
            value |= 0xFFF00000u;
        }

        return static_cast<int32_t>(value);
    }

    [[nodiscard]] static constexpr int32_t AxisSign(const uint8_t axis)
    {
        return axis == 1u ? 1 : -1;
    }
    [[nodiscard]] static constexpr int16_t SaturateToInt16(int32_t value)
    {
        return value > 32767 ? static_cast<int16_t>(32767)
             : value < -32768 ? static_cast<int16_t>(-32768)
             : static_cast<int16_t>(value);
    }

    void CS_Low() const;
    void CS_High() const;

    // ── SPI 总线与片选绑定（构造时初始化，必须保持为前三个非静态成员） ──
    SPI_HandleTypeDef* hspi_;
    GPIO_TypeDef* cs_port_;
    uint16_t cs_pin_;

    // ── 寄存器访问 SPI 收发缓冲区 ──
    uint8_t tx_buf_[ICM42688P_Regs::MAX_READ_LENGTH + ICM42688P_Regs::SPI_COMMAND_LENGTH]{};
    uint8_t rx_buf_[ICM42688P_Regs::MAX_READ_LENGTH + ICM42688P_Regs::SPI_COMMAND_LENGTH]{};

    // FIFO 数据链路固定参数与传输缓冲区布局。
    static constexpr uint16_t FIFO_PACKET_SIZE{sizeof(ICM42688P_Regs::FIFO::DATA)};
    static constexpr uint16_t FIFO_ODR_HZ{8000u};
    static constexpr float FIFO_SAMPLE_DT_S{1.0F / static_cast<float>(FIFO_ODR_HZ)};
    static constexpr uint16_t FIFO_OUTPUT_RATE_HZ{400u};
    static constexpr uint16_t FIFO_WATERMARK_PACKETS{FIFO_ODR_HZ / FIFO_OUTPUT_RATE_HZ};
    static constexpr uint16_t FIFO_MAX_PACKETS_PER_UPDATE{96u};
    static constexpr uint16_t FIFO_WATERMARK_BYTES{FIFO_WATERMARK_PACKETS * FIFO_PACKET_SIZE};
    static constexpr uint16_t FIFO_TRANSFER_PREFIX_BYTES{3u};
    static constexpr uint16_t FIFO_TRANSFER_DATA_BYTES{
        FIFO_MAX_PACKETS_PER_UPDATE * FIFO_PACKET_SIZE};
    static constexpr uint16_t FIFO_TRANSFER_BUFFER_SIZE{
        ICM42688P_Regs::SPI_COMMAND_LENGTH + FIFO_TRANSFER_PREFIX_BYTES
        + FIFO_TRANSFER_DATA_BYTES};
    static constexpr uint16_t FIFO_RX_INT_STATUS_INDEX{1u};
    static constexpr uint16_t FIFO_RX_COUNT_HIGH_INDEX{2u};
    static constexpr uint16_t FIFO_RX_COUNT_LOW_INDEX{3u};
    static constexpr uint16_t FIFO_RX_DATA_INDEX{4u};
    static constexpr uint8_t DATA_SOURCE_FIFO{2u};
    static_assert(FIFO_PACKET_SIZE == 20u, "ICM42688P high-resolution FIFO packet must be 20 bytes");
    static_assert(FIFO_ODR_HZ % FIFO_OUTPUT_RATE_HZ == 0u,
                  "FIFO output rate must divide the sensor ODR exactly");
    static_assert(FIFO_WATERMARK_PACKETS == 20u && FIFO_WATERMARK_BYTES == 400u,
                  "400 Hz FIFO output requires a 20-packet, 400-byte watermark");

    // ── FIFO 批量 SPI 读写缓冲区 ──
    uint8_t fifo_tx_buf_[FIFO_TRANSFER_BUFFER_SIZE]{};
    uint8_t fifo_rx_buf_[FIFO_TRANSFER_BUFFER_SIZE]{};

	using BANK0 = ICM42688P_Regs::RegsAdd::BANK0;
	using BANK1 = ICM42688P_Regs::RegsAdd::BANK1;
	using BANK2 = ICM42688P_Regs::RegsAdd::BANK2;
	static constexpr uint8_t size_register_bank0_cfg{13};
	register_bank0_config_t register_bank0_cfg_[size_register_bank0_cfg]
	{
		// Register                              | Set bits, Clear bits
		//INT2中断引脚的设置为默认。设置INT1中断引脚的信号用锁存模式而不是脉冲模式。设置INT1引脚驱动电路用推挽而不是开漏。设置INT1引脚极性为低电平有效而不是高电平有效。
		{ BANK0::INT_CONFIG,
			static_cast<uint8_t>(INT_CONFIG_BITS::INT1_MODE | INT_CONFIG_BITS::INT1_DRIVE_CIRCUIT),
			static_cast<uint8_t>(INT_CONFIG_BITS::INT1_MODE | INT_CONFIG_BITS::INT1_DRIVE_CIRCUIT | INT_CONFIG_BITS::INT1_POLARITY)},

		//设置FIFO为装满数据则停止进入新数据的工作模式。
		{ BANK0::FIFO_CONFIG,
			static_cast<uint8_t>(FIFO_CONFIG_BITS::FIFO_MODE_STOP_ON_FULL),
			static_cast<uint8_t>(FIFO_CONFIG_BITS::FIFO_MODE_MASK) },

		//设置温度数据启用（默认）。设置RC振荡器在陀螺和加计空闲时不启用（默认）。设置陀螺为低噪声模式。设置加计为低噪声模式。
		{ BANK0::PWR_MGMT0,
			static_cast<uint8_t>(PWR_MGMT0_BITS::GYRO_MODE_GYRO_LOW_NOISE | PWR_MGMT0_BITS::ACCEL_MODE_ACC_LOW_NOISE),
			static_cast<uint8_t>(PWR_MGMT0_BITS::GYRO_MODE_MASK | PWR_MGMT0_BITS::ACCEL_MODE_MASK) },

		//设置陀螺量程为±2000deg/s。设置陀螺输出速率为8kHz。
		{ BANK0::GYRO_CONFIG0,
			static_cast<uint8_t>(GYRO_CONFIG0_BITS::GYRO_FS_SEL_2000DPS | GYRO_CONFIG0_BITS::GYRO_ODR_8KHZ),
			static_cast<uint8_t>(GYRO_CONFIG0_BITS::GYRO_FS_SEL_MASK | GYRO_CONFIG0_BITS::GYRO_ODR_MASK)},

		//设置加计量程为±16g。设置加计输出速率为8kHz。
		{ BANK0::ACCEL_CONFIG0,
			static_cast<uint8_t>(ACCEL_CONFIG0_BITS::ACCEL_FS_SEL_16G | ACCEL_CONFIG0_BITS::ACCEL_ODR_8KHZ),
			static_cast<uint8_t>(ACCEL_CONFIG0_BITS::ACCEL_FS_SEL_MASK | ACCEL_CONFIG0_BITS::ACCEL_ODR_MASK)},

		//设置温度数据的低通滤波器带宽为4000Hz（默认）。设置陀螺的低通滤波器为1阶。设置陀螺的抽取滤波器阶数为3阶（默认）
		{ BANK0::GYRO_CONFIG1,
			static_cast<uint8_t>(GYRO_CONFIG1_BITS::GYRO_UI_FILT_ORD_1),
			static_cast<uint8_t>(GYRO_CONFIG1_BITS::GYRO_UI_FILT_ORD_MASK)},

		//设置陀螺的低通滤波器带宽，超低延迟，抽取滤波器超采样降噪后直出。设置加计的低通滤波器带宽，超低延迟，抽取滤波器超采样降噪后直出。
		{ BANK0::GYRO_ACCEL_CONFIG0,
			static_cast<uint8_t>(GYRO_ACCEL_CONFIG0_BITS::ACCEL_UI_FILT_BW_LOW_LATENCY2 | GYRO_ACCEL_CONFIG0_BITS::GYRO_UI_FILT_BW_LOW_LATENCY2),
			static_cast<uint8_t>(GYRO_ACCEL_CONFIG0_BITS::ACCEL_UI_FILT_BW_MASK | GYRO_ACCEL_CONFIG0_BITS::GYRO_UI_FILT_BW_MASK)},

		//设置加计的低通滤波器为1阶。设置加计的抽取滤波器阶数为3阶（默认）
		{ BANK0::ACCEL_CONFIG1,
			static_cast<uint8_t>(ACCEL_CONFIG1_BITS::ACCEL_UI_FILT_ORD_1),
			static_cast<uint8_t>(ACCEL_CONFIG1_BITS::ACCEL_UI_FILT_ORD_MASK)},

		//设置TMST_VALUE寄存器可被读取时间戳。设置时间戳精度为16us。设置时间戳为增量形式输出。设置时间戳不记录FSYNC。设置启用时间戳寄存器（默认）。
		{ BANK0::TMST_CONFIG,
			static_cast<uint8_t>(TMST_CONFIG_BITS::TMST_TO_REGS_EN | TMST_CONFIG_BITS::TMST_RES | TMST_CONFIG_BITS::TMST_DELTA_EN),
			static_cast<uint8_t>(TMST_CONFIG_BITS::TMST_TO_REGS_EN | TMST_CONFIG_BITS::TMST_RES | TMST_CONFIG_BITS::TMST_DELTA_EN | TMST_CONFIG_BITS::TMST_FSYNC_EN)},

		//设置禁止部分读取FIFO（默认）。设置启用FIFO watermark中断。设置陀螺、加计、温度采用最高位数精度数据。设置在FIFO禁用帧同步数据。设置启用温度数据。设置启用陀螺数据。设置启用加计数据。
		{ BANK0::FIFO_CONFIG1,
			static_cast<uint8_t>(FIFO_CONFIG1_BITS::FIFO_WM_GT_TH | FIFO_CONFIG1_BITS::FIFO_HIRES_EN | FIFO_CONFIG1_BITS::FIFO_TEMP_EN | FIFO_CONFIG1_BITS::FIFO_GYRO_EN | FIFO_CONFIG1_BITS::FIFO_ACCEL_EN),
			static_cast<uint8_t>(FIFO_CONFIG1_BITS::FIFO_WM_GT_TH | FIFO_CONFIG1_BITS::FIFO_HIRES_EN | FIFO_CONFIG1_BITS::FIFO_TMST_FSYNC_EN | FIFO_CONFIG1_BITS::FIFO_TEMP_EN | FIFO_CONFIG1_BITS::FIFO_GYRO_EN | FIFO_CONFIG1_BITS::FIFO_ACCEL_EN)},

		//设置Data Ready中断清除时机（默认）。设置FIFO Threshold中断清除时机为FIFO数据被读取1字节之后。设置FIFO Full中断清除时机（默认）。
		{ BANK0::INT_CONFIG0,
			static_cast<uint8_t>(INT_CONFIG0_BITS::FIFO_THS_INT_CLEAR_FIFO_READ),
			static_cast<uint8_t>(INT_CONFIG0_BITS::FIFO_THS_INT_CLEAR_MASK)},

		//设置INT_ASYNC_RESET: User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation
		{ BANK0::INT_CONFIG1,
			static_cast<uint8_t>(0),
			static_cast<uint8_t>(INT_CONFIG1_BITS::INT_ASYNC_RESET)},

		//设置Reset done中断定向到INT1（默认）。设置FIFO threshold中断定向到INT1。
		{ BANK0::INT_SOURCE0,
			static_cast<uint8_t>(INT_SOURCE0_BITS::FIFO_THS_INT1_EN),
			static_cast<uint8_t>(INT_SOURCE0_BITS::FIFO_THS_INT1_EN)},
	};

	static constexpr uint8_t size_register_bank1_cfg{4};
	register_bank1_config_t register_bank1_cfg_[size_register_bank1_cfg]
	{
		// Register                              | Set bits, Clear bits
		//设置启用陀螺的抗混叠滤波器。设置启用陀螺的陷波器。
		{ BANK1::GYRO_CONFIG_STATIC2,
			static_cast<uint8_t>(0),
			static_cast<uint8_t>(GYRO_CONFIG_STATIC2_BITS::GYRO_NF_DIS | GYRO_CONFIG_STATIC2_BITS::GYRO_AAF_DIS)},

		//设置陀螺的抗混叠滤波器的带宽为585Hz
		{ BANK1::GYRO_CONFIG_STATIC3,
			static_cast<uint8_t>(GYRO_CONFIG_STATIC3_BITS::GYRO_AAF_DELT_585HZ),
			static_cast<uint8_t>(GYRO_CONFIG_STATIC3_BITS::GYRO_AAF_DELT_MASK)},

		//设置陀螺的抗混叠滤波器的带宽为585Hz
		{ BANK1::GYRO_CONFIG_STATIC4,
			static_cast<uint8_t>(GYRO_CONFIG_STATIC4_BITS::GYRO_AAF_DELTSQR_7_0_585HZ),
			static_cast<uint8_t>(GYRO_CONFIG_STATIC4_BITS::GYRO_AAF_DELTSQR_7_0_MASK)},

		//设置陀螺的抗混叠滤波器的带宽为585Hz
		{ BANK1::GYRO_CONFIG_STATIC5,
			static_cast<uint8_t>(GYRO_CONFIG_STATIC5_BITS::GYRO_AAF_BITSHIFT_585HZ | GYRO_CONFIG_STATIC5_BITS::GYRO_AAF_DELTSQR_11_8_585HZ),
			static_cast<uint8_t>(GYRO_CONFIG_STATIC5_BITS::GYRO_AAF_BITSHIFT_MASK | GYRO_CONFIG_STATIC5_BITS::GYRO_AAF_DELTSQR_11_8_MASK)},
	};

	static constexpr uint8_t size_register_bank2_cfg{3};
	register_bank2_config_t register_bank2_cfg_[size_register_bank2_cfg]
	{
		// Register                              | Set bits, Clear bits
		//设置加计的抗混叠滤波器的带宽为585Hz。设置启用加计的抗混叠滤波器（默认）。
		{ BANK2::ACCEL_CONFIG_STATIC2,
			static_cast<uint8_t>(ACCEL_CONFIG_STATIC2_BITS::ACCEL_AAF_DELT_585HZ),
			static_cast<uint8_t>(ACCEL_CONFIG_STATIC2_BITS::ACCEL_AAF_DELT_MASK | ACCEL_CONFIG_STATIC2_BITS::ACCEL_AAF_DIS)},

		//设置加计的抗混叠滤波器的带宽为585Hz。
		{ BANK2::ACCEL_CONFIG_STATIC3,
			static_cast<uint8_t>(ACCEL_CONFIG_STATIC3_BITS::ACCEL_AAF_DELTSQR_7_0_585HZ),
			static_cast<uint8_t>(ACCEL_CONFIG_STATIC3_BITS::ACCEL_AAF_DELTSQR_7_0_MASK)},

		//设置加计的抗混叠滤波器的带宽为585Hz。
		{ BANK2::ACCEL_CONFIG_STATIC4,
			static_cast<uint8_t>(ACCEL_CONFIG_STATIC4_BITS::ACCEL_AAF_BITSHIFT_585HZ | ACCEL_CONFIG_STATIC4_BITS::ACCEL_AAF_DELTSQR_11_8_585HZ),
			static_cast<uint8_t>(ACCEL_CONFIG_STATIC4_BITS::ACCEL_AAF_BITSHIFT_MASK | ACCEL_CONFIG_STATIC4_BITS::ACCEL_AAF_DELTSQR_11_8_MASK)},
	};

    // ── 状态机与生命周期 ──
    // initialized_ 在 Init() 成功后为 true；configured_ 在 CONFIGURE + FIFO_RESET
    // 均成功后为 true。
    bool initialized_{false};
    bool configured_{false};
    volatile DriverState state_{DriverState::RESET};  // PX4 风格 5 态
    Status last_status_{Status::Ok};                  // RunImpl() 最新返回码

    // ── 调度节拍与错误计数 ──
    // next_run_ms_ 为 ScheduleDelayed() / ScheduleNow() 的目标时间，data-ready
    // pending 可使其在 FIFO_READ 下被跳过。
    volatile uint32_t next_run_ms_{0};
    uint32_t failure_count_{0};                        // 连续失败计数，超阈值触发 Reset
    uint32_t reset_timestamp_ms_{0};                   // RESET 状态起始时间戳
    uint32_t sample_count_{0};                         // 累计成功输出 FIFO sample 数
    uint32_t error_count_{0};                          // 驱动层累计错误计数

    // ── 最新输出样本缓存 ──
    Sample latest_{};  // GetLatest() / GetDeltaLatest() 只读访问

    // ── FIFO 解码 batch 缓存 ──
    FifoDecodedSample fifo_last_decoded_{};            // 最近解码的单个 sample
    FifoDecodedSample fifo_decoded_batch_[FIFO_MAX_PACKETS_PER_UPDATE]{};
    uint16_t fifo_decoded_count_{0u};                  // 当前 batch 成功解码 packet 数
    uint16_t last_fifo_count_bytes_{0};                // 最近一次 FIFO 计数值（字节）
    uint16_t last_fifo_valid_packets_{0};              // 最近一次有效 packet 数

    // ── 跨 batch 梯形积分状态 ──
    // 保存上一批末尾 sample 的 effective 值，当前 batch 到达时构成 0.5 边界权重。
    bool accel_last_effective_valid_{false};
    bool gyro_last_effective_valid_{false};
    int32_t accel_last_effective_[3]{};
    int32_t gyro_last_effective_[3]{};

    // ── 实测 batch dt 时间戳基准 ──
    // 相邻成功 data-ready 的 MCU 微秒时间戳，用于 ComputeBatchDeltaTimeSeconds()。
    // FIFO packet 内部 timestamp 不参与此计算。
    uint64_t last_fifo_timestamp_sample_us_{0u};
    bool last_fifo_timestamp_sample_valid_{false};

    // ── ISR data-ready 事件桥接状态 ──
    // DataReady() (ISR) 写入，ConsumeDataReady() (普通上下文) 原子消费。
    volatile uint8_t data_ready_pending_{0u};              // 待处理标志
    volatile uint32_t data_ready_interrupt_count_{0u};     // 累计中断次数
    volatile uint64_t last_data_ready_timestamp_us_{0u};   // 最近一次 data-ready 时间戳

    // ── 寄存器 bank 选择缓存 ──
    bool bank_selected_valid_{false};
    ICM42688P_Regs::REG_BANK_SEL_BITS current_bank_{ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_0};
};
