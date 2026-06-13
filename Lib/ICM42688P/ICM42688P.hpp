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

class ICM42688P
{
public:
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
    };

    ICM42688P(SPI_HandleTypeDef* hspi,
              GPIO_TypeDef* cs_port, uint16_t cs_pin);

    // Bare-metal initialization: probe, soft reset, configure the FIFO polling
    // path, verify the applied register subset, then enter running state.
    [[nodiscard]] Status Init();
    [[nodiscard]] Status Probe();
    [[nodiscard]] Status Reset();

    [[nodiscard]] Status RegisterRead(ICM42688P_Regs::RegsAdd::BANK0 reg, uint8_t& value);
    [[nodiscard]] Status RegisterWrite(ICM42688P_Regs::RegsAdd::BANK0 reg, uint8_t value);
    [[nodiscard]] Status ReadBuffer(ICM42688P_Regs::RegsAdd::BANK0 start_reg,
                                    uint8_t* buffer,
                                    uint16_t length);
    [[nodiscard]] Status ReadRawAccel(RawVector& data);
    [[nodiscard]] Status ReadRawGyro(RawVector& data);

    [[nodiscard]] Status Update();
    [[nodiscard]] Status GetLatest(Sample& sample) const;

private:
    enum class DriverState : uint8_t
    {
        Uninitialized,
        Probing,
        Resetting,
        Configuring,
        Running,
        Reconfiguring,
        Error,
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

    [[nodiscard]] Status Configure();
    [[nodiscard]] Status ConfigureFifoWatermark(uint16_t watermark_bytes);
    [[nodiscard]] Status FifoReset();
    [[nodiscard]] Status FifoReadCount(uint16_t& count_bytes);
    [[nodiscard]] Status FifoReadBatch(uint16_t requested_packets, uint16_t& valid_packets);
    [[nodiscard]] Status DecodeFifoPacket(const ICM42688P_Regs::FIFO::DATA& packet,
                                          FifoDecodedSample& decoded) const;
    [[nodiscard]] Status UpdateFromUiRegisters();
    [[nodiscard]] Status VerifyRegister(ICM42688P_Regs::RegsAdd::BANK0 reg,
                                        uint8_t expected_value);
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
    [[nodiscard]] static constexpr bool ShouldApplyFifoPollingConfig(
        ICM42688P_Regs::RegsAdd::BANK0 reg)
    {
        using FifoPollingBank0 = ICM42688P_Regs::RegsAdd::BANK0;

        switch (reg) {
        case FifoPollingBank0::INT_CONFIG:
        case FifoPollingBank0::INT_CONFIG0:
        case FifoPollingBank0::INT_CONFIG1:
        case FifoPollingBank0::INT_SOURCE0:
            return false;

        default:
            return true;
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
    [[nodiscard]] static constexpr int16_t SaturateToInt16(int32_t value)
    {
        return value > 32767 ? static_cast<int16_t>(32767)
             : value < -32768 ? static_cast<int16_t>(-32768)
             : static_cast<int16_t>(value);
    }

    void CS_Low() const;
    void CS_High() const;

    SPI_HandleTypeDef* hspi_;
    GPIO_TypeDef* cs_port_;
    uint16_t cs_pin_;

    uint8_t tx_buf_[ICM42688P_Regs::MAX_READ_LENGTH + ICM42688P_Regs::SPI_COMMAND_LENGTH]{};
    uint8_t rx_buf_[ICM42688P_Regs::MAX_READ_LENGTH + ICM42688P_Regs::SPI_COMMAND_LENGTH]{};

    static constexpr uint16_t FIFO_PACKET_SIZE{sizeof(ICM42688P_Regs::FIFO::DATA)};
    static constexpr uint16_t FIFO_ODR_HZ{8000u};
    static constexpr uint16_t FIFO_UPDATE_HZ{100u};
    static constexpr uint16_t FIFO_EXPECTED_PACKETS_PER_UPDATE{FIFO_ODR_HZ / FIFO_UPDATE_HZ};
    static constexpr uint16_t FIFO_MAX_PACKETS_PER_UPDATE{96u};
    static constexpr uint16_t FIFO_WATERMARK_BYTES{1600u};
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
    static constexpr uint8_t DATA_SOURCE_UI_REGISTERS{1u};
    static constexpr uint8_t DATA_SOURCE_FIFO{2u};
    static_assert(FIFO_PACKET_SIZE == 20u, "ICM42688P high-resolution FIFO packet must be 20 bytes");

    uint8_t fifo_tx_buf_[FIFO_TRANSFER_BUFFER_SIZE]{};
    uint8_t fifo_rx_buf_[FIFO_TRANSFER_BUFFER_SIZE]{};

	using BANK0 = ICM42688P_Regs::RegsAdd::BANK0;
	using BANK1 = ICM42688P_Regs::RegsAdd::BANK1;
	using BANK2 = ICM42688P_Regs::RegsAdd::BANK2;
    uint8_t checked_register_bank0_{0};
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

	uint8_t checked_register_bank1_{0};
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

	uint8_t checked_register_bank2_{0};
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

    bool initialized_{false};
    bool configured_{false};
    DriverState state_{DriverState::Uninitialized};
    Status last_status_{Status::Ok};
    uint32_t failure_count_{0};
    uint32_t sample_count_{0};
    uint32_t error_count_{0};
    uint32_t last_update_ms_{0};
    Sample latest_{};
    FifoDecodedSample fifo_last_decoded_{};
    uint16_t last_fifo_count_bytes_{0};
    uint16_t last_fifo_valid_packets_{0};
    bool bank_selected_valid_{false};
    ICM42688P_Regs::REG_BANK_SEL_BITS current_bank_{ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_0};
};

    // void WriteBits(uint8_t reg, uint8_t set_bits, uint8_t clear_bits)
    // {
    //     uint8_t val = ReadReg(reg);
    //     val |= set_bits;
    //     val &= ~clear_bits;
    //     WriteReg(reg, val);
    // }


