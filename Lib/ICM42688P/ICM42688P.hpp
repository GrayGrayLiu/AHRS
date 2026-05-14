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
#include "ICM42688P_Regisrers.hpp"

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
    ICM42688P(SPI_HandleTypeDef* hspi,
              GPIO_TypeDef* cs_port, uint16_t cs_pin);

    bool Init();
    bool Update();
    bool ReadLatest(int16_t accel[3], int16_t gyro[3], int16_t *temp) const;

    // ===== 单字节 =====
    void WriteByte(uint8_t reg, uint8_t value) const;
    void ReadByte(uint8_t reg);

    // ===== 多字节 =====
    void ReadBytes(uint8_t reg, uint16_t len);

    // DMA完成标志
    volatile bool transfer_done_{false};

    // 获取数据（跳过dummy）
    uint8_t* GetRxData() { return &rx_buf_[1]; }

    // HAL回调入口
    void TxRxCpltCallback(SPI_HandleTypeDef* hspi);

private:
    bool CheckWhoAmI();
    bool SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS bank, bool force = false);

    bool WriteRegister(ICM42688P_Regs::RegsAdd::BANK0 reg, uint8_t value);
    bool WriteRegister(ICM42688P_Regs::RegsAdd::BANK1 reg, uint8_t value);
    bool WriteRegister(ICM42688P_Regs::RegsAdd::BANK2 reg, uint8_t value);

    bool ReadRegister(ICM42688P_Regs::RegsAdd::BANK0 reg, uint8_t &value);
    bool ReadRegister(ICM42688P_Regs::RegsAdd::BANK1 reg, uint8_t &value);
    bool ReadRegister(ICM42688P_Regs::RegsAdd::BANK2 reg, uint8_t &value);

    bool WriteRegisterRaw(uint8_t reg, uint8_t value) const;
    bool ReadRegisterRaw(uint8_t reg, uint8_t &value) const;

    void CS_Low() const;
    void CS_High() const;

    SPI_HandleTypeDef* hspi_;
    GPIO_TypeDef* cs_port_;
    uint16_t cs_pin_;

    // DMA缓冲区（必须是成员变量）
    static constexpr uint16_t MAX_LEN = 256;
    uint8_t tx_buf_[MAX_LEN + 1];
    uint8_t rx_buf_[MAX_LEN + 1];

    uint16_t rx_len_{0};

	using BANK0 = ICM42688P_Regs::RegsAdd::BANK0;
	using BANK1 = ICM42688P_Regs::RegsAdd::BANK1;
	using BANK2 = ICM42688P_Regs::RegsAdd::BANK2;
    uint8_t checked_register_bank0_{0};
	static constexpr uint8_t size_register_bank0_cfg{16};
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

		// { BANK0::INTF_CONFIG1,         0, 0}, // RTC_MODE[2] set at runtime

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

		//设置
		// { BANK0::TMST_CONFIG,TMST_CONFIG_BIT::TMST_EN | TMST_CONFIG_BIT::TMST_DELTA_EN | TMST_CONFIG_BIT::TMST_TO_REGS_EN | TMST_CONFIG_BIT::TMST_RES,TMST_CONFIG_BIT::TMST_FSYNC_EN },

		//设置禁止部分读取FIFO（默认）。设置启用FIFO watermark中断。设置陀螺、加计、温度采用最高位数精度数据。设置在FIFO禁用帧同步数据。设置启用温度数据。设置启用陀螺数据。设置启用加计数据。
		{ BANK0::FIFO_CONFIG1,
			static_cast<uint8_t>(FIFO_CONFIG1_BITS::FIFO_WM_GT_TH | FIFO_CONFIG1_BITS::FIFO_HIRES_EN | FIFO_CONFIG1_BITS::FIFO_TEMP_EN | FIFO_CONFIG1_BITS::FIFO_GYRO_EN | FIFO_CONFIG1_BITS::FIFO_ACCEL_EN),
			static_cast<uint8_t>(FIFO_CONFIG1_BITS::FIFO_WM_GT_TH | FIFO_CONFIG1_BITS::FIFO_HIRES_EN | FIFO_CONFIG1_BITS::FIFO_TMST_FSYNC_EN | FIFO_CONFIG1_BITS::FIFO_TEMP_EN | FIFO_CONFIG1_BITS::FIFO_GYRO_EN | FIFO_CONFIG1_BITS::FIFO_ACCEL_EN)},

		// { BANK0::FIFO_CONFIG2,         0, 0 }, // FIFO_WM[7:0] set at runtime
		// { BANK0::FIFO_CONFIG3,         0, 0 }, // FIFO_WM[11:8] set at runtime

		//设置Data Ready中断清除时机（默认）。设置FIFO Threshold中断清除时机为FIFO数据被读取1字节之后。设置FIFO Full中断清除时机（默认）。
		{ BANK0::INT_CONFIG0,
			static_cast<uint8_t>(INT_CONFIG0_BITS::FIFO_THS_INT_CLEAR_FIFO_READ),
			static_cast<uint8_t>(INT_CONFIG0_BITS::FIFO_THS_INT_CLEAR_MASK)},

		//设置
		{ BANK0::INT_CONFIG1,
			static_cast<uint8_t>(0),
			static_cast<uint8_t>(INT_CONFIG1_BITS::INT_ASYNC_RESET)},

		//设置Reset done中断定向到INT1（默认）。设置FIFO threshold中断定向到INT1。
		{ BANK0::INT_SOURCE0,
			static_cast<uint8_t>(INT_SOURCE0_BITS::FIFO_THS_INT1_EN),
			static_cast<uint8_t>(INT_SOURCE0_BITS::FIFO_THS_INT1_EN)},
	};

	uint8_t checked_register_bank1_{0};
	static constexpr uint8_t size_register_bank1_cfg{5};
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

		//运行时设置PIN9的功能为CLKIN
		// { BANK1::INTF_CONFIG5,
		// 	static_cast<uint8_t>(0),
		// 	static_cast<uint8_t>(0)},
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
    uint32_t error_count_{0};
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


