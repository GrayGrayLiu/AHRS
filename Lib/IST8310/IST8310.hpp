/******************************************************************************
 * @file    IST8310.hpp
 * @brief   Minimal blocking I2C driver for the IST8310 magnetometer
 *
 * @details
 * Provides device probing, reset, configuration, single measurements and
 * conversion from raw samples to microtesla. Coordinate transforms,
 * calibration, filtering, scheduling, DMA and interrupts are intentionally
 * outside this first-stage driver.
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

#pragma once

#include <cstdint>

#include "i2c.h"
#include "IST8310_Registers.hpp"

struct ist8310_register_config_t
{
    IST8310_Regs::Register reg{IST8310_Regs::Register::WAI};
    uint8_t setBits{0};
    uint8_t mask{0};
};

class IST8310
{
public:
    enum class Status : int32_t
    {
        Ok = 0,
        InvalidArgument = -1,
        I2cError = -2,
        WrongDeviceId = -3,
        ResetTimeout = -4,
        DataNotReady = -5,
        DataOverrun = -6,
        ConfigMismatch = -7,
    };

    struct RawMagData
    {
        int16_t x{0};
        int16_t y{0};
        int16_t z{0};
    };

    struct MagData_uT
    {
        float x{0.0F};
        float y{0.0F};
        float z{0.0F};
    };

    // If board-level GPIO initialization holds MAG_RSTN low, callers must
    // provide a valid reset_port and reset_pin so Init() can release reset.
    explicit IST8310(I2C_HandleTypeDef* hi2c,
                     uint8_t address_7bit = IST8310_Regs::I2C_ADDRESS_DEFAULT_7BIT,
                     GPIO_TypeDef* reset_port = nullptr,
                     uint16_t reset_pin = 0u);
    ~IST8310() = default;

    [[nodiscard]] Status Init();
    [[nodiscard]] Status Probe() const;
    [[nodiscard]] Status Reset() const;

    [[nodiscard]] Status RegisterRead(IST8310_Regs::Register reg, uint8_t& value) const;
    [[nodiscard]] Status RegisterWrite(IST8310_Regs::Register reg, uint8_t value) const;
    [[nodiscard]] Status ReadBuffer(IST8310_Regs::Register start_reg,
                                    uint8_t* buffer,
                                    uint16_t length) const;

    [[nodiscard]] Status ReadRawMag(RawMagData& data);
    [[nodiscard]] Status ReadMag_uT(MagData_uT& data);

    /**
     * @brief  从已就绪芯片读取磁力计原始数据（不触发测量，不等待 DRDY，不 delay）
     * @param  data 输出：二补码原始数据 x/y/z
     * @retval Ok / I2cError / DataOverrun / DataNotReady
     *
     * @note   调用方必须先写 CNTL1[3:0] = Single Measurement 触发测量，
     *         等待至少 6 ms（低噪声配置），并轮询 STAT1.DRDY 确认数据就绪。
     *         本函数仅执行 burst read + 校验 + 小端拼接，
     *         适合非阻塞 service 状态机的 READ 阶段。
     */
    [[nodiscard]] Status ReadMeasurement(RawMagData& data) const;

private:
    using Register = IST8310_Regs::Register;

    [[nodiscard]] Status ReleaseHardwareReset() const;
    [[nodiscard]] Status Configure();
    [[nodiscard]] Status ApplyRegisterConfig(const ist8310_register_config_t& config) const;
    [[nodiscard]] Status StartSingleMeasurement() const;
    [[nodiscard]] Status WaitDataReady() const;

    [[nodiscard]] uint16_t HalDeviceAddress() const;
    [[nodiscard]] static int16_t CombineLittleEndian(uint8_t low, uint8_t high);

    I2C_HandleTypeDef* hi2c_;
    uint8_t address_7bit_;
    GPIO_TypeDef* reset_port_;
    uint16_t reset_pin_;
    float scale_uT_per_lsb_{IST8310_Regs::DEFAULT_UT_PER_LSB};

    static constexpr uint8_t size_register_cfg_{2};
    ist8310_register_config_t register_cfg_[size_register_cfg_]
    {
        // Register      | Set bits, Mask
        // 设置 PDCNTL 的脉冲持续时间为 Normal，对应手册推荐值 PDCNTL=0xC0。
        {
            Register::PDCNTL,
            static_cast<uint8_t>(IST8310_Regs::PDCNTL_BITS::PULSE_DURATION_NORMAL),
            static_cast<uint8_t>(IST8310_Regs::PDCNTL_BITS::PULSE_DURATION_MASK)
        },

        // 设置 AVGCNTL 中 Y 轴与 X/Z 轴平均次数均为 16 次，对应手册推荐值 AVGCNTL=0x24。
        {
            Register::AVGCNTL,
            static_cast<uint8_t>(IST8310_Regs::AVGCNTL_BITS::Y_AVG_16_TIMES
                                 | IST8310_Regs::AVGCNTL_BITS::XZ_AVG_16_TIMES),
            static_cast<uint8_t>(IST8310_Regs::AVGCNTL_BITS::Y_AVG_MASK
                                 | IST8310_Regs::AVGCNTL_BITS::XZ_AVG_MASK)
        },
    };
};
