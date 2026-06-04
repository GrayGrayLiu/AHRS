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

#ifndef AHRS_IST8310_HPP
#define AHRS_IST8310_HPP

#include <cstdint>

#include "i2c.h"
#include "IST8310_Registers.hpp"

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

private:
    [[nodiscard]] Status ReleaseHardwareReset() const;
    [[nodiscard]] Status Configure();
    [[nodiscard]] Status StartSingleMeasurement() const;
    [[nodiscard]] Status WaitDataReady() const;

    [[nodiscard]] uint16_t HalDeviceAddress() const;
    [[nodiscard]] static int16_t CombineLittleEndian(uint8_t low, uint8_t high);

    I2C_HandleTypeDef* hi2c_;
    uint8_t address_7bit_;
    GPIO_TypeDef* reset_port_;
    uint16_t reset_pin_;
    float scale_uT_per_lsb_{IST8310_Regs::DEFAULT_UT_PER_LSB};
};

#endif //AHRS_IST8310_HPP
