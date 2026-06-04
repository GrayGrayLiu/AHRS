/******************************************************************************
 * @file    IST8310_Registers.hpp
 * @brief   IST8310 register addresses, configuration values and timing
 *
 * @details
 * The customer-defined register map and default sensitivity are based on the
 * provided IST8310 datasheet version 1.2. CNTL3 and the 16-bit sensitivity
 * configuration are based on the PX4 IST8310 driver because they are not
 * clearly described in the provided datasheet.
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

#ifndef AHRS_IST8310_REGISTERS_HPP
#define AHRS_IST8310_REGISTERS_HPP

#include <cstddef>
#include <cstdint>

namespace IST8310_Regs
{
    inline constexpr uint8_t BitNone = 0u;
    inline constexpr uint8_t Bit0 = 1u << 0;
    inline constexpr uint8_t Bit1 = 1u << 1;

    inline constexpr uint8_t I2C_ADDRESS_DEFAULT_7BIT = 0x0C;
    inline constexpr uint8_t I2C_ADDRESS_MIN_7BIT = 0x0C;
    inline constexpr uint8_t I2C_ADDRESS_MAX_7BIT = 0x0F;
    inline constexpr uint8_t HAL_I2C_ADDRESS_SHIFT = 1u;
    inline constexpr uint8_t DEVICE_ID = 0x10;

    enum class Register : uint8_t
    {
        WAI = 0x00,

        STAT1 = 0x02,
        DATAXL = 0x03,
        DATAXH = 0x04,
        DATAYL = 0x05,
        DATAYH = 0x06,
        DATAZL = 0x07,
        DATAZH = 0x08,
        STAT2 = 0x09,
        CNTL1 = 0x0A,
        CNTL2 = 0x0B,
        STR = 0x0C,

        // PX4 uses CNTL3 to select 16-bit output. It is not clearly described
        // in the provided IST8310 datasheet version 1.2.
        CNTL3 = 0x0D,

        AVGCNTL = 0x41,
        PDCNTL = 0x42,
    };

    enum class STAT1_BITS : uint8_t
    {
        DRDY = Bit0,
        DOR = Bit1,
    };

    enum class CNTL1_VALUES : uint8_t
    {
        STANDBY = BitNone,
        SINGLE_MEASUREMENT = Bit0,
    };

    enum class CNTL2_BITS : uint8_t
    {
        SRST = Bit0,
    };

    inline constexpr uint8_t AVGCNTL_LOW_NOISE_RECOMMENDED = 0x24;
    inline constexpr uint8_t PDCNTL_NORMAL_RECOMMENDED = 0xC0;

    // PX4 configures X/Y/Z for 16-bit output with CNTL3 = 0x70 and scales
    // samples using 1320 LSB/Gauss. This mode is not clearly described in the
    // provided datasheet.
    // TODO: If hardware validation fails, set this to false and use the
    // datasheet-default sensitivity.
    inline constexpr uint8_t CNTL3_PX4_16BIT_CONFIG = 0x70;
    inline constexpr bool DEFAULT_USE_PX4_16BIT_MODE = true;

    inline constexpr float DATASHEET_DEFAULT_LSB_PER_UT = 3.3F;
    inline constexpr float DATASHEET_DEFAULT_UT_PER_LSB = 1.0F / DATASHEET_DEFAULT_LSB_PER_UT;
    inline constexpr float UT_PER_GAUSS = 100.0F;
    inline constexpr float PX4_16BIT_LSB_PER_GAUSS = 1320.0F;
    inline constexpr float PX4_16BIT_UT_PER_LSB = UT_PER_GAUSS / PX4_16BIT_LSB_PER_GAUSS;
    inline constexpr float DEFAULT_UT_PER_LSB =
        DEFAULT_USE_PX4_16BIT_MODE ? PX4_16BIT_UT_PER_LSB : DATASHEET_DEFAULT_UT_PER_LSB;

    inline constexpr uint32_t I2C_TRANSACTION_TIMEOUT_MS = 20u;
    inline constexpr uint32_t POR_RESET_WAIT_MS = 50u;
    inline constexpr uint32_t SOFT_RESET_TIMEOUT_MS = 100u;
    inline constexpr uint32_t RESET_POLL_INTERVAL_MS = 5u;
    inline constexpr uint32_t SINGLE_MEASUREMENT_MIN_WAIT_MS = 6u;
    inline constexpr uint32_t SINGLE_MEASUREMENT_TIMEOUT_MS = 20u;
    inline constexpr uint32_t DATA_READY_POLL_INTERVAL_MS = 1u;

    inline constexpr uint16_t REGISTER_VALUE_LENGTH = 1u;
    inline constexpr uint8_t HIGH_BYTE_SHIFT = 8u;

    inline constexpr uint16_t DATA_BURST_LENGTH = 7u;
    inline constexpr std::size_t DATA_BURST_STAT1_INDEX = 0u;
    inline constexpr std::size_t DATA_BURST_X_L_INDEX = 1u;
    inline constexpr std::size_t DATA_BURST_X_H_INDEX = 2u;
    inline constexpr std::size_t DATA_BURST_Y_L_INDEX = 3u;
    inline constexpr std::size_t DATA_BURST_Y_H_INDEX = 4u;
    inline constexpr std::size_t DATA_BURST_Z_L_INDEX = 5u;
    inline constexpr std::size_t DATA_BURST_Z_H_INDEX = 6u;
}

#endif //AHRS_IST8310_REGISTERS_HPP
