/******************************************************************************
 * @file    IST8310_Registers.hpp
 * @brief   IST8310 main register descriptions
 *
 * @details
 * Translates the customer-defined register section of the IST8310 Datasheet
 * Version 1.2 into register addresses, bit-field masks and documented values.
 * PX4 reference values that are not described by the datasheet are isolated
 * in PX4_Undocumented and are not enabled by the default driver.
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

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace IST8310_Regs
{
    /***************************************************************Define Bit***********************************************************************/
    inline constexpr uint8_t BitNone = 0u;
    inline constexpr uint8_t Bit0 = 1u << 0;
    inline constexpr uint8_t Bit1 = 1u << 1;
    inline constexpr uint8_t Bit2 = 1u << 2;
    inline constexpr uint8_t Bit3 = 1u << 3;
    inline constexpr uint8_t Bit4 = 1u << 4;
    inline constexpr uint8_t Bit5 = 1u << 5;
    inline constexpr uint8_t Bit6 = 1u << 6;
    inline constexpr uint8_t Bit7 = 1u << 7;
    /************************************************************************************************************************************************/


    /***********************************************************Device / I2C Constants***************************************************************/
    inline constexpr uint8_t I2C_ADDRESS_CAD1_VSS_CAD0_VSS_7BIT = 0x0C;
    inline constexpr uint8_t I2C_ADDRESS_CAD1_VSS_CAD0_VDD_7BIT = 0x0D;
    inline constexpr uint8_t I2C_ADDRESS_CAD1_VDD_CAD0_VSS_7BIT = 0x0E;
    inline constexpr uint8_t I2C_ADDRESS_CAD1_VDD_CAD0_VDD_7BIT = 0x0F;

    // Datasheet default when both CAD pins are floating.
    inline constexpr uint8_t DATASHEET_FLOATING_CAD_I2C_ADDRESS_7BIT =
        I2C_ADDRESS_CAD1_VDD_CAD0_VSS_7BIT;

    // Current board straps CAD1=VSS and CAD0=VSS. This is the board default,
    // not the datasheet floating-CAD default.
    inline constexpr uint8_t BOARD_I2C_ADDRESS_7BIT =
        I2C_ADDRESS_CAD1_VSS_CAD0_VSS_7BIT;

    // Compatibility alias for the existing constructor default parameter.
    // It selects the current board address (CAD1=VSS, CAD0=VSS, 0x0C), not
    // the datasheet floating-CAD default address (0x0E).
    inline constexpr uint8_t I2C_ADDRESS_DEFAULT_7BIT = BOARD_I2C_ADDRESS_7BIT;

    inline constexpr uint8_t I2C_ADDRESS_MIN_7BIT = I2C_ADDRESS_CAD1_VSS_CAD0_VSS_7BIT;
    inline constexpr uint8_t I2C_ADDRESS_MAX_7BIT = I2C_ADDRESS_CAD1_VDD_CAD0_VDD_7BIT;
    inline constexpr uint8_t HAL_I2C_ADDRESS_SHIFT = 1u;
    inline constexpr uint8_t DEVICE_ID = 0x10;
    /************************************************************************************************************************************************/


    /***********************************************************Registers Address********************************************************************/
    namespace RegsAdd
    {
        // Only registers explicitly described by IST8310 Datasheet Version 1.2
        // are allowed in this address enum.
        enum class IST8310 : uint8_t
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

            TEMPL = 0x1C,
            TEMPH = 0x1D,

            AVGCNTL = 0x41,
            PDCNTL = 0x42,
        };
    }

    using Register = RegsAdd::IST8310;
    /************************************************************************************************************************************************/


    /************************************************************Registers Describe******************************************************************/
    /**
     * @Name: WAI
     * @Address: 0 (00h)
     * @Serial IF: R
     * @Reset value: 0x10
     * @brief Device identification register.
     */
    enum class WAI_BITS : uint8_t
    {
        DEVICE_ID_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
    };

    /**
     * @Name: STAT1
     * @Address: 2 (02h)
     * @Serial IF: R
     * @Reset value: 0x00
     * @brief Data status register. Reading any output data register clears
     *        DOR and DRDY.
     */
    enum class STAT1_BITS : uint8_t
    {
        RESERVED_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2,
        DOR_MASK = Bit1,
        DRDY_MASK = Bit0,

        DOR_NO_OVERRUN = BitNone,
        DOR_OVERRUN = Bit1,

        DRDY_NOT_READY = BitNone,
        DRDY_READY = Bit0,
    };

    /**
     * @Name: DATAXL
     * @Address: 3 (03h)
     * @Serial IF: R
     * @Reset value: 0x00
     * @brief Low byte of X-axis two's-complement measurement data. The low
     *        byte is stored before the high byte. No coordinate transform is
     *        implied by this register description.
     */
    enum class DATAXL_BITS : uint8_t
    {
        DATA_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
    };

    /**
     * @Name: DATAXH
     * @Address: 4 (04h)
     * @Serial IF: R
     * @Reset value: 0x00
     * @brief High byte of X-axis two's-complement measurement data.
     */
    enum class DATAXH_BITS : uint8_t
    {
        DATA_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
    };

    /**
     * @Name: DATAYL
     * @Address: 5 (05h)
     * @Serial IF: R
     * @Reset value: 0x00
     * @brief Low byte of Y-axis two's-complement measurement data. The low
     *        byte is stored before the high byte. No coordinate transform is
     *        implied by this register description.
     */
    enum class DATAYL_BITS : uint8_t
    {
        DATA_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
    };

    /**
     * @Name: DATAYH
     * @Address: 6 (06h)
     * @Serial IF: R
     * @Reset value: 0x00
     * @brief High byte of Y-axis two's-complement measurement data.
     */
    enum class DATAYH_BITS : uint8_t
    {
        DATA_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
    };

    /**
     * @Name: DATAZL
     * @Address: 7 (07h)
     * @Serial IF: R
     * @Reset value: 0x00
     * @brief Low byte of Z-axis two's-complement measurement data. The low
     *        byte is stored before the high byte. No coordinate transform is
     *        implied by this register description.
     */
    enum class DATAZL_BITS : uint8_t
    {
        DATA_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
    };

    /**
     * @Name: DATAZH
     * @Address: 8 (08h)
     * @Serial IF: R
     * @Reset value: 0x00
     * @brief High byte of Z-axis two's-complement measurement data.
     */
    enum class DATAZH_BITS : uint8_t
    {
        DATA_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
    };

    /**
     * @Name: STAT2
     * @Address: 9 (09h)
     * @Serial IF: R
     * @Reset value: 0x00
     * @brief Interrupt status register. INT is set when the absolute sum of
     *        the measured three-axis output exceeds 1600 uT.
     */
    enum class STAT2_BITS : uint8_t
    {
        RESERVED_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit2 | Bit1 | Bit0,
        INT_MASK = Bit3,

        INT_NO_EVENT = BitNone,
        INT_EVENT = Bit3,
    };

    /**
     * @Name: CNTL1
     * @Address: 10 (0Ah)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @brief Operating mode control. Values other than the documented
     *        Stand-By and Single Measurement modes are reserved.
     */
    enum class CNTL1_BITS : uint8_t
    {
        RESERVED_MASK = Bit7 | Bit6 | Bit5 | Bit4,
        MODE_MASK = Bit3 | Bit2 | Bit1 | Bit0,

        MODE_STANDBY = BitNone,
        MODE_SINGLE_MEASUREMENT = Bit0,
    };

    /**
     * @Name: CNTL2
     * @Address: 11 (0Bh)
     * @Serial IF: R/W
     * @Reset value: 0x0C, derived from DREN=1, DRP=1 and SRST=0
     * @brief Data-ready output and soft-reset control. SRST automatically
     *        clears to zero after the POR routine completes.
     */
    enum class CNTL2_BITS : uint8_t
    {
        RESERVED_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit1,
        DREN_MASK = Bit3,
        DRP_MASK = Bit2,
        SRST_MASK = Bit0,

        DREN_DISABLE = BitNone,
        DREN_ENABLE = Bit3,

        DRP_ACTIVE_LOW = BitNone,
        DRP_ACTIVE_HIGH = Bit2,

        SRST_NO_ACTION = BitNone,
        SRST_START_POR = Bit0,
    };

    /**
     * @Name: STR
     * @Address: 12 (0Ch)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @brief Self-test control. Write 0x40 to enable self-test and 0x00 to
     *        disable self-test.
     */
    enum class STR_BITS : uint8_t
    {
        RESERVED_MASK = Bit7 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
        SELF_TEST_MASK = Bit6,

        SELF_TEST_DISABLE = BitNone,
        SELF_TEST_ENABLE = Bit6,
    };

    /**
     * @Name: TEMPL
     * @Address: 28 (1Ch)
     * @Serial IF: R
     * @Reset value: 0x00
     * @brief Low byte of the two's-complement temperature output. IST8310
     *        Datasheet Version 1.2 does not provide a temperature conversion
     *        formula; no temperature unit conversion should be inferred.
     */
    enum class TEMPL_BITS : uint8_t
    {
        DATA_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
    };

    /**
     * @Name: TEMPH
     * @Address: 29 (1Dh)
     * @Serial IF: R
     * @Reset value: 0x00
     * @brief High byte of the two's-complement temperature output. IST8310
     *        Datasheet Version 1.2 does not provide a temperature conversion
     *        formula; no temperature unit conversion should be inferred.
     */
    enum class TEMPH_BITS : uint8_t
    {
        DATA_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
    };

    /**
     * @Name: AVGCNTL
     * @Address: 65 (41h)
     * @Serial IF: R/W
     * @Reset value: TODO - Datasheet Version 1.2 is internally inconsistent
     * @brief Controls internal averaging. For each field, values other than
     *        000, 001, 010, 011 and 100 mean no average.
     *
     * @note The field Default column states 0, while the value descriptions
     *       mark 3'b010 as Default. Confirm the actual reset value by reading
     *       AVGCNTL after hardware reset.
     */
    enum class AVGCNTL_BITS : uint8_t
    {
        RESERVED_MASK = Bit7 | Bit6,
        Y_AVG_MASK = Bit5 | Bit4 | Bit3,
        XZ_AVG_MASK = Bit2 | Bit1 | Bit0,

        Y_AVG_NONE = BitNone,
        Y_AVG_2_TIMES = Bit3,
        Y_AVG_4_TIMES = Bit4,
        Y_AVG_8_TIMES = Bit4 | Bit3,
        Y_AVG_16_TIMES = Bit5,

        XZ_AVG_NONE = BitNone,
        XZ_AVG_2_TIMES = Bit0,
        XZ_AVG_4_TIMES = Bit1,
        XZ_AVG_8_TIMES = Bit1 | Bit0,
        XZ_AVG_16_TIMES = Bit2,
    };

    /**
     * @Name: PDCNTL
     * @Address: 66 (42h)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @brief Controls AMR set/reset pulse duration. Values other than Long
     *        and Normal are documented only for extreme cases.
     */
    enum class PDCNTL_BITS : uint8_t
    {
        PULSE_DURATION_MASK = Bit7 | Bit6,
        RESERVED_MASK = Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,

        PULSE_DURATION_LONG = Bit6,
        PULSE_DURATION_NORMAL = Bit7 | Bit6,
    };
    /************************************************************************************************************************************************/


    /*******************************************************Datasheet Recommended Configuration*******************************************************/
    // IST8310 Datasheet Version 1.2 section 3.1.1 recommendations. These are
    // recommended operating values, not register reset values.
    inline constexpr uint8_t AVGCNTL_LOW_NOISE_RECOMMENDED = 0x24;
    inline constexpr uint8_t PDCNTL_NORMAL_RECOMMENDED = 0xC0;
    /************************************************************************************************************************************************/


    /*************************************************************Timing Constants*******************************************************************/
    // Datasheet: POR completes within 50 ms.
    inline constexpr uint32_t POR_RESET_WAIT_MS = 50u;

    // Datasheet: low-noise configuration requires at least 6 ms between
    // measurements.
    inline constexpr uint32_t SINGLE_MEASUREMENT_MIN_WAIT_MS = 6u;

    // Driver policy constants, not register-defined datasheet values.
    inline constexpr uint32_t I2C_TRANSACTION_TIMEOUT_MS = 20u;
    inline constexpr uint32_t SOFT_RESET_TIMEOUT_MS = 100u;
    inline constexpr uint32_t RESET_POLL_INTERVAL_MS = 5u;
    inline constexpr uint32_t SINGLE_MEASUREMENT_TIMEOUT_MS = 20u;
    inline constexpr uint32_t DATA_READY_POLL_INTERVAL_MS = 1u;
    /************************************************************************************************************************************************/


    /**********************************************************Sensitivity Constants*****************************************************************/
    // IST8310 Datasheet Version 1.2 section 4.4.
    inline constexpr float DATASHEET_DEFAULT_LSB_PER_UT = 3.3F;
    inline constexpr float DATASHEET_DEFAULT_UT_PER_LSB =
        1.0F / DATASHEET_DEFAULT_LSB_PER_UT;
    inline constexpr float DATASHEET_RESOLUTION_UT_PER_LSB = 0.3F;
    inline constexpr float DEFAULT_UT_PER_LSB = DATASHEET_DEFAULT_UT_PER_LSB;
    /************************************************************************************************************************************************/


    /********************************************************Burst Layout Constants******************************************************************/
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
    /************************************************************************************************************************************************/


    /************************************************************PX4 Undocumented********************************************************************/
    namespace PX4_Undocumented
    {
        // These values appear in the PX4 IST8310 reference driver but are not
        // described by IST8310 Datasheet Version 1.2. They are not part of
        // RegsAdd::IST8310, are not enabled by the default driver and must not
        // be written before dedicated hardware validation.
        inline constexpr uint8_t CNTL3_ADDRESS = 0x0D;
        inline constexpr uint8_t CNTL3_PX4_16BIT_CONFIG = 0x70;

        // 1320 LSB/Gauss is a PX4 reference value, not the datasheet default
        // sensitivity.
        inline constexpr float PX4_16BIT_LSB_PER_GAUSS = 1320.0F;
        inline constexpr float PX4_16BIT_UT_PER_LSB =
            100.0F / PX4_16BIT_LSB_PER_GAUSS;
    }
    /************************************************************************************************************************************************/


    /*******************************************Bit Operators for Register Field enum class Types****************************************************/
    template<typename E>
    constexpr bool _disableBitOperators = false;

    // Register addresses are not bit fields and must never participate in the
    // field bitwise operators below.
    template<>
    constexpr bool _disableBitOperators<RegsAdd::IST8310> = true;

    template<typename E>
    constexpr std::enable_if_t<std::is_enum_v<E> && !_disableBitOperators<E>, E>
    operator&(E lhs, E rhs)
    {
        using U = std::underlying_type_t<E>;
        return static_cast<E>(static_cast<U>(lhs) & static_cast<U>(rhs));
    }

    template<typename E>
    constexpr std::enable_if_t<std::is_enum_v<E> && !_disableBitOperators<E>, E>
    operator|(E lhs, E rhs)
    {
        using U = std::underlying_type_t<E>;
        return static_cast<E>(static_cast<U>(lhs) | static_cast<U>(rhs));
    }

    template<typename E>
    constexpr std::enable_if_t<std::is_enum_v<E> && !_disableBitOperators<E>, E>
    operator~(E rhs)
    {
        using U = std::underlying_type_t<E>;
        return static_cast<E>(~static_cast<U>(rhs));
    }

    template<typename E>
    constexpr std::enable_if_t<std::is_enum_v<E> && !_disableBitOperators<E>, E&>
    operator&=(E& lhs, E rhs)
    {
        using U = std::underlying_type_t<E>;
        lhs = static_cast<E>(static_cast<U>(lhs) & static_cast<U>(rhs));
        return lhs;
    }

    template<typename E>
    constexpr std::enable_if_t<std::is_enum_v<E> && !_disableBitOperators<E>, E&>
    operator|=(E& lhs, E rhs)
    {
        using U = std::underlying_type_t<E>;
        lhs = static_cast<E>(static_cast<U>(lhs) | static_cast<U>(rhs));
        return lhs;
    }
    /************************************************************************************************************************************************/
}
