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
    // IST8310 Datasheet Version 1.2 Section 6.1.1: CAD1/CAD0 地址选择表（7-bit）。
    inline constexpr uint8_t I2C_ADDRESS_CAD1_VSS_CAD0_VSS_7BIT = 0x0C;
    inline constexpr uint8_t I2C_ADDRESS_CAD1_VSS_CAD0_VDD_7BIT = 0x0D;
    inline constexpr uint8_t I2C_ADDRESS_CAD1_VDD_CAD0_VSS_7BIT = 0x0E;
    inline constexpr uint8_t I2C_ADDRESS_CAD1_VDD_CAD0_VDD_7BIT = 0x0F;

    // 手册 Section 6.1.1: CAD1 和 CAD0 浮空时，I2C 7-bit 地址默认 0x0E。
    // 这是芯片手册唯一指定的默认地址，与 CAD 引脚悬空时的硬件行为一致。
    inline constexpr uint8_t DATASHEET_DEFAULT_I2C_ADDRESS_7BIT =
        I2C_ADDRESS_CAD1_VDD_CAD0_VSS_7BIT;

    // 当前板级 CAD1/CAD0 实际连接（需硬件确认，当前假设 CAD1=VSS, CAD0=VSS）。
    // 在未确认板级 CAD 引脚连接前，此地址仅作为候选项，不作为驱动默认值。
    inline constexpr uint8_t BOARD_CAD1_VSS_CAD0_VSS_I2C_ADDRESS_7BIT =
        I2C_ADDRESS_CAD1_VSS_CAD0_VSS_7BIT;

    // 驱动构造函数默认参数。指向手册默认地址（CAD 浮空 = 0x0E）。
    // 若板级 CAD 引脚实际连接与手册默认不一致，调用方须显式传入正确地址。
    inline constexpr uint8_t I2C_ADDRESS_DEFAULT_7BIT =
        DATASHEET_DEFAULT_I2C_ADDRESS_7BIT;

    inline constexpr uint8_t I2C_ADDRESS_MIN_7BIT = I2C_ADDRESS_CAD1_VSS_CAD0_VSS_7BIT;
    inline constexpr uint8_t I2C_ADDRESS_MAX_7BIT = I2C_ADDRESS_CAD1_VDD_CAD0_VDD_7BIT;
    inline constexpr uint8_t HAL_I2C_ADDRESS_SHIFT = 1u;
    inline constexpr uint8_t DEVICE_ID = 0x10; // 手册 Section 6.4.2: WAI 复位值 0x10
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
     * @Reset value: 手册 Section 6.4.7 明确记载 DREN=1, DRP=1, SRST=0；
     *               Reserved[7:4] 和 Reserved[1] 未给出默认值，不应推导。
     *               仅按手册公开具名位计算，复位后有效控制位约为 0x0C；
     *               若需要全字节复位值，应上电后硬件读取确认。
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
     * @Name: STR (手册 Section 6.4.8: Self-Test Register)
     * @Address: 12 (0Ch)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @brief Self-test control. Write 0x40 to enable self-test and 0x00 to
     *        disable self-test. 手册 Section 3.1.3: 使能后三轴输出极性反转，
     *        前后绝对值相同则芯片工作正常。
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
     * @Name: AVGCNTL（手册 Section 6.4.10: Average Control Register）
     * @Address: 65 (41h)
     * @Serial IF: R/W
     * @Reset value: 未可靠确定 — 手册 Section 6.4.10 内部矛盾：
     *              Default 列标注 0（即不平均），但值描述中 3'b010（平均 4 次）
     *              标记为 "(Default)"。此矛盾两种解释均可引用手册原文，
     *              精确复位值需上电后硬件读取 AVGCNTL 确认。
     * @brief Controls internal averaging. For each field, values other than
     *        000, 001, 010, 011 and 100 mean no average.
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
     * @Name: PDCNTL（手册 Section 6.4.11: Pulse Duration Control Register）
     * @Address: 66 (42h)
     * @Serial IF: R/W
     * @Reset value: 0x00（手册 Default 列：Pulse duration=0, Reserved=0）
     * @brief Controls AMR set/reset pulse duration.
     *        手册 Section 3.1.1 推荐性能优化值 0xC0（Normal）。
     *        2'b01=Long, 2'b11=Normal, Others=仅极端情况。
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
    // 手册 Section 3.1.1 推荐值（不是寄存器复位值）：
    //   PDCNTL(0x42) = 0xC0 — 性能优化，脉冲持续时间 Normal（手册 Section 6.4.11: 2'b11）。
    //   AVGCNTL(0x41) = 0x24 — 低噪声性能，Y/XZ 各平均 16 次（手册 Section 6.4.10）。
    //   低噪声配置下两次测量最小间隔 6 ms（最高 ODR ≈ 166 Hz）。
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
    // 手册 Section 4.4 Magnetic Sensor Specifications（手册公开 14-bit 输出规格）：
    //   Resolution  RESO = 0.3  uT/LSB
    //   Sensitivity SEN  = 3.3  LSB/uT
    //
    // 注意：PX4 参考驱动使用 CNTL3 使能 16-bit 输出后的灵敏度为 1320 LSB/Gauss
    // （= 13.2 LSB/uT，约 0.0758 uT/LSB），与手册公开 14-bit 输出规格不同。
    // CNTL3 不是手册公开寄存器（见下方 PX4_Undocumented），默认驱动不使用。
    //
    // 以下常量基于手册 Section 4.4 公开 14-bit 输出规格。
    inline constexpr float DATASHEET_DEFAULT_LSB_PER_UT = 3.3F;
    inline constexpr float DATASHEET_DEFAULT_UT_PER_LSB =
        1.0F / DATASHEET_DEFAULT_LSB_PER_UT; // ≈ 0.303 uT/LSB，对应手册 RESO = 0.3 uT/LSB
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
        // ==================================================================
        // 以下定义来自 PX4 参考驱动（Reference/ist8310/iSentek_IST8310_registers.hpp）
        // 和 PX4 IST8310.cpp，但不存在于 IST8310 Datasheet Version 1.2 的
        // "Customer Defined Registers" (Section 6.4) 中。
        //
        // 这些寄存器/常量的地址、位域和数值系 PX4 工程实践，非手册公开内容。
        // 默认驱动不启用，不得在未完成专用硬件验证前写入。
        // ==================================================================

        // CNTL3 (0x0D) — 不存在于手册 Section 6.4。
        // PX4 参考中用于使能 X/Y/Z 三轴 16-bit 输出，对应灵敏度 1320 LSB/Gauss。
        // 手册公开输出规格为 14-bit（Section 4.4: Resolution = 0.3 uT/LSB）。
        inline constexpr uint8_t CNTL3_ADDRESS = 0x0D;
        inline constexpr uint8_t CNTL3_PX4_16BIT_CONFIG = 0x70;

        // 1320 LSB/Gauss — PX4 16-bit 模式灵敏度，非手册 Section 4.4 公开规格值。
        // 1 Gauss = 100 uT，故 1320 LSB/Gauss = 13.2 LSB/uT ≈ 0.0758 uT/LSB。
        // 手册公开 14-bit 输出规格：SEN = 3.3 LSB/uT ≈ 0.303 uT/LSB。
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
