//
// Created by Gray on 2026/1/14.
//

#pragma once

#include <cstdint>
#include <type_traits>

namespace ICM42688P
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


    /***********************************************************Registers Address********************************************************************/
    namespace RegAdd
    {
        enum class BANK0:uint8_t
        {
            DEVICE_CONFIG = 0x11,

            INT_CONFIG = 0x14,

            FIFO_CONFIG = 0x16,

            TEMP_DATA1 = 0x1D,
            TEMP_DATA0 = 0x1E,

            ACCEL_DATA_X1 = 0x1F,
            ACCEL_DATA_X0 = 0x20,
            ACCEL_DATA_Y1 = 0x21,
            ACCEL_DATA_Y0 = 0x22,
            ACCEL_DATA_Z1 = 0x23,
            ACCEL_DATA_Z0 = 0x24,

            GYRO_DATA_X1 = 0x25,
            GYRO_DATA_X0 = 0x26,
            GYRO_DATA_Y1 = 0x27,
            GYRO_DATA_Y0 = 0x28,
            GYRO_DATA_Z1 = 0x29,
            GYRO_DATA_Z0 = 0x2A,

            TMST_FSYNCH = 0x2B,
            TMST_FSYNCL = 0x2C,

            INT_STATUS = 0x2D,

            FIFO_COUNTH = 0x2E,
            FIFO_COUNTL = 0x2F,
            FIFO_DATA = 0x30,

            SIGNAL_PATH_RESET = 0x4B,
            INTF_CONFIG0 = 0x4C,
            INTF_CONFIG1 = 0x4D,
            PWR_MGMT0 = 0x4E,

            GYRO_CONFIG0 = 0x4F,
            ACCEL_CONFIG0 = 0x50,
            GYRO_CONFIG1 = 0x51,
            GYRO_ACCEL_CONFIG0 = 0x52,
            ACCEL_CONFIG1 = 0x53,

            TMST_CONFIG = 0x54,

            FIFO_CONFIG1 = 0x5F,
            FIFO_CONFIG2 = 0x60,
            FIFO_CONFIG3 = 0x61,

            FSYNC_CONFIG = 0x62,

            INT_CONFIG0 = 0x63,
            INT_CONFIG1 = 0x64,

            INT_SOURCE0 = 0x65,
            INT_SOURCE1 = 0x66,
            INT_SOURCE3 = 0x68,
            INT_SOURCE4 = 0x69,

            SELF_TEST_CONFIG = 0x70,

            WHO_AM_I = 0x75,
            REG_BANK_SEL = 0x76,
        };

        enum class BANK1:uint8_t
        {
            SENSOR_CONFIG0 = 0x03,

            GYRO_CONFIG_STATIC2 = 0x0B,
            GYRO_CONFIG_STATIC3 = 0x0C,
            GYRO_CONFIG_STATIC4 = 0x0D,
            GYRO_CONFIG_STATIC5 = 0x0E,
            GYRO_CONFIG_STATIC6 = 0x0F,
            GYRO_CONFIG_STATIC7 = 0x10,
            GYRO_CONFIG_STATIC8 = 0x11,
            GYRO_CONFIG_STATIC9 = 0x12,
            GYRO_CONFIG_STATIC10 = 0x13,

            INTF_CONFIG5 = 0x7B,
        };

        enum class BANK2:uint8_t
        {
            ACCEL_CONFIG_STATIC2 = 0x03,
            ACCEL_CONFIG_STATIC3 = 0x04,
            ACCEL_CONFIG_STATIC4 = 0x05,
        };

        enum class BANK4:uint8_t
        {
            OFFSET_USER0 = 0x77,
            OFFSET_USER1 = 0x78,
            OFFSET_USER2 = 0x79,
            OFFSET_USER3 = 0x7A,
            OFFSET_USER4 = 0x7B,
            OFFSET_USER5 = 0x7C,
            OFFSET_USER6 = 0x7D,
            OFFSET_USER7 = 0x7E,
            OFFSET_USER8 = 0x7F,
        };
    }
    /************************************************************************************************************************************************/


    /************************************************************BANK0 Registers Describe************************************************************/
    /**
     * @Name: DEVICE_CONFIG
     * @Address: 17 (11h)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class DEVICE_CONFIG_BITS:uint8_t
    {
        SPI_MODE = Bit4, //SPI mode selection 0: Mode 0 and Mode 3 (default) 1: Mode 1 and Mode 2
        SOFT_RESET_CONFIG = Bit0, /*Software reset configuration 0: Normal (default) 1: Enable reset
                                    After writing 1 to this bitfield, wait 1ms for soft reset to be effective, before attempting any other register access*/
    };

    /**
     * @Name: INT_CONFIG
     * @Address: 20 (14h)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class INT_CONFIG_BITS:uint8_t
    {
        INT2_MODE = Bit5, //INT2 interrupt mode 0: Pulsed mode 1: Latched mode
        INT2_DRIVE_CIRCUIT = Bit4, //INT2 drive circuit 0: Open drain 1: Push pull
        INT2_POLARITY = Bit3, //INT2 interrupt polarity 0: Active low (default) 1: Active high
        INT1_MODE = Bit2, //INT1 interrupt mode 0: Pulsed mode 1: Latched mode
        INT1_DRIVE_CIRCUIT = Bit1, //INT1 drive circuit 0: Open drain 1: Push pull
        INT1_POLARITY = Bit0, //INT1 interrupt polarity 0: Active low (default) 1: Active high
    };

    /**
     * @Name: FIFO_CONFIG
     * @Address: 22 (16h)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class FIFO_CONFIG_BITS:uint8_t
    {
        FIFO_MODE_MASK = Bit7 | Bit6,
        FIFO_MODE_BYPASS = BitNone, //00: Bypass Mode (default)
        FIFO_MODE_STREAM_TO_FIFO = Bit6, //01: Stream-to-FIFO Mode
        FIFO_MODE_STOP_ON_FULL = Bit7, //10: STOP-on-FULL Mode
                                       //11: STOP-on-FULL Mode
    };

    /**
     * @Name: TEMP_DATA1
     * @Address: 29 (1Dh)
     * @Serial IF: SYNCR
     * @Reset value: 0x80
     * @Clock Domain: SCLK_UI
     */
    enum class TEMP_DATA1_BITS:uint8_t
    {
        TEMP_DATA_15_8_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Upper byte of temperature data
    };

    /**
     * @Name: TEMP_DATA0
     * @Address: 30 (1Eh)
     * @Serial IF: SYNCR
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     * @brief Temperature sensor register data TEMP_DATA is updated with new data at max(Accelerometer ODR, Gyroscope ODR).
     *        Temperature data value from the sensor data registers can be converted to degrees centigrade by using the following formula:
     *        Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25
     *        Temperature data stored in FIFO is an 8-bit quantity, FIFO_TEMP_DATA.  It can be converted to degrees centigrade by using the following formula:
     *        Temperature in Degrees Centigrade = (FIFO_TEMP_DATA / 2.07) + 25
     */
    enum class TEMP_DATA0_BITS:uint8_t
    {
        TEMP_DATA_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Lower byte of temperature data
    };

    /**
     * @Name: ACCEL_DATA_X1
     * @Address: 31 (1Fh)
     * @Serial IF: SYNCR
     * @Reset value: 0x80
     * @Clock Domain: SCLK_UI
     */
    enum class ACCEL_DATA_X1_BITS:uint8_t
    {
        ACCEL_DATA_15_8_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Upper byte of Accel X-axis data
    };

    /**
     * @Name: ACCEL_DATA_X0
     * @Address: 32 (20h)
     * @Serial IF: SYNCR
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class ACCEL_DATA_X0_BITS:uint8_t
    {
        ACCEL_DATA_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Lower byte of Accel X-axis data
    };

    /**
     * @Name: ACCEL_DATA_Y1
     * @Address: 33 (21h)
     * @Serial IF: SYNCR
     * @Reset value: 0x80
     * @Clock Domain: SCLK_UI
     */
    enum class ACCEL_DATA_Y1_BITS:uint8_t
    {
        ACCEL_DATA_15_8_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Upper byte of Accel Y-axis data
    };

    /**
     * @Name: ACCEL_DATA_Y0
     * @Address: 34 (22h)
     * @Serial IF: SYNCR
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class ACCEL_DATA_Y0_BITS:uint8_t
    {
        ACCEL_DATA_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Lower byte of Accel Y-axis data
    };

    /**
     * @Name: ACCEL_DATA_Z1
     * @Address: 35 (23h)
     * @Serial IF: SYNCR
     * @Reset value: 0x80
     * @Clock Domain: SCLK_UI
     */
    enum class ACCEL_DATA_Z1_BITS:uint8_t
    {
        ACCEL_DATA_Z_15_8_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Upper byte of Accel Z-axis data
    };

    /**
     * @Name: ACCEL_DATA_Z0
     * @Address: 36 (24h)
     * @Serial IF: SYNCR
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class ACCEL_DATA_Z0_BITS:uint8_t
    {
        ACCEL_DATA_Z_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Lower byte of Accel Z-axis data
    };

    /**
     * @Name: GYRO_DATA_X1
     * @Address: 37 (25h)
     * @Serial IF: SYNCR
     * @Reset value: 0x80
     * @Clock Domain: SCLK_UI
     */
    enum class GYRO_DATA_X1_BITS:uint8_t
    {
        GYRO_DATA_15_8_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Upper byte of Gyro X-axis data
    };

    /**
     * @Name: GYRO_DATA_X0
     * @Address: 38 (26h)
     * @Serial IF: SYNCR
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class GYRO_DATA_X0_BITS:uint8_t
    {
        GYRO_DATA_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Lower byte of Gyro X-axis data
    };

    /**
     * @Name: GYRO_DATA_Y1
     * @Address: 39 (27h)
     * @Serial IF: SYNCR
     * @Reset value: 0x80
     * @Clock Domain: SCLK_UI
     */
    enum class GYRO_DATA_Y1_BITS:uint8_t
    {
        GYRO_DATA_15_8_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Upper byte of Gyro Y-axis data
    };

    /**
     * @Name: GYRO_DATA_Y0
     * @Address: 40 (28h)
     * @Serial IF: SYNCR
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class GYRO_DATA_Y0_BITS:uint8_t
    {
        GYRO_DATA_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Lower byte of Gyro Y-axis data
    };

    /**
     * @Name: GYRO_DATA_Z1
     * @Address: 41 (29h)
     * @Serial IF: SYNCR
     * @Reset value: 0x80
     * @Clock Domain: SCLK_UI
     */
    enum class GYRO_DATA_Z1_BITS:uint8_t
    {
        GYRO_DATA_Z_15_8_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Upper byte of Gyro Z-axis data
    };

    /**
     * @Name: GYRO_DATA_Z0
     * @Address: 42 (2Ah)
     * @Serial IF: SYNCR
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class GYRO_DATA_Z0_BITS:uint8_t
    {
        GYRO_DATA_Z_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Lower byte of Gyro Z-axis data
    };

    /**
     * @Name: TMST_FSYNCH
     * @Address: 43 (2Bh)
     * @Serial IF: SYNCR
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class TMST_FSYNCH_BITS:uint8_t
    {
        TMST_FSYNC_DATA_UI_15_8_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, /*Stores the upper byte of the time delta from the rising edge of FSYNC to
                                                                                                the latest ODR until the UI Interface reads the FSYNC tag in the status
                                                                                                register*/
    };

    /**
     * @Name: TMST_FSYNCL
     * @Address: 44 (2Ch)
     * @Serial IF: SYNCR
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class TMST_FSYNCL_BITS:uint8_t
    {
        TMST_FSYNC_DATA_UI_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, /*Stores the lower byte of the time delta from the rising edge of FSYNC to
                                                                                                the latest ODR until the UI Interface reads the FSYNC tag in the status
                                                                                                register*/
    };

    /**
     * @Name: INT_STATUS
     * @Address: 45 (2Dh)
     * @Serial IF: R/C
     * @Reset value: 0x10
     * @Clock Domain: SCLK_UI
     */
    enum class INT_STATUS_BITS:uint8_t
    {
        UI_FSYNC_INT = Bit6, //This bit automatically sets to 1 when a UI FSYNC interrupt is generated. The bit clears to 0 after the register has been read.
        PLL_RDY_INT = Bit5, //This bit automatically sets to 1 when a PLL Ready interrupt is generated. The bit clears to 0 after the register has been read.
        RESET_DONE_INT = Bit4, //This bit automatically sets to 1 when software reset is complete. The bit clears to 0 after the register has been read.
        DATA_RDY_INT = Bit3, //This bit automatically sets to 1 when a Data Ready interrupt is generated. The bit clears to 0 after the register has been read.
        FIFO_THS_INT = Bit2, //This bit automatically sets to 1 when the FIFO buffer reaches the threshold value. The bit clears to 0 after the register has been read.
        FIFO_FULL_INT = Bit1, //This bit automatically sets to 1 when the FIFO buffer is full.  The bit clears to 0 after the register has been read.
        AGC_RDY_INT = Bit0, //This bit automatically sets to 1 when an AGC Ready interrupt is generated. The bit clears to 0 after the register has been read.
    };

    /**
     * @Name: FIFO_COUNTH
     * @Address: 46 (2Eh)
     * @Serial IF: R
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class FIFO_COUNTH_BITS:uint8_t
    {
        FIFO_COUNT_15_8_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, /*High Bits, count indicates the number of records or bytes available in FIFO
                                                                                        according to FIFO_COUNT_REC setting.
                                                                                        Note:  Must read FIFO_COUNTL to latch new data for both FIFO_COUNTH and FIFO_COUNTL.*/
    };

    /**
     * @Name: FIFO_COUNTL
     * @Address: 47 (2Fh)
     * @Serial IF: R
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class FIFO_COUNTL_BITS:uint8_t
    {
        FIFO_COUNT_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, /*Low Bits, count indicates the number of records or bytes available in FIFO
                                                                                       according to FIFO_COUNT_REC setting.
                                                                                       Reading this byte latches the data for both FIFO_COUNTH, and FIFO_COUNTL.*/
    };

    /**
     * @Name: FIFO_DATA
     * @Address: 48 (30h)
     * @Serial IF: R
     * @Reset value: 0xFF
     * @Clock Domain: SCLK_UI
     */
    enum class FIFO_DATA_BITS:uint8_t
    {
        FIFO_DATA_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //FIFO data port
    };

    /**
     * @Name: SIGNAL_PATH_RESET
     * @Address: 75 (4Bh)
     * @Serial IF: W/C
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class SIGNAL_PATH_RESET_BITS:uint8_t
    {
        DMP_INIT_EN = Bit6, //When this bit is set to 1, the DMP is enabled
        DMP_MEM_RESET_EN = Bit5, //When this bit is set to 1, the DMP memory is reset
        ABORT_AND_RESET = Bit3, //When this bit is set to 1, the signal path is reset by restarting the ODR counter and signal path controls
        TMST_STROBE = Bit2, //When this bit is set to 1, the time stamp counter is latched into the time stamp register. This is a write on clear bit.
        FIFO_FLUSH = Bit1, //When set to 1, FIFO will get flushed.
    };

    /**
     * @Name: INTF_CONFIG0
     * @Address: 76 (4Ch)
     * @Serial IF: R/W
     * @Reset value: 0x30
     * @Clock Domain: SCLK_UI
     * @brief Invalid Data Generation:  FIFO/Sense Registers may contain invalid data under the following conditions:
     *        a) From power on reset to first ODR sample of any sensor (accel, gyro, temp sensor)
     *        b) When any sensor is disabled (accel, gyro, temp sensor)
     *        c) When accel and gyro are enabled with different ODRs.  In this case, the sensor with lower ODR will generate invalid samples when it has no new data.
     *        Invalid data can take special values or can hold last valid sample received.  For -32768 to be used as a flag for invalid accel/gyro
     *        samples, the valid accel/gyro sample range is limited in such case as well.  Bit 7 of INTF_CONFIG0 controls what values invalid (and valid) samples can take as shown above.
     */
    enum class INTF_CONFIG0_BITS:uint8_t
    {
        FIFO_HOLD_LAST_DATA_EN = Bit7, //This bit selects the treatment of invalid samples. See Invalid Data Generation note below this register description.
        FIFO_COUNT_REC = Bit6, /*0: FIFO count is reported in bytes
                                 1: FIFO count is reported in records (1 record = 16 bytes for header + gyro + accel + temp sensor data + time stamp, or 8 bytes for header + gyro/accel +
                                 temp sensor data, or 20 bytes for header + gyro + accel + temp sensor data + time stamp + 20-bit extension data)*/
        FIFO_COUNT_ENDIAN = Bit5, //0: FIFO count is reported in Little Endian format 1: FIFO count is reported in Big Endian format (default)
        SENSOR_DATA_ENDIAN = Bit4, //0: Sensor data is reported in Little Endian format 1: Sensor data is reported in Big Endian format (default)

        UI_SIFS_CFG_MASK = Bit1 | Bit0,
                                        //0x: Reserved
        UI_SIFS_CFG_DISABLE_SPI = Bit1, //10: Disable SPI
        UI_SIFS_CFG_DISABLE_I2C = Bit1 | Bit0, //11: Disable I2C
    };

    /**
     * @Name: INTF_CONFIG1
     * @Address: 77 (4Dh)
     * @Serial IF: R/W
     * @Reset value: 0x91
     * @Clock Domain: SCLK_UI
     */
    enum class INTF_CONFIG1_BITS:uint8_t
    {
        ACCEL_LP_CLK_SEL = Bit3, //0: Accelerometer LP mode uses Wake Up oscillator clock 1: Accelerometer LP mode uses RC oscillator clock
        RTC_MODE = Bit2, //0: No input RTC clock is required 1: RTC clock input is required

        CLKSEL_MASK = Bit1 | Bit0,
        CLKSEL_MASK_INTERNAL_RC = BitNone, //00: Always select internal RC oscillator
        CLKSEL_MASK_PLL = Bit0, //01: Select PLL when available, else select RC oscillator (default)
                                //10: Reserved
        CLKSEL_MASK_DISABLE_ALL_CLOCKS = Bit1 | Bit0, //11: Disable all clocks
    };

    /**
     * @Name: PWR_MGMT0
     * @Address: 78 (4Eh)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class PWR_MGMT0_BITS:uint8_t
    {
        TEMP_DIS = Bit5, //0: Temperature sensor is enabled (default) 1: Temperature sensor is disabled
        IDLE = Bit4, /*If this bit is set to 1, the RC oscillator is powered on even if Accel and Gyro are powered off.
                       Nominally this bit is set to 0, so when Accel and Gyro are powered off, the chip will go to OFF state, since the RC oscillator will also be powered off*/

        GYRO_MODE_MASK = Bit3 | Bit2, /*Gyroscope needs to be kept ON for a minimum of 45ms. When transitioning
                                        from OFF to any of the other modes, do not issue any register writes for 200µs.*/
        GYRO_MODE_TURNS_GYRO_OFF = BitNone, //00: Turns gyroscope off (default)
        GYRO_MODE_GYRO_STANDBY = Bit2, //01: Places gyroscope in Standby Mode
                                       //10: Reserved
        GYRO_MODE_GYRO_LOW_NOISE = Bit3 | Bit2, //11: Places gyroscope in Low Noise (LN) Mode

        ACCEL_MODE_MASK = Bit1 | Bit0, /*When transitioning from OFF to any of the other modes, do not issue any register writes for 200µs.*/
        ACCEL_MODE_TURNS_ACC_OFF = BitNone, //00: Turns accelerometer off (default)
                                         //01: Turns accelerometer off
        ACCEL_MODE_ACC_LOW_POWER = Bit1, //10: Places accelerometer in Low Power (LP) Mode
        ACCEL_MODE_ACC_LOW_NOISE = Bit1 | Bit0, //11: Places accelerometer in Low Noise (LN) Mode
    };

    /**
     * @Name: GYRO_CONFIG0
     * @Address: 79 (4Fh)
     * @Serial IF: R/W
     * @Reset value: 0x06
     * @Clock Domain: SCLK_UI
     */
    enum class GYRO_CONFIG0_BITS:uint8_t
    {
        GYRO_FS_SEL_MASK = Bit7 | Bit6 | Bit5, //Full scale select for gyroscope UI interface output
        GYRO_FS_SEL_2000DPS = BitNone, //000: ±2000dps (default)
        GYRO_FS_SEL_1000DPS = Bit5, //001: ±1000dps
        GYRO_FS_SEL_500DPS = Bit6, //010: ±500dps
        GYRO_FS_SEL_250DPS = Bit6 | Bit5, //011: ±250dps
        GYRO_FS_SEL_125DPS = Bit7, //100: ±125dps
        GYRO_FS_SEL_62_5DPS = Bit7 | Bit5, //101: ±62.5dps
        GYRO_FS_SEL_31_25DPS = Bit7 | Bit6, //110: ±31.25dps
        GYRO_FS_SEL_15_625DPS = Bit7 | Bit6 | Bit5, //111: ±15.625dps

        GYRO_ODR_MASK = Bit3 | Bit2 | Bit1 | Bit0, //Gyroscope ODR selection for UI interface output
                               //0000: Reserved
        GYRO_ODR_32KHZ = Bit0, //0001: 32kHz
        GYRO_ODR_16KHZ = Bit1, //0010: 16kHz
        GYRO_ODR_8KHZ = Bit1 | Bit0, //0011: 8kHz
        GYRO_ODR_4KHZ = Bit2, //0100: 4kHz
        GYRO_ODR_2KHZ = Bit2 | Bit0, //0101: 2kHz
        GYRO_ODR_1KHZ = Bit2 | Bit1, //0110: 1kHz (default)
        GYRO_ODR_200HZ = Bit2 | Bit1 | Bit0, //0111: 200Hz
        GYRO_ODR_100HZ = Bit3, //1000: 100Hz
        GYRO_ODR_50HZ = Bit3 | Bit0, //1001: 50Hz
        GYRO_ODR_25HZ = Bit3 | Bit1, //1010: 25Hz
        GYRO_ODR_12_5HZ = Bit3 | Bit1 | Bit0, //1011: 12.5Hz
                                              //1100: Reserved
                                              //1101: Reserved
                                              //1110: Reserved
        GYRO_ODR_500HZ = Bit3 | Bit2 | Bit1 | Bit0, //1111: 500Hz
    };

    /**
     * @Name: ACCEL_CONFIG0
     * @Address: 80 (50h)
     * @Serial IF: R/W
     * @Reset value: 0x06
     * @Clock Domain: SCLK_UI
     */
    enum class ACCEL_CONFIG0_BITS:uint8_t
    {
        ACCEL_FS_SEL_MASK = Bit7 | Bit6 | Bit5, //Full scale select for accelerometer UI interface output
        ACCEL_FS_SEL_16G = BitNone, //000: ±16g (default)
        ACCEL_FS_SEL_8G = Bit5, //001: ±8g
        ACCEL_FS_SEL_4G = Bit6, //010: ±4g
        ACCEL_FS_SEL_2G = Bit6 | Bit5, //011: ±2g
                                       //100: Reserved
                                       //101: Reserved
                                       //110: Reserved
                                       //111: Reserved

        ACCEL_ODR_MASK = Bit3 | Bit2 | Bit1 | Bit0, //Accelerometer ODR selection for UI interface output
                                //0000: Reserved
        ACCEL_ODR_32KHZ = Bit0, //0001: 32kHz (LN mode)
        ACCEL_ODR_16KHZ = Bit1, //0010: 16kHz (LN mode)
        ACCEL_ODR_8KHZ = Bit1 | Bit0, //0011: 8kHz (LN mode)
        ACCEL_ODR_4KHZ = Bit2, //0100: 4kHz (LN mode)
        ACCEL_ODR_2KHZ = Bit2 | Bit0, //0101: 2kHz (LN mode)
        ACCEL_ODR_1KHZ = Bit2 | Bit1, //0110: 1kHz (LN mode) (default)
        ACCEL_ODR_200HZ = Bit2 | Bit1 | Bit0, //0111: 200Hz (LP or LN mode)
        ACCEL_ODR_100HZ = Bit3, //1000: 100Hz (LP or LN mode)
        ACCEL_ODR_50HZ = Bit3 | Bit0, //1001: 50Hz (LP or LN mode)
        ACCEL_ODR_25HZ = Bit3 | Bit1, //1010: 25Hz (LP or LN mode)
        ACCEL_ODR_12_5HZ = Bit3 | Bit1 | Bit0, //1011: 12.5Hz (LP or LN mode)
        ACCEL_ODR_6_25HZ = Bit3 | Bit2, //1100: 6.25Hz (LP mode)
        ACCEL_ODR_3_125HZ = Bit3 | Bit2 | Bit0, //1101: 3.125Hz (LP mode)
        ACCEL_ODR_1_5625HZ = Bit3 | Bit2 | Bit1, //1110: 1.5625Hz (LP mode)
        ACCEL_ODR_500HZ = Bit3 | Bit2 | Bit1 | Bit0, //1111: 500Hz (LP or LN mode)
    };

    /**
     * @Name: GYRO_CONFIG1
     * @Address: 81 (51h)
     * @Serial IF: R/W
     * @Reset value: 0x16
     * @Clock Domain: SCLK_UI
     */
    enum class GYRO_CONFIG1_BITS:uint8_t
    {
        TEMP_FILT_BW_MASK = Bit7 | Bit6 | Bit5, //Sets the bandwidth of the temperature signal DLPF
        TEMP_FILT_BW_4000HZ = BitNone, //000: DLPF BW = 4000Hz; DLPF Latency = 0.125ms (default)
        TEMP_FILT_BW_170HZ = Bit5, //001: DLPF BW = 170Hz; DLPF Latency = 1ms
        TEMP_FILT_BW_82HZ = Bit6, //010: DLPF BW = 82Hz; DLPF Latency = 2ms
        TEMP_FILT_BW_40HZ = Bit6 | Bit5, //011: DLPF BW = 40Hz; DLPF Latency = 4ms
        TEMP_FILT_BW_20HZ = Bit7, //100: DLPF BW = 20Hz; DLPF Latency = 8ms
        TEMP_FILT_BW_10HZ = Bit7 | Bit5, //101: DLPF BW = 10Hz; DLPF Latency = 16ms
        TEMP_FILT_BW_5HZ = Bit7 | Bit6, //110: DLPF BW = 5Hz; DLPF Latency = 32ms
                                        //111: DLPF BW = 5Hz; DLPF Latency = 32ms

        GYRO_UI_FILT_ORD_MASK = Bit3 | Bit2, //Selects order of GYRO UI filter
        GYRO_UI_FILT_ORD_1 = BitNone, //00: 1st Order
        GYRO_UI_FILT_ORD_2 = Bit2, //01: 2nd Order
        GYRO_UI_FILT_ORD_3 = Bit3, //10: 3rd Order
                                   //11: Reserved

        GYRO_DEC2_M2_ORD_MASK = Bit1 | Bit0,
                                   //00: Reserved
                                   //01: Reserved
        GYRO_DEC2_M2_ORD_3 = Bit1, //10: 3rd Order
                                   //11: Reserved
    };

    /**
     * @Name: GYRO_ACCEL_CONFIG0
     * @Address: 82 (52h)
     * @Serial IF: R/W
     * @Reset value: 0x11
     * @Clock Domain: SCLK_UI
     */
    enum class GYRO_ACCEL_CONFIG0_BITS:uint8_t
    {
                                                           /*LP Mode: 0 Reserved
                                                                      1 1x AVG filter (default)
                                                                      2 to 5 Reserved
                                                                      6 16x AVG filter
                                                                      7 to 15 Reserved
                                                             LN Mode: Bandwidth for Accel LPF*/
        ACCEL_UI_FILT_BW_MASK = Bit7 | Bit6 | Bit5 | Bit4,
        ACCEL_UI_FILT_BW_ODR_2 = BitNone, //0 BW=ODR/2
        ACCEL_UI_FILT_BW_ODR_4 = Bit4, //1 BW=max(400Hz, ODR)/4 (default)
        ACCEL_UI_FILT_BW_ODR_5 = Bit5, //2 BW=max(400Hz, ODR)/5
        ACCEL_UI_FILT_BW_ODR_8 = Bit5 | Bit4, //3 BW=max(400Hz, ODR)/8
        ACCEL_UI_FILT_BW_ODR_10 = Bit6, //4 BW=max(400Hz, ODR)/10
        ACCEL_UI_FILT_BW_ODR_16 = Bit6 | Bit4, //5 BW=max(400Hz, ODR)/16
        ACCEL_UI_FILT_BW_ODR_20 = Bit6 | Bit5, //6 BW=max(400Hz, ODR)/20
        ACCEL_UI_FILT_BW_ODR_40 = Bit6 | Bit5 | Bit4, //7 BW=max(400Hz, ODR)/40
                                                      //8 to 13: Reserved
        ACCEL_UI_FILT_BW_LOW_LATENCY1 = Bit7 | Bit6 | Bit5, //14 Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(400Hz, ODR)
        ACCEL_UI_FILT_BW_LOW_LATENCY2 = Bit7 | Bit6 | Bit5 | Bit4, //15 Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(200Hz, 8*ODR)

        GYRO_UI_FILT_BW_MASK = Bit3 | Bit2 | Bit1 | Bit0, //LN Mode: Bandwidth for Gyro LPF
        GYRO_UI_FILT_BW_ODR_2 = BitNone, //0 BW=ODR/2
        GYRO_UI_FILT_BW_ODR_4 = Bit0, //1 BW=max(400Hz, ODR)/4 (default)
        GYRO_UI_FILT_BW_ODR_5 = Bit1, //2 BW=max(400Hz, ODR)/5
        GYRO_UI_FILT_BW_ODR_8 = Bit1 | Bit0, //3 BW=max(400Hz, ODR)/8
        GYRO_UI_FILT_BW_ODR_10 = Bit2, //4 BW=max(400Hz, ODR)/10
        GYRO_UI_FILT_BW_ODR_16 = Bit2 | Bit0, //5 BW=max(400Hz, ODR)/16
        GYRO_UI_FILT_BW_ODR_20 = Bit2 | Bit1, //6 BW=max(400Hz, ODR)/20
        GYRO_UI_FILT_BW_ODR_40 = Bit2 | Bit1 | Bit0, //7 BW=max(400Hz, ODR)/40
                                                     //8 to 13: Reserved
        GYRO_UI_FILT_BW_LOW_LATENCY1 = Bit3 | Bit2 | Bit1, //14 Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(400Hz, ODR)
        GYRO_UI_FILT_BW_LOW_LATENCY2 = Bit3 | Bit2 | Bit1 | Bit0, //15 Low Latency option: Trivial decimation @ ODR of Dec2 filter output. Dec2 runs at max(200Hz, 8*ODR)
    };

    /**
     * @Name: ACCEL_CONFIG1
     * @Address: 83 (53h)
     * @Serial IF: R/W
     * @Reset value: 0x0D
     * @Clock Domain: SCLK_UI
     */
    enum class ACCEL_CONFIG1_BITS:uint8_t
    {
        ACCEL_UI_FILT_ORD_MASK = Bit4 | Bit3, //Selects order of ACCEL UI filter
        ACCEL_UI_FILT_ORD_1 = BitNone, //00: 1st Order
        ACCEL_UI_FILT_ORD_2 = Bit3, //01: 2nd Order
        ACCEL_UI_FILT_ORD_3 = Bit4, //10: 3rd Order
                                    //11: Reserved

        ACCEL_DEC2_M2_ORD_MASK = Bit2 | Bit1, //Order of Accelerometer DEC2_M2 filter
                                   //00: Reserved
                                   //01: Reserved
        ACCEL_DEC2_M2_ORD3 = Bit2, //10: 3rd order
                                   //11: Reserved
    };

    /**
     * @Name: TMST_CONFIG
     * @Address: 84 (54h)
     * @Serial IF: R/W
     * @Reset value: 0x23
     * @Clock Domain: SCLK_UI
     */
    enum class TMST_CONFIG_BITS:uint8_t
    {
        TMST_TO_REGS_EN = Bit4, //0: TMST_VALUE[19:0] read always returns 0s 1: TMST_VALUE[19:0] read returns timestamp value
        TMST_RES = Bit3, //Time Stamp resolution: When set to 0 (default), time stamp resolution is 1 µs. When set to 1, resolution is 16µs
        TMST_DELTA_EN = Bit2, //Time Stamp delta enable: When set to 1, the time stamp field contains the  measurement of time since  the last occurrence of ODR.
        TMST_FSYNC_EN = Bit1, /*Time Stamp register FSYNC enable (default). When set to 1, the contents of the Timestamp feature of FSYNC is enabled.
                                The user also needs to select FIFO_TMST_FSYNC_EN in order to propagate the timestamp value to the FIFO.*/
        TMST_EN = Bit0, //0: Time Stamp register disable 1: Time Stamp register enable (default)
    };

    /**
     * @Name: FIFO_CONFIG1
     * @Address: 95 (5Fh)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class FIFO_CONFIG1_BITS:uint8_t
    {
        FIFO_RESUME_PARTIAL_RD = Bit6, //0: Partial FIFO read disabled, requires re-reading of the entire FIFO 1: FIFO read can be partial, and resume from last read point
        FIFO_WM_GT_TH = Bit5, //Trigger FIFO watermark interrupt on every ODR (DMA write) if FIFO_COUNT ≥ FIFO_WM_TH
        FIFO_HIRES_EN = Bit4, //Enable 3 bytes of extended 20-bits accel, gyro data + 1 byte of extended 16-bit temperature sensor data to be placed into the FIFO
        FIFO_TMST_FSYNC_EN = Bit3, //Must be set to 1 for all FIFO use cases when FSYNC is used.
        FIFO_TEMP_EN = Bit2, //Enable temperature sensor packets to go to FIFO
        FIFO_GYRO_EN = Bit1, //Enable gyroscope packets to go to FIFO
        FIFO_ACCEL_EN = Bit0, //Enable accelerometer packets to go to FIFO
    };

    /**
     * @Name: FIFO_CONFIG2
     * @Address: 96 (60h)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class FIFO_CONFIG2_BITS:uint8_t
    {
        FIFO_WM_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, /*Lower bits of FIFO watermark.  Generate interrupt when the FIFO reaches
                                                                                    or exceeds FIFO_WM size in bytes or records according to
                                                                                    FIFO_COUNT_REC setting.  FIFO_WM_EN must be zero before writing this register.
                                                                                    Interrupt only fires once.  This register should be set to non-zero
                                                                                    value, before choosing this interrupt source.*/
    };

    /**
     * @Name: FIFO_CONFIG3
     * @Address: 97 (61h)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     * @Note: Do not set FIFO_WM to value 0.
     */
    enum class FIFO_CONFIG3_BITS:uint8_t
    {
        FIFO_WM_11_8_MASK = Bit3 | Bit2 | Bit1 | Bit0, /*Upper bits of FIFO watermark.  Generate interrupt when the FIFO reaches
                                                         or exceeds FIFO_WM size in bytes or records according to
                                                         FIFO_COUNT_REC setting.  FIFO_WM_EN must be zero before writing this register.
                                                         Interrupt only fires once.  This register should be set to non-zero
                                                         value, before choosing this interrupt source.*/
    };

    /**
     * @Name: FSYNC_CONFIG
     * @Address: 98 (62h)
     * @Serial IF: R/W
     * @Reset value: 0x10
     * @Clock Domain: SCLK_UI
     */
    enum class FSYNC_CONFIG_BITS:uint8_t
    {
        FSYNC_UI_SEL_MASK = Bit6 | Bit5 |Bit4,
        FSYNC_UI_SEL_DO_NOT = BitNone, //000: Do not tag FSYNC flag
        FSYNC_UI_SEL_TEMP_OUT = Bit4, //001: Tag FSYNC flag to TEMP_OUT LSB
        FSYNC_UI_SEL_GYRO_XOUT = Bit5, //010: Tag FSYNC flag to GYRO_XOUT LSB
        FSYNC_UI_SEL_GYRO_YOUT = Bit5 |Bit4, //011: Tag FSYNC flag to GYRO_YOUT LSB
        FSYNC_UI_SEL_GYRO_ZOUT = Bit6, //100: Tag FSYNC flag to GYRO_ZOUT LSB
        FSYNC_UI_SEL_ACCEL_XOUT = Bit6 | Bit4, //101: Tag FSYNC flag to ACCEL_XOUT LSB
        FSYNC_UI_SEL_ACCEL_YOUT = Bit6 | Bit5, //110: Tag FSYNC flag to ACCEL_YOUT LSB
        FSYNC_UI_SEL_ACCEL_ZOUT = Bit6 | Bit5 |Bit4, //111: Tag FSYNC flag to ACCEL_ZOUT LSB

        FSYNC_UI_FLAG_CLEAR_SEL = Bit1, /*0: FSYNC flag is cleared when UI sensor register is updated
                                         1: FSYNC flag is cleared when UI interface reads the sensor register LSB of FSYNC tagged axis*/
        FSYNC_POLARITY = Bit0, //0: Start from Rising edge of FSYNC pulse to measure FSYNC interval 1: Start from Falling edge of FSYNC pulse to measure FSYNC interval
    };

    /**
     * @Name: INT_CONFIG0
     * @Address: 99 (63h)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class INT_CONFIG0_BITS:uint8_t
    {
        UI_DRDY_INT_CLEAR_MASK = Bit5 | Bit4, //Data Ready Interrupt Clear Option (latched mode)
        UI_DRDY_INT_CLEAR_STATUS_READ = BitNone, //00: Clear on Status Bit Read (default)
                                              //01: Clear on Status Bit Read
        UI_DRDY_INT_CLEAR_SENSOR_READ = Bit5, //10: Clear on Sensor Register Read
        UI_DRDY_INT_CLEAR_STATUS_AND_SENSOR_READ = Bit5 | Bit4, //11: Clear on Status Bit Read AND on Sensor Register read

        FIFO_THS_INT_CLEAR_MASK = Bit3 | Bit2, //FIFO Threshold Interrupt Clear Option (latched mode)
        FIFO_THS_INT_CLEAR_STATUS_READ = BitNone, //00: Clear on Status Bit Read (default)
                                               //01: Clear on Status Bit Read
        FIFO_THS_INT_CLEAR_FIFO_READ = Bit3, //10: Clear on FIFO data 1Byte Read
        FIFO_THS_INT_CLEAR_STATUS_AND_FIFO_READ = Bit3 | Bit2, //11: Clear on Status Bit Read AND on FIFO data 1 byte read

        FIFO_FULL_INT_CLEAR_MASK = Bit1 | Bit0, //FIFO Full Interrupt Clear Option (latched mode)
        FIFO_FULL_INT_CLEAR_STATUS_READ = BitNone, //00: Clear on Status Bit Read (default)
                                                //01: Clear on Status Bit Read
        FIFO_FULL_INT_CLEAR_FIFO_READ = Bit1, //10: Clear on FIFO data 1Byte Read
        FIFO_FULL_INT_CLEAR_STATUS_AND_FIFO_READ = Bit1 | Bit0, //11: Clear on Status Bit Read AND on FIFO data 1 byte read
    };

    /**
     * @Name: INT_CONFIG1
     * @Address: 100 (64h)
     * @Serial IF: R/W
     * @Reset value: 0x10
     * @Clock Domain: SCLK_UI
     */
    enum class INT_CONFIG1_BITS:uint8_t
    {
        INT_TPULSE_DURATION = Bit6, /*Interrupt pulse duration 0: Interrupt pulse duration is 100µs.  Use only if ODR < 4kHz. (Default)
                                                               1: Interrupt pulse duration is 8 µs.  Required if ODR ≥ 4kHz, optional for ODR < 4kHz.*/
        INT_TDEASSERT_DISABLE = Bit5, /*Interrupt de-assertion duration 0: The interrupt de-assertion duration is set to a minimum of 100µs. Use only if ODR < 4kHz. (Default)
                                                                        1: Disables de-assert duration.  Required if ODR ≥ 4kHz, optional for ODR < 4kHz. */
        INT_ASYNC_RESET = Bit4, /*User should change setting to 0 from default setting of 1, for proper INT1 and INT2 pin operation*/
    };

    /**
     * @Name: INT_SOURCE0
     * @Address: 101 (65h)
     * @Serial IF: R/W
     * @Reset value: 0x10
     * @Clock Domain: SCLK_UI
     */
    enum class INT_SOURCE0_BITS:uint8_t
    {
        UI_FSYNC_INT1_EN = Bit6, //0: UI FSYNC interrupt not routed to INT1 1: UI FSYNC interrupt routed to INT1
        PLL_RDY_INT1_EN = Bit5, //0: PLL ready interrupt not routed to INT1 1: PLL ready interrupt routed to INT1
        RESET_DONE_INT1_EN = Bit4, //0: Reset done interrupt not routed to INT1 1: Reset done interrupt routed to INT1
        UI_DRDY_INT1_EN = Bit3, //0: UI data ready interrupt not routed to INT1 1: UI data ready interrupt routed to INT1
        FIFO_THS_INT1_EN = Bit2, //0: FIFO threshold interrupt not routed to INT1 1: FIFO threshold interrupt routed to INT1
        FIFO_FULL_INT1_EN = Bit1, //0: FIFO full interrupt not routed to INT1 1: FIFO full interrupt routed to INT1
        UI_AGC_RDY_INT1_EN = Bit0, //0: UI AGC ready interrupt not routed to INT1 1: UI AGC ready interrupt routed to INT1
    };

    /**
     * @Name: INT_SOURCE1
     * @Address: 102 (66h)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class INT_SOURCE1_BITS:uint8_t
    {
        I3C_PROTOCOL_ERROR_INT1_EN = Bit6, //0: I3CSM protocol error interrupt not routed to INT1 1: I3CSM protocol error interrupt routed to INT1
        SMD_INT1_EN = Bit3, //0: SMD interrupt not routed to INT1 1: SMD interrupt routed to INT1
        WOM_Z_INT1_EN = Bit2, //0: Z-axis WOM interrupt not routed to INT1 1: Z-axis WOM interrupt routed to INT1
        WOM_Y_INT1_EN = Bit1, //0: Y-axis WOM interrupt not routed to INT1 1: Y-axis WOM interrupt routed to INT1
        WOM_X_INT1_EN = Bit0, //0: X-axis WOM interrupt not routed to INT1 1: X-axis WOM interrupt routed to INT1
    };

    /**
     * @Name: INT_SOURCE3
     * @Address: 104 (68h)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class INT_SOURCE3_BITS:uint8_t
    {
        UI_FSYNC_INT2_EN = Bit6, //0: UI FSYNC interrupt not routed to INT2 1: UI FSYNC interrupt routed to INT2
        PLL_RDY_INT2_EN = Bit5, //0: PLL ready interrupt not routed to INT2 1: PLL ready interrupt routed to INT2
        RESET_DONE_INT2_EN = Bit4, //0: Reset done interrupt not routed to INT2 1: Reset done interrupt routed to INT2
        UI_DRDY_INT2_EN = Bit3, //0: UI data ready interrupt not routed to INT2 1: UI data ready interrupt routed to INT2
        FIFO_THS_INT2_EN = Bit2, //0: FIFO threshold interrupt not routed to INT2 1: FIFO threshold interrupt routed to INT2
        FIFO_FULL_INT2_EN = Bit1, //0: FIFO full interrupt not routed to INT2 1: FIFO full interrupt routed to INT2
        UI_AGC_RDY_INT2_EN = Bit0, //0: UI AGC ready interrupt not routed to INT2 1: UI AGC ready interrupt routed to INT2
    };

    /**
     * @Name: INT_SOURCE4
     * @Address: 105 (69h)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class INT_SOURCE4_BITS:uint8_t
    {
        I3C_PROTOCOL_ERROR_INT2_EN = Bit6, //0: I3CSM protocol error interrupt not routed to INT2 1: I3CSM protocol error interrupt routed to INT2
        SMD_INT2_EN = Bit3, //0: SMD interrupt not routed to INT2 1: SMD interrupt routed to INT2
        WOM_Z_INT2_EN = Bit2, //0: Z-axis WOM interrupt not routed to INT2 1: Z-axis WOM interrupt routed to INT2
        WOM_Y_INT2_EN = Bit1, //0: Y-axis WOM interrupt not routed to INT2 1: Y-axis WOM interrupt routed to INT2
        WOM_X_INT2_EN = Bit0, //0: X-axis WOM interrupt not routed to INT2 1: X-axis WOM interrupt routed to INT2
    };

    /**
     * @Name: SELF_TEST_CONFIG
     * @Address: 112 (70h)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class SELF_TEST_CONFIG_BITS:uint8_t
    {
        ACCEL_ST_POWER = Bit6, //Set to 1 for accel self-test Otherwise set to 0; Set to 0 after self-test is completed
        EN_AZ_ST = Bit5, //Enable Z-accel self-test
        EN_AY_ST = Bit4, //Enable Y-accel self-test
        EN_AX_ST = Bit3, //Enable X-accel self-test
        EN_GZ_ST = Bit2, //Enable Z-gyro self-test
        EN_GY_ST = Bit1, //Enable Y-gyro self-test
        EN_GX_ST = Bit0, //Enable X-gyro self-test
    };

    /**
     * @Name: SELF_TEST_CONFIG
     * @Address: 117 (75h)
     * @Serial IF: R
     * @Reset value: 0x47
     * @Clock Domain: SCLK_UI
     * @brief This register is used to verify the identity of the device. The contents of WHOAMI is an 8-bit device ID. The default value of the
     *        register is 0x47. This is different from the I2C address of the device as seen on the slave I2C controller by the applications processor.
     */
    enum class WHO_AM_I_BITS:uint8_t
    {
        WHOAMI_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Register to indicate to user which device is being accessed
    };

    /**
     * @Name: REG_BANK_SEL
     * @Address: 118 (76h)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: ALL
     * @Note: This register is accessible from all register banks
     */
    enum class REG_BANK_SEL_BITS:uint8_t
    {
        BANK_SEL_MASK = Bit2 | Bit1 | Bit0, //Register bank selection
        BANK_SEL_0 = BitNone, //000: Bank 0 (default)
        BANK_SEL_1 = Bit0, //001: Bank 1
        BANK_SEL_2 = Bit1, //010: Bank 2
        BANK_SEL_3 = Bit1 | Bit0, //011: Bank 3
        BANK_SEL_4 = Bit2, //100: Bank 4
                           //101: Reserved
                           //110: Reserved
                           //111: Reserved
    };
    /************************************************************************************************************************************************/


    /************************************************************BANK1 Registers Describe************************************************************/
    /**
     * @Name: SENSOR_CONFIG0
     * @Address: 03 (03h)
     * @Serial IF: R/W
     * @Reset value: 0x80
     * @Clock Domain: SCLK_UI
     */
    enum class SENSOR_CONFIG0_BITS:uint8_t
    {
        ZG_DISABLE = Bit5, //0: Z gyroscope is on 1: Z gyroscope is disabled
        YG_DISABLE = Bit4, //0: Y gyroscope is on 1: Y gyroscope is disabled
        XG_DISABLE = Bit3, //0: X gyroscope is on 1: X gyroscope is disabled
        ZA_DISABLE = Bit2, //0: Z accelerometer is on 1: Z accelerometer is disabled
        YA_DISABLE = Bit1, //0: Y accelerometer is on 1: Y accelerometer is disabled
        XA_DISABLE = Bit0, //0: X accelerometer is on 1: X accelerometer is disabled
    };

    /**
     * @Name: GYRO_CONFIG_STATIC2
     * @Address: 11 (0Bh)
     * @Serial IF: R/W
     * @Reset value: 0xA0
     * @Clock Domain: SCLK_UI
     */
    enum class GYRO_CONFIG_STATIC2_BITS:uint8_t
    {
        GYRO_AAF_DIS = Bit1, //0: Enable gyroscope anti-aliasing filter (default) 1: Disable gyroscope anti-aliasing filter
        GYRO_NF_DIS = Bit0, //0: Enable Notch Filter (default) 1: Disable Notch Filter
    };

    /**
     * @Name: GYRO_CONFIG_STATIC3
     * @Address: 12 (0Ch)
     * @Serial IF: R/W
     * @Reset value: 0x0D
     * @Clock Domain: SCLK_UI
     */
    enum class GYRO_CONFIG_STATIC3_BITS:uint8_t
    {
        GYRO_AAF_DELT_MASK = Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Controls bandwidth of the gyroscope anti-alias filter See section 5.2 for details
    };

    /**
     * @Name: GYRO_CONFIG_STATIC4
     * @Address: 13 (0Dh)
     * @Serial IF: R/W
     * @Reset value: 0xAA
     * @Clock Domain: SCLK_UI
     */
    enum class GYRO_CONFIG_STATIC4_BITS:uint8_t
    {
        GYRO_AAF_DELTSQR_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Controls bandwidth of the gyroscope anti-alias filter See section 5.2 for details
    };

    /**
     * @Name: GYRO_CONFIG_STATIC5
     * @Address: 14 (0Eh)
     * @Serial IF: R/W
     * @Reset value: 0x80
     * @Clock Domain: SCLK_UI
     */
    enum class GYRO_CONFIG_STATIC5_BITS:uint8_t
    {
        GYRO_AAF_BITSHIFT_MASK = Bit7 | Bit6 | Bit5 | Bit4, //Controls bandwidth of the gyroscope anti-alias filter See section 5.2 for details
        GYRO_AAF_DELTSQR_11_8_MASK = Bit3 | Bit2 | Bit1 | Bit0, //Controls bandwidth of the gyroscope anti-alias filter See section 5.2 for details
    };

    /**
     * @Name: GYRO_CONFIG_STATIC6
     * @Address: 15 (0Fh)
     * @Serial IF: R/W
     * @Reset value: 0xXX (Factory trimmed on an individual device basis)
     * @Clock Domain: SCLK_UI
     */
    enum class GYRO_CONFIG_STATIC6_BITS:uint8_t
    {
        GYRO_X_NF_COSWZ_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Used for gyroscope X-axis notch filter frequency selection See section 5.1 for details
    };

    /**
     * @Name: GYRO_CONFIG_STATIC7
     * @Address: 16 (10h)
     * @Serial IF: R/W
     * @Reset value: 0xXX (Factory trimmed on an individual device basis)
     * @Clock Domain: SCLK_UI
     */
    enum class GYRO_CONFIG_STATIC7_BITS:uint8_t
    {
        GYRO_Y_NF_COSWZ_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Used for gyroscope Y-axis notch filter frequency selection See section 5.1 for details
    };

    /**
     * @Name: GYRO_CONFIG_STATIC8
     * @Address: 17 (11h)
     * @Serial IF: R/W
     * @Reset value: 0xXX (Factory trimmed on an individual device basis)
     * @Clock Domain: SCLK_UI
     */
    enum class GYRO_CONFIG_STATIC8_BITS:uint8_t
    {
        GYRO_Z_NF_COSWZ_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Used for gyroscope Z-axis notch filter frequency selection See section 5.1 for details
    };

    /**
     * @Name: GYRO_CONFIG_STATIC9
     * @Address: 18 (12h)
     * @Serial IF: R/W
     * @Reset value: 0xXX (Factory trimmed on an individual device basis)
     * @Clock Domain: SCLK_UI
     */
    enum class GYRO_CONFIG_STATIC9_BITS:uint8_t
    {
        GYRO_Z_NF_COSWZ_SEL = Bit5, //Used for gyroscope Z-axis notch filter frequency selection See section 5.1 for details
        GYRO_Y_NF_COSWZ_SEL = Bit4, //Used for gyroscope Y-axis notch filter frequency selection See section 5.1 for details
        GYRO_X_NF_COSWZ_SEL = Bit3, //Used for gyroscope X-axis notch filter frequency selection See section 5.1 for details
        GYRO_Z_NF_COSWZ_8_MASK = Bit2, //Used for gyroscope Z-axis notch filter frequency selection See section 5.1 for details
        GYRO_Y_NF_COSWZ_8_MASK = Bit1, //Used for gyroscope Y-axis notch filter frequency selection See section 5.1 for details
        GYRO_X_NF_COSWZ_8_MASK = Bit0, //Used for gyroscope X-axis notch filter frequency selection See section 5.1 for details
    };

    /**
     * @Name: GYRO_CONFIG_STATIC10
     * @Address: 19 (13h)
     * @Serial IF: R/W
     * @Reset value: 0x11
     * @Clock Domain: SCLK_UI
     */
    enum class GYRO_CONFIG_STATIC10_BITS:uint8_t
    {
        GYRO_NF_BW_SEL_MASK = Bit6 | Bit5 | Bit4, //Selects bandwidth for gyroscope notch filter See section 5.1 for details
        GYRO_NF_BW_SEL_1449HZ = BitNone, //0:1449Hz
        GYRO_NF_BW_SEL_680HZ = Bit4, //1:680Hz
        GYRO_NF_BW_SEL_329HZ = Bit5, //2:329Hz
        GYRO_NF_BW_SEL_162HZ = Bit5 | Bit4, //3:162Hz
        GYRO_NF_BW_SEL_80HZ = Bit6, //4:80Hz
        GYRO_NF_BW_SEL_40HZ = Bit6 | Bit4, //5:40Hz
        GYRO_NF_BW_SEL_20HZ = Bit6 | Bit5, //6:20Hz
        GYRO_NF_BW_SEL_10HZ = Bit6 | Bit5 | Bit4, //7:10Hz
    };

    /**
     * @Name: INTF_CONFIG5
     * @Address: 123 (7Bh)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class INTF_CONFIG5_BITS:uint8_t
    {
        PIN9_FUNCTION_MASK = Bit2 | Bit1, //Selects among the following functionalities for pin 9
        PIN9_FUNCTION_INT2 = BitNone, //00: INT2
        PIN9_FUNCTION_FSYNC = Bit1, //01: FSYNC
        PIN9_FUNCTION_CLKIN = Bit2, //10: CLKIN
                                    //11: Reserved
    };
    /************************************************************************************************************************************************/


    /************************************************************BANK2 Registers Describe************************************************************/
    /**
     * @Name: ACCEL_CONFIG_STATIC2
     * @Address: 03 (03h)
     * @Serial IF: R/W
     * @Reset value: 0x30
     * @Clock Domain: SCLK_UI
     */
    enum class ACCEL_CONFIG_STATIC2_BITS:uint8_t
    {
        ACCEL_AAF_DELT_MASK = Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1, //Controls bandwidth of the accelerometer anti-alias filter See section 5.2 for details
        ACCEL_AAF_DIS = Bit0, //0: Enable accelerometer anti-aliasing filter (default) 1: Disable accelerometer anti-aliasing filter
    };

    /**
     * @Name: ACCEL_CONFIG_STATIC3
     * @Address: 04 (04h)
     * @Serial IF: R/W
     * @Reset value: 0x40
     * @Clock Domain: SCLK_UI
     */
    enum class ACCEL_CONFIG_STATIC3_BITS:uint8_t
    {
        ACCEL_AAF_DELTSQR_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Controls bandwidth of the accelerometer anti-alias filter See section 5.2 for details
    };

    /**
     * @Name: ACCEL_CONFIG_STATIC4
     * @Address: 05 (05h)
     * @Serial IF: R/W
     * @Reset value: 0x62
     * @Clock Domain: SCLK_UI
     */
    enum class ACCEL_CONFIG_STATIC4_BITS:uint8_t
    {
        ACCEL_AAF_BITSHIFT_MASK = Bit7 | Bit6 | Bit5 | Bit4, //Controls bandwidth of the accelerometer anti-alias filter See section 5.2 for details
        ACCEL_AAF_DELTSQR_11_8_MASK = Bit3 | Bit2 | Bit1 | Bit0, //Controls bandwidth of the accelerometer anti-alias filter See section 5.2 for details
    };
    /************************************************************************************************************************************************/


    /************************************************************BANK4 Registers Describe************************************************************/
    /**
     * @Name: OFFSET_USER0
     * @Address: 119 (77h)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class OFFSET_USER0_BITS:uint8_t
    {
        GYRO_X_OFFUSER_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Lower bits of X-gyro offset programmed by user.  Max value is ±64 dps, resolution is 1/32 dps.
    };

    /**
     * @Name: OFFSET_USER1
     * @Address: 120 (78h)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class OFFSET_USER1_BITS:uint8_t
    {
        GYRO_Y_OFFUSER_11_8_MASK = Bit7 | Bit6 | Bit5 | Bit4, //Upper bits of Y-gyro offset programmed by user.  Max value is ±64 dps, resolution is 1/32 dps.
    };

    /**
     * @Name: OFFSET_USER2
     * @Address: 121 (79h)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class OFFSET_USER2_BITS:uint8_t
    {
        GYRO_Y_OFFUSER_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Lower bits of Y-gyro offset programmed by user.  Max value is ±64 dps, resolution is 1/32 dps.
    };

    /**
     * @Name: OFFSET_USER3
     * @Address: 122 (7Ah)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class OFFSET_USER3_BITS:uint8_t
    {
        GYRO_Z_OFFUSER_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Lower bits of Z-gyro offset programmed by user.  Max value is ±64 dps, resolution is 1/32 dps.
    };

    /**
     * @Name: OFFSET_USER4
     * @Address: 123 (7Bh)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class OFFSET_USER4_BITS:uint8_t
    {
        ACCEL_X_OFFUSER_11_8_MASK = Bit7 | Bit6 | Bit5 | Bit4, //Upper bits of X-accel offset programmed by user.  Max value is ±1g, resolution is 0.5mg.
        GYRO_Z_OFFUSER_11_8_MASK = Bit3 | Bit2 | Bit1 | Bit0, //Upper bits of Z-gyro offset programmed by user.  Max value is ±64 dps, resolution is 1/32 dps.
    };

    /**
     * @Name: OFFSET_USER5
     * @Address: 124 (7Ch)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class OFFSET_USER5_BITS:uint8_t
    {
        ACCEL_X_OFFUSER_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Lower bits of X-accel offset programmed by user.  Max value is ±1g, resolution is 0.5mg.
    };

    /**
     * @Name: OFFSET_USER6
     * @Address: 125 (7Dh)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class OFFSET_USER6_BITS:uint8_t
    {
        ACCEL_Y_OFFUSER_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Lower bits of Y-accel offset programmed by user.  Max value is ±1g, resolution is 0.5mg.
    };

    /**
     * @Name: OFFSET_USER7
     * @Address: 126 (7Eh)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class OFFSET_USER7_BITS:uint8_t
    {
        ACCEL_Z_OFFUSER_11_8_MASK = Bit7 | Bit6 | Bit5 | Bit4, //Upper bits of Z-accel offset programmed by user.  Max value is ±1g, resolution is 0.5mg.
        ACCEL_Y_OFFUSER_11_8_MASK = Bit3 | Bit2 | Bit1 | Bit0, //Upper bits of Y-accel offset programmed by user.  Max value is ±1g, resolution is 0.5mg.
    };

    /**
     * @Name: OFFSET_USER8
     * @Address: 127 (7Fh)
     * @Serial IF: R/W
     * @Reset value: 0x00
     * @Clock Domain: SCLK_UI
     */
    enum class OFFSET_USER8_BITS:uint8_t
    {
        ACCEL_Z_OFFUSER_7_0_MASK = Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0, //Lower bits of Z-accel offset programmed by user.  Max value is ±1g, resolution is 0.5mg.
    };
    /************************************************************************************************************************************************/


    /********************************************位运算重载，因为作用域枚举（enum class）不支持位运算******************************************************/
    //定义trait（默认 false）
    template<typename E>
    constexpr bool _disableBitOperators = false;

    //特化DisableBitOperators<E>的值，置为true的枚举将不被允许后面的重载，因为寄存器地址不涉及需要位运算
    template<>
    constexpr bool _disableBitOperators<RegAdd::BANK0> = true;

    template<>
    constexpr bool _disableBitOperators<RegAdd::BANK1> = true;

    template<>
    constexpr bool _disableBitOperators<RegAdd::BANK2> = true;

    template<>
    constexpr bool _disableBitOperators<RegAdd::BANK4> = true;

    //重载“位与&”运算
    template<typename E>
    constexpr std::enable_if_t< std::is_enum_v<E> && !_disableBitOperators<E>, E > operator&(E lhs, E rhs)
    {
        using U = std::underlying_type_t<E>;

        return static_cast<E>( static_cast<U>(lhs) & static_cast<U>(rhs) );
    }

    //重载“位或|”运算
    template<typename E>
    constexpr std::enable_if_t< std::is_enum_v<E> && !_disableBitOperators<E>, E > operator|(E lhs, E rhs) //如果E是枚举类型，该重载才启用
    {
        using U = std::underlying_type_t<E>;

        return static_cast<E>( static_cast<U>(lhs) | static_cast<U>(rhs) );
    }

    //重载“位非~”运算
    template<typename E>
    constexpr std::enable_if_t< std::is_enum_v<E> && !_disableBitOperators<E>, E > operator~(E rhs)
    {
        using U = std::underlying_type_t<E>;

        return static_cast<E>( ~static_cast<U>(rhs) );
    }

    //重载“复合赋值位与&=”运算
    template<typename E>
    constexpr std::enable_if_t< std::is_enum_v<E> && !_disableBitOperators<E>, E& > operator&=(E& lhs, E rhs)
    {
        using U = std::underlying_type_t<E>;

        lhs = static_cast<E>( static_cast<U>(lhs) & static_cast<U>(rhs) );

        return lhs;
    }

    //重载“复合赋值位或|=”运算
    template<typename E>
    constexpr std::enable_if_t< std::is_enum_v<E> && !_disableBitOperators<E>, E& > operator|=(E& lhs, E rhs)
    {
        using U = std::underlying_type_t<E>;

        lhs = static_cast<E>( static_cast<U>(lhs) | static_cast<U>(rhs) );

        return lhs;
    }
    /************************************************************************************************************************************************/
}
