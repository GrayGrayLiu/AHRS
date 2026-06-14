#pragma once

#include <stdint.h>

#include "spi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ICM42688_STATUS_OK = 0,
    ICM42688_STATUS_INVALID_ARGUMENT = -1,
    ICM42688_STATUS_SPI_ERROR = -2,
    ICM42688_STATUS_WRONG_DEVICE_ID = -3,
    ICM42688_STATUS_RESET_TIMEOUT = -4,
    ICM42688_STATUS_UNSUPPORTED = -5,
    ICM42688_STATUS_CONFIG_MISMATCH = -6,
    ICM42688_STATUS_NO_DATA = -7,
    ICM42688_STATUS_FIFO_OVERFLOW = -8,
    ICM42688_STATUS_BAD_FIFO_PACKET = -9,
} ICM42688_Status;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} ICM42688_RawVector;

typedef struct
{
    uint32_t timestamp_ms;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t temp_raw;
    float accel_m_s2[3];
    float gyro_rad_s[3];
    float temperature_deg_c;
    uint32_t sample_counter;
    uint32_t error_counter;
    uint8_t configured;
    uint8_t data_valid;
    int32_t accel_raw20[3];
    int32_t gyro_raw20[3];
    int32_t accel_effective[3];
    int32_t gyro_effective[3];
    uint16_t fifo_count_bytes;
    uint16_t fifo_valid_packets;
    uint16_t fifo_timestamp;
    uint8_t fifo_header;
    uint8_t data_source;
    float delta_angle_rad[3];
    float delta_velocity_m_s[3];
    float delta_time_s;
    uint16_t delta_samples;
    uint32_t interrupt_counter;
    uint32_t last_interrupt_timestamp_ms;
} ICM42688_Sample;

ICM42688_Status ICM42688_Bind(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
ICM42688_Status ICM42688_Init(void);
ICM42688_Status ICM42688_Probe(void);
ICM42688_Status ICM42688_Reset(void);

ICM42688_Status ICM42688_RegisterRead(uint8_t reg, uint8_t *value);
ICM42688_Status ICM42688_RegisterWrite(uint8_t reg, uint8_t value);
ICM42688_Status ICM42688_ReadBuffer(uint8_t start_reg, uint8_t *buffer, uint16_t length);
ICM42688_Status ICM42688_ReadRawAccel(ICM42688_RawVector *data);
ICM42688_Status ICM42688_ReadRawGyro(ICM42688_RawVector *data);

ICM42688_Status ICM42688_Update(void);
ICM42688_Status ICM42688_GetLatest(ICM42688_Sample *sample);
void ICM42688_OnDataReadyInterrupt(uint32_t timestamp_ms);

#ifdef __cplusplus
}
#endif
