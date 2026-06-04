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
} ICM42688_Status;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} ICM42688_RawVector;

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
ICM42688_Status ICM42688_Read(int16_t accel[3], int16_t gyro[3], int16_t *temp);

#ifdef __cplusplus
}
#endif
