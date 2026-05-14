#pragma once

#include <stdint.h>

#include "spi.h"

#ifdef __cplusplus
extern "C" {
#endif

int ICM42688_Bind(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
int ICM42688_Init(void);
int ICM42688_Update(void);
int ICM42688_Read(int16_t accel[3], int16_t gyro[3], int16_t *temp);

#ifdef __cplusplus
}
#endif
