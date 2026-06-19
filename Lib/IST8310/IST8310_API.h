/******************************************************************************
 * @file    IST8310_API.h
 * @brief   C API wrapper for the IST8310 C++ driver
 *
 * @details
 * Exposes an opaque handle so C modules can use the minimal IST8310 driver
 * without depending on C++ class definitions.
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

#ifndef AHRS_IST8310_API_H
#define AHRS_IST8310_API_H

#include <stdint.h>

#include "i2c.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct IST8310_HandleImpl* IST8310_Handle;

typedef enum
{
    IST8310_STATUS_OK = 0,
    IST8310_STATUS_INVALID_ARGUMENT = -1,
    IST8310_STATUS_I2C_ERROR = -2,
    IST8310_STATUS_WRONG_DEVICE_ID = -3,
    IST8310_STATUS_RESET_TIMEOUT = -4,
    IST8310_STATUS_DATA_NOT_READY = -5,
    IST8310_STATUS_DATA_OVERRUN = -6,
    IST8310_STATUS_CONFIG_MISMATCH = -7,
} IST8310_Status;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} IST8310_RawMagData;

typedef struct
{
    float x;
    float y;
    float z;
} IST8310_MagData_uT;

IST8310_Handle IST8310_Create(I2C_HandleTypeDef* hi2c,
                              uint8_t address_7bit,
                              GPIO_TypeDef* reset_port,
                              uint16_t reset_pin);
void IST8310_Destroy(IST8310_Handle handle);

IST8310_Status IST8310_Init(IST8310_Handle handle);
IST8310_Status IST8310_Probe(IST8310_Handle handle);
IST8310_Status IST8310_Reset(IST8310_Handle handle);

IST8310_Status IST8310_RegisterRead(IST8310_Handle handle, uint8_t reg, uint8_t* value);
IST8310_Status IST8310_RegisterWrite(IST8310_Handle handle, uint8_t reg, uint8_t value);
IST8310_Status IST8310_ReadBuffer(IST8310_Handle handle,
                                  uint8_t start_reg,
                                  uint8_t* buffer,
                                  uint16_t length);

IST8310_Status IST8310_ReadRawMag(IST8310_Handle handle, IST8310_RawMagData* data);
IST8310_Status IST8310_ReadMag_uT(IST8310_Handle handle, IST8310_MagData_uT* data);

#ifdef __cplusplus
}
#endif

#endif //AHRS_IST8310_API_H
