/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RGB_R_Pin GPIO_PIN_2
#define RGB_R_GPIO_Port GPIOE
#define RGB_G_Pin GPIO_PIN_3
#define RGB_G_GPIO_Port GPIOE
#define RGB_B_Pin GPIO_PIN_4
#define RGB_B_GPIO_Port GPIOE
#define IMU_INT1_Pin GPIO_PIN_2
#define IMU_INT1_GPIO_Port GPIOA
#define IMU_INT2_Pin GPIO_PIN_3
#define IMU_INT2_GPIO_Port GPIOA
#define IMU_SPI_CS_Pin GPIO_PIN_4
#define IMU_SPI_CS_GPIO_Port GPIOA
#define IMU_SPI_SCK_Pin GPIO_PIN_5
#define IMU_SPI_SCK_GPIO_Port GPIOA
#define IMU_SPI_MISO_Pin GPIO_PIN_6
#define IMU_SPI_MISO_GPIO_Port GPIOA
#define IMU_SPI_MOSI_Pin GPIO_PIN_7
#define IMU_SPI_MOSI_GPIO_Port GPIOA
#define MAG_RSTN_Pin GPIO_PIN_4
#define MAG_RSTN_GPIO_Port GPIOB
#define MAG_DRDY_Pin GPIO_PIN_5
#define MAG_DRDY_GPIO_Port GPIOB
#define MAG_I2C_SCL_Pin GPIO_PIN_6
#define MAG_I2C_SCL_GPIO_Port GPIOB
#define MAG_I2C_SDA_Pin GPIO_PIN_7
#define MAG_I2C_SDA_GPIO_Port GPIOB
#define MAG_CAD0_Pin GPIO_PIN_8
#define MAG_CAD0_GPIO_Port GPIOB
#define MAG_CAD1_Pin GPIO_PIN_9
#define MAG_CAD1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
