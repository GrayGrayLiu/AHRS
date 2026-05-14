/******************************************************************************
 * @file    ICM42688P.cpp
 * @brief   ICM42688P源文件
 *
 * @details
 *
 * @author  Gray
 * @email   grayme12345@gmail.com
 * @date    2026/1/14
 *
 * @copyright
 * Copyright (c) 2026 Gray
 *
 * This software is provided "as is" without any warranties.
 *
 ******************************************************************************/

#include "ICM42688P.hpp"
#include <cstring>

ICM42688P::ICM42688P(SPI_HandleTypeDef* hspi,
                     GPIO_TypeDef* cs_port, uint16_t cs_pin)
    : hspi_(hspi), cs_port_(cs_port), cs_pin_(cs_pin)
{
}

void ICM42688P::CS_Low() const
{
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
}

void ICM42688P::CS_High() const
{
    HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
}

//////////////////////////////////////////////////////////
// 单字节写（阻塞即可，没必要DMA）
//////////////////////////////////////////////////////////
void ICM42688P::WriteByte(const uint8_t reg, const uint8_t value) const
{
    uint8_t tx[2];
    tx[0] = reg & 0x7F;
    tx[1] = value;

    CS_Low();
    HAL_SPI_Transmit(hspi_, tx, 2, HAL_MAX_DELAY);
    CS_High();
}

//////////////////////////////////////////////////////////
// 单字节读（DMA）
//////////////////////////////////////////////////////////
void ICM42688P::ReadByte(uint8_t reg)
{
    tx_buf_[0] = reg | 0x80;
    tx_buf_[1] = 0xFF;

    rx_len_ = 1;
    transfer_done_ = false;

    CS_Low();
    HAL_SPI_TransmitReceive_DMA(hspi_, tx_buf_, rx_buf_, 2);
}

//////////////////////////////////////////////////////////
// 多字节读（DMA）
//////////////////////////////////////////////////////////
void ICM42688P::ReadBytes(uint8_t reg, uint16_t len)
{
    if (len > MAX_LEN) return;

    tx_buf_[0] = reg | 0x80;
    memset(&tx_buf_[1], 0xFF, len);

    rx_len_ = len;
    transfer_done_ = false;

    CS_Low();
    HAL_SPI_TransmitReceive_DMA(hspi_, tx_buf_, rx_buf_, len + 1);
}

//////////////////////////////////////////////////////////
// DMA完成回调（核心）
//////////////////////////////////////////////////////////
void ICM42688P::TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    if (hspi != hspi_) return;

    CS_High();

    transfer_done_ = true;
}