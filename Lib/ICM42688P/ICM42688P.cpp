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

using ICM42688P_Regs::RegsAdd::BANK0;
using ICM42688P_Regs::RegsAdd::BANK1;
using ICM42688P_Regs::RegsAdd::BANK2;

namespace
{
constexpr uint8_t kReadFlag = 0x80U;
constexpr uint8_t kWhoAmI = 0x47U;
constexpr int kWhoAmIRetry = 3;
}

ICM42688P::ICM42688P(SPI_HandleTypeDef* hspi,
                     GPIO_TypeDef* cs_port, uint16_t cs_pin)
    : hspi_(hspi), cs_port_(cs_port), cs_pin_(cs_pin)
{
}

bool ICM42688P::Init()
{
    initialized_ = false;

    if (hspi_ == nullptr || cs_port_ == nullptr) {
        ++error_count_;
        return false;
    }

    // Do not assume current chip bank; force BANK0 once during init.
    if (!SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_0, true)) {
        ++error_count_;
        return false;
    }

    if (!CheckWhoAmI()) {
        ++error_count_;
        return false;
    }

    initialized_ = true;
    return true;
}

bool ICM42688P::Update()
{
    // Stage1: no health check and no data path.
    return false;
}

bool ICM42688P::ReadLatest(int16_t accel[3], int16_t gyro[3], int16_t *temp) const
{
    (void)accel;
    (void)gyro;
    (void)temp;

    // Stage1: no sample fetch.
    return false;
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

bool ICM42688P::CheckWhoAmI()
{
    uint8_t who_am_i = 0;

    for (int i = 0; i < kWhoAmIRetry; ++i) {
        if (ReadRegister(BANK0::WHO_AM_I, who_am_i) && who_am_i == kWhoAmI) {
            return true;
        }
    }

    return false;
}

bool ICM42688P::SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS bank, bool force)
{
    if (!force && bank_selected_valid_ && bank == current_bank_) {
        return true;
    }

    const uint8_t bank_value = static_cast<uint8_t>(bank) &
        static_cast<uint8_t>(ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_MASK);

    if (!WriteRegisterRaw(static_cast<uint8_t>(BANK0::REG_BANK_SEL), bank_value)) {
        return false;
    }

    current_bank_ = bank;
    bank_selected_valid_ = true;
    return true;
}

bool ICM42688P::WriteRegister(BANK0 reg, uint8_t value)
{
    return SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_0) &&
        WriteRegisterRaw(static_cast<uint8_t>(reg), value);
}

bool ICM42688P::WriteRegister(BANK1 reg, uint8_t value)
{
    return SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_1) &&
        WriteRegisterRaw(static_cast<uint8_t>(reg), value);
}

bool ICM42688P::WriteRegister(BANK2 reg, uint8_t value)
{
    return SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_2) &&
        WriteRegisterRaw(static_cast<uint8_t>(reg), value);
}

bool ICM42688P::ReadRegister(BANK0 reg, uint8_t &value)
{
    return SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_0) &&
        ReadRegisterRaw(static_cast<uint8_t>(reg), value);
}

bool ICM42688P::ReadRegister(BANK1 reg, uint8_t &value)
{
    return SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_1) &&
        ReadRegisterRaw(static_cast<uint8_t>(reg), value);
}

bool ICM42688P::ReadRegister(BANK2 reg, uint8_t &value)
{
    return SelectBank(ICM42688P_Regs::REG_BANK_SEL_BITS::BANK_SEL_2) &&
        ReadRegisterRaw(static_cast<uint8_t>(reg), value);
}

bool ICM42688P::WriteRegisterRaw(uint8_t reg, uint8_t value) const
{
    uint8_t tx[2] = {static_cast<uint8_t>(reg & static_cast<uint8_t>(~kReadFlag)), value};

    CS_Low();
    const HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi_, tx, 2, HAL_MAX_DELAY);
    CS_High();

    return status == HAL_OK;
}

bool ICM42688P::ReadRegisterRaw(uint8_t reg, uint8_t &value) const
{
    uint8_t tx[2] = {static_cast<uint8_t>(reg | kReadFlag), 0xFFU};
    uint8_t rx[2] = {0U, 0U};

    CS_Low();
    const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi_, tx, rx, 2, HAL_MAX_DELAY);
    CS_High();

    if (status != HAL_OK) {
        return false;
    }

    value = rx[1];
    return true;
}
