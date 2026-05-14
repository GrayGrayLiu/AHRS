#include "ICM42688_API.h"

#include <new>

#include "ICM42688P.hpp"

namespace
{
alignas(ICM42688P) unsigned char g_icm_storage[sizeof(ICM42688P)];
ICM42688P *g_icm = nullptr;
bool g_bound = false;
bool g_initialized = false;
} // namespace

extern "C" int ICM42688_Bind(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    if (hspi == nullptr || cs_port == nullptr) {
        return -1;
    }

    if (g_bound || g_icm != nullptr) {
        return -2;
    }

    g_icm = new (g_icm_storage) ICM42688P(hspi, cs_port, cs_pin);
    g_bound = true;
    g_initialized = false;
    return 0;
}

extern "C" int ICM42688_Init(void)
{
    if (!g_bound || g_icm == nullptr) {
        return -10;
    }

    if (!g_icm->Init()) {
        g_initialized = false;
        return -11;
    }

    g_initialized = true;
    return 0;
}

extern "C" int ICM42688_Update(void)
{
    if (!g_bound || g_icm == nullptr) {
        return -10;
    }

    if (!g_initialized) {
        return -12;
    }

    return g_icm->Update() ? 1 : 0;
}

extern "C" int ICM42688_Read(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
    if (!g_bound || g_icm == nullptr) {
        return -10;
    }

    if (!g_initialized) {
        return -12;
    }

    return g_icm->ReadLatest(accel, gyro, temp) ? 0 : 1;
}
