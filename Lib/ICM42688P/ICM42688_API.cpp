//
// Created by Gray on 2026/1/7.
//

#include "ICM42688_API.h"
#include "ICM42688P.hpp"

MotorHandle Motor_Create(const int id)
{
    auto* motor = new Motor(id);
    return motor;
}

void Motor_SetSpeed(MotorHandle motor, const float speed)
{
    motor->setSpeed(speed);
}

float Motor_GetSpeed(MotorHandle motor)
{
    return motor->getSpeed();
}
