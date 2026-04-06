//
// Created by Gray on 2026/1/7.
//

#include "ICM42688P.hpp"

Motor::Motor(const int id) : _id(id),_speed(0.0f)
{
}

void Motor::setSpeed(const float rpm)
{
    _speed = rpm;
}

[[nodiscard]] float Motor::getSpeed() const
{
    return _speed;
}
