//
// Created by Gray on 2026/1/7.
//

#pragma once

#include "ICM42688P_Regisrers.hpp"

class Motor
{
public:
    explicit Motor(int id);

    void setSpeed(float rpm);
    [[nodiscard]] float getSpeed() const;

private:
    int _id;
    float _speed;
};
