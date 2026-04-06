//
// Created by Gray on 2026/1/7.
//

#pragma once

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
