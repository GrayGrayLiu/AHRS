//
// Created by Gray on 2026/1/7.
//

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct Motor Motor;
    typedef Motor* MotorHandle;

    MotorHandle Motor_Create(int id);
    void Motor_SetSpeed(MotorHandle motor, float speed);
    float Motor_GetSpeed(MotorHandle motor);

#ifdef __cplusplus
}
#endif