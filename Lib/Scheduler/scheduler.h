//
// Created by Gray on 2026/1/7.
//

#ifndef ELECTROMAGNETICARTILLERY_SCHEDULER_H
#define ELECTROMAGNETICARTILLERY_SCHEDULER_H

#include "stm32h7xx.h"

#define TICK_PER_SECOND	1000
#define SCHED_HP_EVENT_IMU_DRDY (1u << 0)

typedef struct
{
    void(*task_func)(void);
    float rate_hz;
    uint16_t interval_ticks;
    uint32_t last_run;
}sched_task_t;

void Scheduler_Setup(void);
void Scheduler_Run(void);
void Scheduler_PostHighPriorityEventFromISR(uint32_t event);

#endif //ELECTROMAGNETICARTILLERY_SCHEDULER_H
