/**
 * @file    scheduler_app_events.h
 * @brief   应用层 Scheduler 事件掩码定义
 *
 * @details
 * 本文件集中定义挂载到 generic Scheduler 的应用级事件掩码。
 * scheduler.h 作为通用调度器核心接口，不定义任何具体外设事件。
 */

#ifndef ELECTROMAGNETICARTILLERY_SCHEDULER_APP_EVENTS_H
#define ELECTROMAGNETICARTILLERY_SCHEDULER_APP_EVENTS_H

#include "scheduler.h"

namespace scheduler_app_events
{
// IMU data-ready event
constexpr SchedulerEventMask IMU_DRDY = (1u << 0);
} // namespace scheduler_app_events

#endif // ELECTROMAGNETICARTILLERY_SCHEDULER_APP_EVENTS_H
