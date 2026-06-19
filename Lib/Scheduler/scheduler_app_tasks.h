/**
 * @file    scheduler_app_tasks.h
 * @brief   Generic Scheduler 应用/调试任务注册接口（C ABI）
 *
 * @details
 * 本文件声明挂载到 generic Scheduler 的应用任务注册入口。
 * 所有应用级 task callback 和注册逻辑放在 scheduler_app_tasks.cpp 中，
 * 不污染 Scheduler 核心也不堆积在 main.c。
 *
 * main.c 只需在 Scheduler_Init() 成功后调用 SchedulerAppTasks_RegisterAll()。
 */

#ifndef ELECTROMAGNETICARTILLERY_SCHEDULER_APP_TASKS_H
#define ELECTROMAGNETICARTILLERY_SCHEDULER_APP_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  注册应用层任务到 generic Scheduler。
 * @note   必须在 Scheduler_Init() 成功之后调用。具体任务集合、周期、事件绑定、
 *         deadline 和优先级由 scheduler_app_tasks.cpp 维护，不在此头文件中枚举。
 */
void SchedulerAppTasks_RegisterAll(void);

#ifdef __cplusplus
}
#endif

#endif // ELECTROMAGNETICARTILLERY_SCHEDULER_APP_TASKS_H
