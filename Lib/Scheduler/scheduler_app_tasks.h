/**
 * @file    scheduler_app_tasks.h
 * @brief   Generic Scheduler 应用任务注册与业务初始化接口（C ABI）
 *
 * @details
 * 本文件声明应用层 scheduler task 注册入口和业务模块初始化入口。
 * main.c 在 Scheduler_Init() 成功后先调用 App_Init() 完成业务初始化，
 * 再调用 SchedulerAppTasks_RegisterAll() 注册 scheduler task。
 *
 * App_Init() 负责业务模块初始化；
 * SchedulerAppTasks_RegisterAll() 负责 scheduler task 注册和元数据绑定。
 */

#ifndef ELECTROMAGNETICARTILLERY_SCHEDULER_APP_TASKS_H
#define ELECTROMAGNETICARTILLERY_SCHEDULER_APP_TASKS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  应用/业务模块初始化入口。
 * @note   当前暂无额外初始化动作；该入口用于以后集中放置 scheduler task
 *         运行前必须完成的驱动、服务或算法模块初始化。
 *         本函数不注册 scheduler task；任务注册由 SchedulerAppTasks_RegisterAll() 负责。
 */
void App_Init(void);

/**
 * @brief  注册应用层任务到 generic Scheduler。
 * @note   必须在 Scheduler_Init() 和 App_Init() 之后调用。具体任务集合、周期、
 *         事件绑定、deadline 和优先级由 scheduler_app_tasks.cpp 维护。
 */
void SchedulerAppTasks_RegisterAll(void);

#ifdef __cplusplus
}
#endif

#endif // ELECTROMAGNETICARTILLERY_SCHEDULER_APP_TASKS_H
