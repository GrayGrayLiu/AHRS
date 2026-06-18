/**
 * @file    scheduler.cpp
 * @brief   通用裸机协作式 WorkQueue-like 调度器实现
 *
 * @details
 * 本文件分为两大部分：
 *   1. Generic WorkQueue-like Scheduler Core — 新增通用调度器核心
 *   2. Legacy Compatibility Scheduler — 旧运行路径（阶段 7 移除）
 *
 * Generic core 不直接依赖 ICM42688P / TimeBase / CMSIS / HAL 外设。
 * 时间和临界区通过 SchedulerPort 回调获取。
 *
 * Legacy section 仍使用原有的 TimeBase / ICM42688_Service 依赖，
 * 保持旧 Scheduler_Setup / Scheduler_Run / Scheduler_PostHighPriorityEventFromISR
 * 行为不变，直至后续阶段迁移完成。
 */

#include "scheduler.h"

// ── Legacy compatibility includes（仅供 legacy section 使用） ──
#include "stm32h7xx.h"
#include "ICM42688_Service.hpp"
#include "TimeBase.h"
#include <stdio.h>

// Generic core only uses scheduler.h types and port callbacks — no additional includes.

namespace
{

// ############################################################################
// Generic WorkQueue-like Scheduler Core — 内部类型与静态数据
// ############################################################################

// 内部任务槽位。调度器核心通过该结构管理每个已注册任务的完整生命周期。
struct TaskSlot
{
    const char        *name;                // 调试名称（可为 nullptr）
    SchedulerTaskFn    fn;                  // 任务回调
    void              *context;             // 用户上下文指针
    SchedulerPriority  priority;            // 优先级（三档）
    SchedulerEventMask event_mask;          // 本任务订阅的事件位掩码（0 = 不订阅事件）
    uint32_t           period_ms;           // 周期间隔（0 = 非周期任务）
    uint32_t           event_deadline_ms;   // event deadline 最大等待时间（0 = 无 deadline）

    // 下一个 time-triggered ready 的目标毫秒时间；用于 delayed / deadline / interval 到达判断。
    uint32_t           next_run_ms;
    // 已匹配到本任务但尚未交给 callback 的事件位集合。
    SchedulerEventMask pending_events;
    // 本任务已就绪但尚未被 dispatcher 取出执行的原因位掩码。
    // 可同时包含 EVENT / DEADLINE / INTERVAL / MANUAL 的任意组合。
    SchedulerRunReason pending_reason;

    uint8_t            registered;          // 槽位已被 RegisterTask 占用
    uint8_t            enabled;             // 使能标志
    uint8_t            in_progress;         // 任务 callback 正在执行（防同一任务重入）
    // scheduled = 1 表示 next_run_ms 当前有效（已设置了一个未来时间点）。
    // 不等于"任务已经 ready"；ready 由 pending_reason != NONE 决定。
    uint8_t            scheduled;

    SchedulerTaskStats stats;               // 运行时统计（累计值）
};

// 调度器全局状态
static const SchedulerPort *scheduler_port = nullptr;           // 平台适配回调表
static bool scheduler_initialized = false;                      // Init() 是否成功
static TaskSlot scheduler_tasks[SCHEDULER_MAX_TASKS];           // 静态任务槽位数组
static uint8_t scheduler_task_count = 0;                        // 已注册任务数
static volatile uint32_t scheduler_pending_events = 0;          // 全局待处理事件位图（ISR 可写）

// ############################################################################
// Generic WorkQueue-like Scheduler Core — 内部 helper
// ############################################################################

/**
 * @brief  wrap-safe 时间比较：处理 32-bit 毫秒回绕。
 * @param  now    当前时刻。
 * @param  target 目标时刻。
 * @retval true  now 已经到达或超过 target（考虑回绕）。
 */
static bool TimeReached(const uint32_t now, const uint32_t target)
{
    return static_cast<int32_t>(now - target) >= 0;
}

/**
 * @brief  在临界区内原子取出并清空全局 pending events 位图。
 * @retval 取出的 events 位图（调用后全局位图已清零）。
 */
static SchedulerEventMask TakePendingEvents()
{
    const SchedulerCriticalState state = scheduler_port->enter_critical();
    const SchedulerEventMask events = scheduler_pending_events;
    scheduler_pending_events = 0u;
    scheduler_port->exit_critical(state);
    return events;
}

/**
 * @brief  检查 task_id 是否合法（已注册且槽位在范围内）。
 * @retval true  task_id 有效。
 */
static bool TaskIdValid(const SchedulerTaskId task_id)
{
    return task_id < scheduler_task_count && scheduler_tasks[task_id].registered != 0u;
}

/**
 * @brief  将全局 events 位图匹配到所有订阅任务的 pending_events / pending_reason。
 * @param  events  已从全局位图取出的本周期事件掩码。
 * @note   调用方必须持有 scheduler critical section。
 *         只设置 EVENT reason 和 pending_events，不产生 DEADLINE/INTERVAL/MANUAL。
 */
static void DispatchEventsToTasks(const SchedulerEventMask events)
{
    if (events == 0u) {
        return;
    }

    for (uint8_t i = 0; i < scheduler_task_count; ++i) {
        TaskSlot &slot = scheduler_tasks[i];

        if (slot.registered == 0u || slot.enabled == 0u) {
            continue;
        }

        const SchedulerEventMask matched = events & slot.event_mask;

        if (matched != 0u) {
            slot.pending_reason |= SCHEDULER_REASON_EVENT;
            slot.pending_events  |= matched;
        }
    }
}

/**
 * @brief  在临界区内完成一次 callback 返回后的收尾。
 * @param  slot         已执行的任务槽位。
 * @param  reason       本次运行原因位掩码。
 * @param  run_start_ms 本次运行开始时的调度器毫秒时间。
 * @param  run_start_us 本次运行开始前的微秒时间。
 * @param  run_end_us   callback 返回后的微秒时间。
 *
 * @note   调用方必须已持有 scheduler critical section。callback 已在临界区外执行完毕；
 *         本函数负责更新运行统计并释放 in_progress，二者属于同一个 callback 收尾事务。
 */
static void UpdateTaskStatsLocked(TaskSlot &slot, const SchedulerRunReason reason,
                                   const uint32_t run_start_ms, const uint64_t run_start_us,
                                   const uint64_t run_end_us)
{
    uint32_t runtime_us;
    if (run_end_us >= run_start_us) { runtime_us = static_cast<uint32_t>(run_end_us - run_start_us); }
    else                            { runtime_us = 0u; }

    ++slot.stats.run_count;
    slot.stats.last_runtime_us = runtime_us;
    slot.stats.last_run_ms = run_start_ms;
    slot.stats.last_run_us = run_start_us;

    if ((reason & SCHEDULER_REASON_EVENT) != 0u)    { ++slot.stats.event_run_count; }
    if ((reason & SCHEDULER_REASON_DEADLINE) != 0u) { ++slot.stats.deadline_run_count; }
    if ((reason & SCHEDULER_REASON_INTERVAL) != 0u) { ++slot.stats.interval_run_count; }
    if ((reason & SCHEDULER_REASON_MANUAL) != 0u)   { ++slot.stats.manual_run_count; }

    if (runtime_us > slot.stats.max_runtime_us) { slot.stats.max_runtime_us = runtime_us; }
    if (runtime_us < slot.stats.min_runtime_us) { slot.stats.min_runtime_us = runtime_us; }

    slot.in_progress = 0u;
}

// ============================================================================
// 应用层初始化区域（Legacy — 阶段 7 移除）
// ============================================================================

/**
 * @brief  Legacy 应用层初始化占位（阶段 7 移除）。
 * @note   不属于调度器自身的任务表初始化；Scheduler_Setup() 在调度器准备完成后调用。
 */
void Scheduler_AppSetup()
{
    // 当前没有需要集中初始化的应用模块。
}

// ============================================================================
// 响应式 / 高优先级事件调度区域（Legacy — 阶段 7 移除）
// ============================================================================

// 高优先级事件位图。ISR 或 ISR adapter 只负责置位事件，不直接执行驱动、
// SPI 或其他耗时逻辑；实际处理统一由主循环中的 HighPriorityPoll() 完成。
volatile uint32_t scheduler_hp_events = 0u;

using sched_hp_handler_t = void (*)(void);

struct sched_hp_handler_entry_t
{
    uint32_t event;               // 事件位掩码（如 SCHED_HP_EVENT_IMU_DRDY）
    sched_hp_handler_t handler;   // 对应的处理函数（在普通上下文中调用）
};

// 静态事件处理表把事件位映射到普通上下文中的处理函数。后续新增响应式
// 事件时只需扩展表项，不需要把分发器重新改成多段硬编码判断。
const sched_hp_handler_entry_t scheduler_hp_handlers[] =
{
    {SCHED_HP_EVENT_IMU_DRDY, icm42688_service::Run},
};

constexpr uint8_t SCHED_HP_HANDLER_NUM =
    static_cast<uint8_t>(sizeof(scheduler_hp_handlers) / sizeof(scheduler_hp_handlers[0]));

/**
 * @brief  在临界区内原子取出并清空 legacy 高优事件位图（阶段 7 移除）。
 * @retval 取出的事件位图。
 * @note   "读取 + 清零"必须原子完成，避免 ISR 在中间插入投递新事件导致丢失。
 */
uint32_t Scheduler_TakeHighPriorityEvents()
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const uint32_t events = scheduler_hp_events;
    scheduler_hp_events = 0u;

    __set_PRIMASK(primask);
    return events;
}

/**
 * @brief  在普通上下文中分发 legacy 高优事件给 handler（阶段 7 移除）。
 * @note   函数内部的静态 polling 标志用于防止事件 handler 间接触发调度器时发生嵌套分发；
 *         由 Scheduler_Run() 在每次周期任务检查前后调用。
 */
void Scheduler_HighPriorityPoll()
{
    static uint8_t polling = 0u;

    if (polling != 0u) {
        return;
    }

    polling = 1u;
    const uint32_t events = Scheduler_TakeHighPriorityEvents();

    for (uint8_t index = 0u; index < SCHED_HP_HANDLER_NUM; ++index) {
        if ((events & scheduler_hp_handlers[index].event) != 0u) {
            scheduler_hp_handlers[index].handler();
        }
    }

    polling = 0u;
}

// ============================================================================
// 周期轮询任务调度区域（Legacy — 阶段 7 移除）
// ============================================================================

// 记录最近一次已打印的 IMU sample counter，避免 1 Hz 调试任务重复打印同一批数据。
uint32_t imu_last_sample_counter = 0u;

/**
 * @brief  1 Hz IMU delta 调试 print（阶段 7 迁移到低优 periodic task）。
 * @note   只读取 Service 缓存，不直接访问 SPI，不参与 ICM42688P 驱动状态机。
 */
void Print_ICM42688_Delta_Debug()
{
    icm42688_service::DeltaSample imu{};

    const ICM42688P::Status status = icm42688_service::GetDeltaLatest(&imu);

    if (status != ICM42688P::Status::Ok) {
        printf("[imu] st=%d\r\n", static_cast<int>(status));
        return;
    }

    if (imu.sample_counter == imu_last_sample_counter) {
        return; // 没有新数据时不重复打印。
    }

    imu_last_sample_counter = imu.sample_counter;

    if (imu.delta_time_s <= 0.0f) {
        printf("[imu] bad dt\r\n");
        return;
    }

    float gyro_rad_s[3];
    float force_m_s2[3];

    for (uint8_t i = 0; i < 3; i++) {
        gyro_rad_s[i] = imu.delta_angle_rad[i] / imu.delta_time_s;
        force_m_s2[i] = imu.delta_velocity_m_s[i] / imu.delta_time_s;
    }

    printf("t=%llu c=%lu n=%u dt=%.6f "
           "w=%.1f %.1f %.1f "
           "f=%.2f %.2f %.2f "
           "T=%.2f\r\n",
           (unsigned long long)imu.timestamp_us,
           (unsigned long)imu.sample_counter,
           (unsigned int)imu.delta_samples,
           imu.delta_time_s,
           gyro_rad_s[0] * 57.2957795f,
           gyro_rad_s[1] * 57.2957795f,
           gyro_rad_s[2] * 57.2957795f,
           force_m_s2[0],
           force_m_s2[1],
           force_m_s2[2],
           imu.temperature_deg_c);
}

// ── 周期任务回调（Legacy — 阶段 7 移除） ──
// 各频率任务由 Scheduler_Run() 按 interval_ticks 到期调用。均在普通上下文中执行。
// Loop_500Hz ~ Loop_10Hz 及 Loop_0_5Hz ~ Loop_0_1Hz 为预留占位槽，当前空函数。

/**
 * @brief  Legacy 1 kHz 周期 backup 路径（阶段 7 迁移）。
 * @note   即使 EXTI 事件暂时缺失，也能持续推进 IMU 初始化和 watchdog 状态机。
 */
void Loop_1000Hz()
{
    icm42688_service::Run();
}

/**
 * @brief  Legacy 500 Hz 占位槽。
 */
void Loop_500Hz()
{
}
/**
 * @brief  Legacy 250 Hz 占位槽。
 */
void Loop_250Hz()
{
}
/**
 * @brief  Legacy 200 Hz 占位槽。
 */
void Loop_200Hz()
{
}
/**
 * @brief  Legacy 100 Hz 占位槽。
 */
void Loop_100Hz()
{
}
/**
 * @brief  Legacy  50 Hz 占位槽。
 */
void Loop_50Hz()
{
}
/**
 * @brief  Legacy  20 Hz 占位槽。
 */
void Loop_20Hz()
{
}
/**
 * @brief  Legacy  10 Hz 占位槽。
 */
void Loop_10Hz()
{
}
/**
 * @brief  Legacy   5 Hz 占位槽。
 */
void Loop_5Hz()
{
}
/**
 * @brief  Legacy   4 Hz 占位槽。
 */
void Loop_4Hz()
{
}
/**
 * @brief  Legacy   2 Hz 占位槽。
 */
void Loop_2Hz()
{
}

/**
 * @brief  Legacy 1 Hz debug print wrapper（阶段 7 迁移）。
 */
void Loop_1Hz()
{
    Print_ICM42688_Delta_Debug();
}

/**
 * @brief  Legacy 0.5 Hz 占位槽。
 */
void Loop_0_5Hz()
{
}
/**
 * @brief  Legacy 0.2 Hz 占位槽。
 */
void Loop_0_2Hz()
{
}
/**
 * @brief  Legacy 0.1 Hz 占位槽。
 */
void Loop_0_1Hz()
{
}

// ── 周期任务静态配置表（Legacy — 阶段 7 移除） ──
// rate_hz 和任务顺序属于当前调度行为；Scheduler_Setup() 根据频率计算 interval_ticks。
sched_task_t sched_tasks[] =
{
    {Loop_1000Hz, 1000, 0, 0},
    {Loop_500Hz, 500, 0, 0},
    {Loop_250Hz, 250, 0, 0},
    {Loop_200Hz, 200, 0, 0},
    {Loop_100Hz, 100, 0, 0},
    {Loop_50Hz, 50, 0, 0},
    {Loop_20Hz, 20, 0, 0},
    {Loop_10Hz, 10, 0, 0},
    {Loop_5Hz, 5, 0, 0},
    {Loop_4Hz, 4, 0, 0},
    {Loop_2Hz, 2, 0, 0},
    {Loop_1Hz, 1, 0, 0},
    {Loop_0_5Hz, 0.5f, 0, 0},
    {Loop_0_2Hz, 0.2f, 0, 0},
    {Loop_0_1Hz, 0.1f, 0, 0},
};

constexpr size_t TASK_NUM = sizeof(sched_tasks) / sizeof(sched_tasks[0]);
} // namespace

// ############################################################################
// Generic WorkQueue-like Scheduler Core — extern "C" API 实现
// ############################################################################

// ── A. 初始化 ──

/**
 * @brief  初始化调度器：保存 port 回调并清空所有槽位。
 * @param  port  平台适配回调表；不可为 nullptr，且所有回调字段不可为空。
 * @note   在 main() 中调用一次，先于所有 Register/Schedule/RunOnce 操作。
 *         port 或必要回调为空时安全失败（置 initialized=false），不触发 assert。
 */
extern "C" void Scheduler_Init(const SchedulerPort *port)
{
    // port 和必要回调为空时安全失败
    if (port == nullptr
        || port->get_time_ms  == nullptr
        || port->get_time_us  == nullptr
        || port->enter_critical == nullptr
        || port->exit_critical  == nullptr) {
        scheduler_initialized = false;
        return;
    }

    scheduler_port = port;

    // 清空所有槽位
    for (uint8_t i = 0; i < SCHEDULER_MAX_TASKS; ++i) {
        scheduler_tasks[i] = TaskSlot{};
    }

    scheduler_task_count = 0u;
    scheduler_pending_events = 0u;
    scheduler_initialized = true;
}

// ── B. 任务注册 ──

/**
 * @brief  按完整配置注册一个任务（底层通用接口）。
 * @param  config  任务配置；fn 字段不可为 nullptr。
 * @retval 成功时返回任务 ID，失败时返回 SCHEDULER_TASK_ID_INVALID。
 * @note   只做 initialized / config / fn / capacity 基本检查。便捷注册接口在此基础上
 *         做更严格的参数语义检查。
 */
extern "C" SchedulerTaskId Scheduler_RegisterTask(const SchedulerTaskConfig *config)
{
    if (!scheduler_initialized
        || config == nullptr
        || config->fn == nullptr) {
        return SCHEDULER_TASK_ID_INVALID;
    }

    if (scheduler_task_count >= SCHEDULER_MAX_TASKS) {
        return SCHEDULER_TASK_ID_INVALID;
    }

    const SchedulerTaskId id = scheduler_task_count;
    TaskSlot &slot = scheduler_tasks[id];

    slot.name               = config->name;
    slot.fn                 = config->fn;
    slot.context            = config->context;
    slot.priority           = config->priority;
    slot.event_mask         = config->event_mask;
    slot.period_ms          = config->period_ms;
    slot.event_deadline_ms  = config->event_deadline_ms;
    slot.next_run_ms        = 0u;
    slot.pending_events     = 0u;
    slot.pending_reason     = SCHEDULER_REASON_NONE;
    slot.registered         = 1u;
    slot.enabled            = (config->enable_on_register != 0u) ? 1u : 0u;
    slot.in_progress        = 0u;
    slot.scheduled          = 0u;
    slot.stats              = SchedulerTaskStats{};
    slot.stats.min_runtime_us = UINT32_MAX;

    if (slot.enabled == 0u) {
        ++scheduler_task_count;
        return id;
    }

    const uint32_t now = scheduler_port->get_time_ms();

    // event + deadline 任务：设置初始 backup deadline
    if (config->event_deadline_ms != 0u) {
        slot.next_run_ms = now + config->event_deadline_ms;
        slot.scheduled = 1u;
    }

    // 周期任务：设置初始运行时间
    if (config->period_ms != 0u) {
        slot.next_run_ms = now + ((config->initial_delay_ms != 0u)
            ? config->initial_delay_ms : config->period_ms);
        slot.scheduled = 1u;
    }

    ++scheduler_task_count;
    return id;
}

/**
 * @brief  注册周期任务（便捷接口）。
 * @param  period_ms 周期时间，单位 ms；为 0 时拒绝注册。
 * @param  fn        任务回调，不可为 nullptr。
 * @return 成功时返回任务 ID，失败时返回 SCHEDULER_TASK_ID_INVALID。
 * @note   首次运行时间由 RegisterTask 内 initial_delay_ms 规则决定（默认等一个 period_ms）。
 *         该接口不订阅事件，不设置 event deadline。
 */
extern "C" SchedulerTaskId Scheduler_RegisterPeriodicTask(const char *name, const uint32_t period_ms,
                                                          const SchedulerTaskFn fn, void *context,
                                                          const SchedulerPriority priority)
{
    if (period_ms == 0u || fn == nullptr) { return SCHEDULER_TASK_ID_INVALID; }

    const SchedulerTaskConfig config = { name, fn, context, priority, 0u, period_ms, 0u, 0u, 1u };
    return Scheduler_RegisterTask(&config);
}

/**
 * @brief  注册事件任务（便捷接口）。
 * @param  event_mask 订阅的事件掩码；为 0 时拒绝注册。
 * @param  fn         任务回调，不可为 nullptr。
 * @return 成功时返回任务 ID，失败时返回 SCHEDULER_TASK_ID_INVALID。
 * @note   纯事件触发，无周期调度，无 deadline backup。
 */
extern "C" SchedulerTaskId Scheduler_RegisterEventTask(const char *name, const SchedulerEventMask event_mask,
                                                       const SchedulerTaskFn fn, void *context,
                                                       const SchedulerPriority priority)
{
    if (event_mask == 0u || fn == nullptr) { return SCHEDULER_TASK_ID_INVALID; }

    const SchedulerTaskConfig config = { name, fn, context, priority, event_mask, 0u, 0u, 0u, 1u };
    return Scheduler_RegisterTask(&config);
}

/**
 * @brief  注册事件 + deadline 任务（便捷接口）。
 * @param  event_mask  订阅的事件掩码；可为 0（此时仅靠 deadline 触发）。
 * @param  deadline_ms deadline 最大等待时间，ms；可为 0（此时仅靠 event 触发）。
 * @param  fn          任务回调，不可为 nullptr。
 * @return 成功时返回任务 ID，失败时返回 SCHEDULER_TASK_ID_INVALID。
 * @note   可作为纯 deadline task（event_mask==0）、纯 event task（deadline_ms==0）或两者兼备。
 *         event_mask 和 deadline_ms 同时为 0 时拒绝注册（无任何触发源）。
 */
extern "C" SchedulerTaskId Scheduler_RegisterEventDeadlineTask(const char *name, const SchedulerEventMask event_mask,
                                                               const uint32_t deadline_ms, const SchedulerTaskFn fn,
                                                               void *context, const SchedulerPriority priority)
{
    if (fn == nullptr)                         { return SCHEDULER_TASK_ID_INVALID; }
    if (event_mask == 0u && deadline_ms == 0u) { return SCHEDULER_TASK_ID_INVALID; }

    const SchedulerTaskConfig config = { name, fn, context, priority, event_mask, 0u, 0u, deadline_ms, 1u };
    return Scheduler_RegisterTask(&config);
}

// ── C. 事件投递 ──

/**
 * @brief  向调度器投递事件（普通上下文）。
 * @param  event_mask  事件掩码；为 0 时直接返回。
 * @note   只设置全局 pending event 位图，不执行任何 callback。
 *         匹配和分发由下一次 Scheduler_RunOnce() 完成。
 */
extern "C" void Scheduler_PostEvent(const SchedulerEventMask event_mask)
{
    if (!scheduler_initialized || event_mask == 0u) {
        return;
    }

    const SchedulerCriticalState state = scheduler_port->enter_critical();
    scheduler_pending_events |= event_mask;
    scheduler_port->exit_critical(state);
}

/**
 * @brief  向调度器投递事件（ISR 上下文）。
 * @param  event_mask  事件掩码；为 0 时直接返回。
 */
extern "C" void Scheduler_PostEventFromISR(const SchedulerEventMask event_mask)
{
    if (!scheduler_initialized || event_mask == 0u) {
        return;
    }

    // FromISR 版本与普通版本走同一 token-based critical path，保证 pending event
    // 位图更新与普通上下文一致。具体 enter/exit 行为由平台 port 决定；调度器
    // 核心不假设 ISR 中 PRIMASK 的固定取值。
    const SchedulerCriticalState state = scheduler_port->enter_critical();
    scheduler_pending_events |= event_mask;
    scheduler_port->exit_critical(state);
}

// ── D. 调度控制（普通上下文） ──

/**
 * @brief  立即调度任务运行（下一轮 RunOnce 执行）。
 * @note   设置 MANUAL reason 和 scheduled，清除旧 DEADLINE。不立即执行 callback。
 */
extern "C" void Scheduler_ScheduleNow(const SchedulerTaskId task_id)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return;
    }

    TaskSlot &slot = scheduler_tasks[task_id];

    const SchedulerCriticalState state = scheduler_port->enter_critical();
    slot.next_run_ms    = scheduler_port->get_time_ms();
    slot.scheduled      = 1u;
    // 清除旧 deadline（如果有），设置 MANUAL 表示"当前就运行"
    slot.pending_reason &= ~SCHEDULER_REASON_DEADLINE;
    slot.pending_reason |= SCHEDULER_REASON_MANUAL;
    scheduler_port->exit_critical(state);
}

/**
 * @brief  延迟调度任务（从当前时刻起 delay_ms 后运行）。
 * @note   只 arm 未来时间点（设置 scheduled + next_run_ms），清除旧 DEADLINE/MANUAL，
 *         不立即设置 REASON_DEADLINE。DEADLINE 由 RunOnce 在 TimeReached 后设置。
 */
extern "C" void Scheduler_ScheduleDelayed(const SchedulerTaskId task_id, const uint32_t delay_ms)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return;
    }

    TaskSlot &slot = scheduler_tasks[task_id];

    const SchedulerCriticalState state = scheduler_port->enter_critical();
    slot.next_run_ms = scheduler_port->get_time_ms() + delay_ms;
    slot.scheduled   = 1u;
    // 清除旧的 time-triggered pending 状态，避免旧 deadline/manual 导致提前触发。
    // 不立即设置 REASON_DEADLINE：由 Scheduler_RunOnce() 在 TimeReached 时设置。
    // 不碰 REASON_EVENT / REASON_INTERVAL：事件和周期语义独立于延迟调度。
    slot.pending_reason &= ~(SCHEDULER_REASON_DEADLINE | SCHEDULER_REASON_MANUAL);
    scheduler_port->exit_critical(state);
}

/**
 * @brief  在绝对时间点调度任务。
 * @note   语义同 ScheduleDelayed：只 arm 时间点，不立即设置 DEADLINE。
 */
extern "C" void Scheduler_ScheduleAt(const SchedulerTaskId task_id, const uint32_t target_ms)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return;
    }

    TaskSlot &slot = scheduler_tasks[task_id];

    const SchedulerCriticalState state = scheduler_port->enter_critical();
    slot.next_run_ms = target_ms;
    slot.scheduled   = 1u;
    // 清除旧的 time-triggered pending 状态，避免旧 deadline/manual 导致提前触发。
    // 不立即设置 REASON_DEADLINE：由 Scheduler_RunOnce() 在 TimeReached 时设置。
    // 不碰 REASON_EVENT / REASON_INTERVAL。
    slot.pending_reason &= ~(SCHEDULER_REASON_DEADLINE | SCHEDULER_REASON_MANUAL);
    scheduler_port->exit_critical(state);
}

/**
 * @brief  设置或修改任务的周期调度参数。
 * @param  interval_ms       周期时间，ms；为 0 时拒绝。
 * @param  initial_delay_ms  首次延迟；0 表示等一个 interval_ms。
 * @note   清除旧 INTERVAL pending 以避免上一轮周期立即触发。
 */
extern "C" void Scheduler_ScheduleOnInterval(const SchedulerTaskId task_id,
                                             const uint32_t interval_ms,
                                             const uint32_t initial_delay_ms)
{
    if (!scheduler_initialized || !TaskIdValid(task_id) || interval_ms == 0u) {
        return;
    }

    TaskSlot &slot = scheduler_tasks[task_id];

    const SchedulerCriticalState state = scheduler_port->enter_critical();
    slot.period_ms  = interval_ms;
    slot.next_run_ms = scheduler_port->get_time_ms() + ((initial_delay_ms != 0u) ? initial_delay_ms : interval_ms);
    slot.scheduled  = 1u;
    // 清理旧的 INTERVAL pending，避免上一轮周期立即触发。不碰 EVENT/DEADLINE/MANUAL 语义。
    slot.pending_reason &= ~SCHEDULER_REASON_INTERVAL;
    scheduler_port->exit_critical(state);
}

/**
 * @brief  取消所有 time-triggered 调度（清除 DEADLINE/MANUAL reason 和 scheduled）。
 * @note   不清除 EVENT/INTERVAL reason 和 pending_events。
 */
extern "C" void Scheduler_Cancel(const SchedulerTaskId task_id)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return;
    }

    TaskSlot &slot = scheduler_tasks[task_id];

    const SchedulerCriticalState state = scheduler_port->enter_critical();
    slot.scheduled      = 0u;
    slot.pending_reason &= ~(SCHEDULER_REASON_DEADLINE | SCHEDULER_REASON_MANUAL);
    // 不清 event/interrupt reason —— 它们是由事件投递置位的，Cancel 不管事件
    // 不清 pending_events —— 事件数据独立于调度目标时间
    scheduler_port->exit_critical(state);
}

// ── E. 调度控制（ISR 上下文） ──

/**
 * @brief  立即调度任务运行（ISR 上下文版本）。语义同普通版本。
 */
extern "C" void Scheduler_ScheduleNowFromISR(const SchedulerTaskId task_id)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return;
    }

    TaskSlot &slot = scheduler_tasks[task_id];

    const SchedulerCriticalState state = scheduler_port->enter_critical();
    slot.next_run_ms    = scheduler_port->get_time_ms();
    slot.scheduled      = 1u;
    slot.pending_reason &= ~SCHEDULER_REASON_DEADLINE;
    slot.pending_reason |= SCHEDULER_REASON_MANUAL;
    scheduler_port->exit_critical(state);
}

/**
 * @brief  延迟调度任务（ISR 上下文版本）。语义同普通版本。
 */
extern "C" void Scheduler_ScheduleDelayedFromISR(const SchedulerTaskId task_id, const uint32_t delay_ms)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return;
    }

    TaskSlot &slot = scheduler_tasks[task_id];

    const SchedulerCriticalState state = scheduler_port->enter_critical();
    slot.next_run_ms = scheduler_port->get_time_ms() + delay_ms;
    slot.scheduled   = 1u;
    slot.pending_reason &= ~(SCHEDULER_REASON_DEADLINE | SCHEDULER_REASON_MANUAL);
    scheduler_port->exit_critical(state);
}

/**
 * @brief  在绝对时间点调度任务（ISR 上下文版本）。语义同普通版本。
 */
extern "C" void Scheduler_ScheduleAtFromISR(const SchedulerTaskId task_id, const uint32_t target_ms)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return;
    }

    TaskSlot &slot = scheduler_tasks[task_id];

    const SchedulerCriticalState state = scheduler_port->enter_critical();
    slot.next_run_ms = target_ms;
    slot.scheduled   = 1u;
    slot.pending_reason &= ~(SCHEDULER_REASON_DEADLINE | SCHEDULER_REASON_MANUAL);
    scheduler_port->exit_critical(state);
}

/**
 * @brief  取消所有 time-triggered 调度（ISR 上下文版本）。语义同普通版本。
 */
extern "C" void Scheduler_CancelFromISR(const SchedulerTaskId task_id)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return;
    }

    TaskSlot &slot = scheduler_tasks[task_id];

    const SchedulerCriticalState state = scheduler_port->enter_critical();
    slot.scheduled      = 0u;
    slot.pending_reason &= ~(SCHEDULER_REASON_DEADLINE | SCHEDULER_REASON_MANUAL);
    scheduler_port->exit_critical(state);
}

// ── F. 启用 / 禁用 ──

/**
 * @brief  启用任务：允许 RunOnce 调度该任务。
 * @note   不恢复之前的 pending reason/events，只设 enabled 标志。
 */
extern "C" void Scheduler_EnableTask(const SchedulerTaskId task_id)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return;
    }

    const SchedulerCriticalState state = scheduler_port->enter_critical();
    scheduler_tasks[task_id].enabled = 1u;
    scheduler_port->exit_critical(state);
}

/**
 * @brief  禁用任务：清除所有 pending 状态并从 RunOnce 调度中排除。
 */
extern "C" void Scheduler_DisableTask(const SchedulerTaskId task_id)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return;
    }

    TaskSlot &slot = scheduler_tasks[task_id];

    const SchedulerCriticalState state = scheduler_port->enter_critical();
    slot.enabled        = 0u;
    slot.pending_reason = SCHEDULER_REASON_NONE;
    slot.pending_events = 0u;
    slot.scheduled      = 0u;
    scheduler_port->exit_critical(state);
}

/**
 * @brief  查询任务是否已启用。
 * @retval 0 = 禁用或无效 task_id，非 0 = 启用。
 */
extern "C" uint8_t Scheduler_IsTaskEnabled(const SchedulerTaskId task_id)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return 0u;
    }

    const SchedulerCriticalState state = scheduler_port->enter_critical();
    const uint8_t result = scheduler_tasks[task_id].enabled;
    scheduler_port->exit_critical(state);
    return result;
}

// ── G. 调度器入口 ──

/**
 * @brief  运行一个调度周期（在 main while(1) 中反复调用）
 *
 * @note   调度语义：
 *         1. ISR/普通上下文投递的事件先汇总到全局 pending event mask；
 *         2. RunOnce 原子取出事件并分发到各订阅任务的 pending_events / pending_reason；
 *         3. 时间触发（delayed/deadline/interval）只在 TimeReached 到期后转成 pending_reason，
 *            不会提前触发；
 *         4. dispatcher 按优先级（High → Normal → Low）执行 pending_reason 非 NONE 的任务；
 *         5. 任务 callback 在退出临界区后执行，不关中断运行用户代码；
 *         6. callback 运行期间新投递的同任务事件不会重入，pending 状态保留到下一轮，
 *            reentry_block_count++；
 *         7. 不做 catch-up 补跑：周期任务错过一次就按下一周期继续。
 */
extern "C" void Scheduler_RunOnce(void)
{
    if (!scheduler_initialized) {
        return;
    }

    const uint32_t now_ms = scheduler_port->get_time_ms();
    const uint64_t now_us = scheduler_port->get_time_us();

    // 1. 原子取出全局 pending events 并分发到各订阅任务。
    {
        const SchedulerEventMask events = TakePendingEvents();
        const SchedulerCriticalState state = scheduler_port->enter_critical();
        DispatchEventsToTasks(events);
        scheduler_port->exit_critical(state);
    }

    // 2. 检查各任务 delayed / deadline / interval 到期，设置 pending_reason。
    {
        const SchedulerCriticalState state = scheduler_port->enter_critical();

        for (uint8_t i = 0; i < scheduler_task_count; ++i) {
            TaskSlot &slot = scheduler_tasks[i];

            if (slot.registered == 0u || slot.enabled == 0u) {
                continue;
            }

            // 周期任务到期 → REASON_INTERVAL
            if (slot.period_ms != 0u && slot.scheduled != 0u && TimeReached(now_ms, slot.next_run_ms)) {
                slot.pending_reason |= SCHEDULER_REASON_INTERVAL;
            }

            // 非周期任务的 time-triggered 到期（delayed / deadline / event_deadline backup）
            // REASON_MANUAL 已由 ScheduleNow 设置，此处不覆盖。
            if (slot.period_ms == 0u && slot.scheduled != 0u && TimeReached(now_ms, slot.next_run_ms)
                && (slot.pending_reason & SCHEDULER_REASON_MANUAL) == 0u) {
                slot.pending_reason |= SCHEDULER_REASON_DEADLINE;
            }
        }

        scheduler_port->exit_critical(state);
    }

    // 3. 按优先级依次执行 ready 任务（同优先级内按注册顺序）。
    for (int prio = SCHEDULER_PRIORITY_HIGH; prio >= SCHEDULER_PRIORITY_LOW; --prio) {
        for (uint8_t i = 0; i < scheduler_task_count; ++i) {
            TaskSlot &slot = scheduler_tasks[i];

            if (slot.registered == 0u || slot.enabled == 0u) {
                continue;
            }

            SchedulerRunReason reason;
            SchedulerEventMask events;
            bool should_run = false;

            {
                const SchedulerCriticalState state = scheduler_port->enter_critical();

                // 先检查是否有 pending work + priority 匹配，再检查重入。
                // 这样 reentry_block_count 只在"确实有工作但被 in_progress 挡住"时才增加。
                if (slot.pending_reason == SCHEDULER_REASON_NONE
                    || static_cast<int>(slot.priority) != prio) {
                    scheduler_port->exit_critical(state);
                    continue;
                }

                if (slot.in_progress != 0u) {
                    ++slot.stats.reentry_block_count;
                    scheduler_port->exit_critical(state);
                    continue;
                }

                // 取出 pending 状态并标记 in_progress
                reason = slot.pending_reason;
                events = slot.pending_events;
                should_run = true;

                slot.pending_reason = SCHEDULER_REASON_NONE;
                slot.pending_events = 0u;
                slot.in_progress = 1u;

                // 周期任务：安排下一次运行时间（不做 catch-up 补跑）
                if ((reason & SCHEDULER_REASON_INTERVAL) != 0u && slot.period_ms != 0u) {
                    slot.next_run_ms = now_ms + slot.period_ms;
                    slot.scheduled = 1u;
                }

                // event_deadline 任务：重置下一次 backup deadline
                if (slot.event_deadline_ms != 0u) {
                    slot.next_run_ms = now_ms + slot.event_deadline_ms;
                    slot.scheduled = 1u;
                }

                // 非周期且非 deadline 任务的一次性调度 → 清除 scheduled
                if (slot.period_ms == 0u && slot.event_deadline_ms == 0u
                    && (reason & (SCHEDULER_REASON_DEADLINE | SCHEDULER_REASON_MANUAL)) != 0u) {
                    slot.scheduled = 0u;
                }

                scheduler_port->exit_critical(state);
            }

            // 4. 在临界区外执行任务 callback。
            if (should_run) {
                const uint64_t before_us = scheduler_port->get_time_us();
                slot.fn(reason, events, now_ms, now_us, slot.context);
                const uint64_t after_us = scheduler_port->get_time_us();

                // 5. 在临界区内统一收尾：更新统计并释放 in_progress。
                const SchedulerCriticalState state = scheduler_port->enter_critical();
                UpdateTaskStatsLocked(slot, reason, now_ms, before_us, after_us);
                scheduler_port->exit_critical(state);
            }
        }
    }
}

// ── H. 运行时统计查询 ──

/**
 * @brief  读取任务的运行时统计（临界区内快照拷贝）。
 * @param  stats  输出缓冲区；为 nullptr 时返回 0。
 * @retval 1 = 成功，0 = 参数无效。
 */
extern "C" uint8_t Scheduler_GetTaskStats(const SchedulerTaskId task_id, SchedulerTaskStats *stats)
{
    if (!scheduler_initialized || !TaskIdValid(task_id) || stats == nullptr) {
        return 0u;
    }

    const SchedulerCriticalState state = scheduler_port->enter_critical();
    *stats = scheduler_tasks[task_id].stats;
    scheduler_port->exit_critical(state);
    return 1u;
}

/**
 * @brief  清零单个任务的运行时统计。
 */
extern "C" void Scheduler_ClearTaskStats(const SchedulerTaskId task_id)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return;
    }

    const SchedulerCriticalState state = scheduler_port->enter_critical();
    scheduler_tasks[task_id].stats = SchedulerTaskStats{};
    scheduler_tasks[task_id].stats.min_runtime_us = UINT32_MAX;
    scheduler_port->exit_critical(state);
}

/**
 * @brief  清零所有任务的运行时统计。
 */
extern "C" void Scheduler_ClearAllTaskStats(void)
{
    if (!scheduler_initialized) {
        return;
    }

    const SchedulerCriticalState state = scheduler_port->enter_critical();

    for (uint8_t i = 0; i < scheduler_task_count; ++i) {
        scheduler_tasks[i].stats = SchedulerTaskStats{};
        scheduler_tasks[i].stats.min_runtime_us = UINT32_MAX;
    }

    scheduler_port->exit_critical(state);
}

/**
 * @brief  返回当前已注册任务数（临界区内读取）。
 */
extern "C" uint8_t Scheduler_GetTaskCount(void)
{
    if (!scheduler_initialized) {
        return 0u;
    }

    const SchedulerCriticalState state = scheduler_port->enter_critical();
    const uint8_t count = scheduler_task_count;
    scheduler_port->exit_critical(state);
    return count;
}

// ############################################################################
// Legacy Compatibility Scheduler — extern "C" API（阶段 7 移除）
// ############################################################################

/**
 * @brief  ISR 级事件投递入口：原子置位高优先级事件位图
 * @param  event 高优先级事件位（如 SCHED_HP_EVENT_IMU_DRDY）
 *
 * @note   仅操作位图，不在 ISR 上下文执行 SPI、FIFO 或 printf。
 *         实际处理由 Scheduler_HighPriorityPoll() 在普通上下文中完成。
 *         可在 ISR 中安全调用。
 */
extern "C" void Scheduler_PostHighPriorityEventFromISR(const uint32_t event)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    scheduler_hp_events |= event;
    __set_PRIMASK(primask);
}

/**
 * @brief  初始化调度器：计算周期任务 tick 间隔，并调用应用层初始化入口
 *
 * @note   Scheduler_AppSetup() 是独立的应用层初始化扩展点，与调度器自身设置分离。
 *         在 main 中调用一次，先于 Scheduler_Run。
 */
extern "C" void Scheduler_Setup(void)
{
    for (size_t index = 0u; index < TASK_NUM; ++index) {
        sched_tasks[index].interval_ticks = TICK_PER_SECOND / sched_tasks[index].rate_hz;

        if (sched_tasks[index].interval_ticks < 1) {
            sched_tasks[index].interval_ticks = 1;
        }
    }

    Scheduler_AppSetup();
}

/**
 * @brief  cooperative 主循环入口，在 main while(1) 中反复调用
 *
 * @note   在每次任务检查前后插入 Scheduler_HighPriorityPoll()，
 *         使 ISR 投递的 IMU data-ready 事件尽快在普通上下文得到处理。
 *         周期任务按 TimeBase_Millis() + interval_ticks 判断是否到期。
 *
 *         调度顺序：
 *         1. 入循环时先 poll 一次，处理积压的高优事件；
 *         2. 对每个周期任务：poll → 检查到期 → poll → 执行 → poll；
 *         3. poll 的 polling 静态标志防止嵌套。
 */
extern "C" void Scheduler_Run(void)
{
    Scheduler_HighPriorityPoll();

    for (size_t index = 0u; index < TASK_NUM; ++index) {
        Scheduler_HighPriorityPoll();
        const uint32_t t_now = TimeBase_Millis();

        if (t_now - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks) {
            sched_tasks[index].last_run = t_now;
            Scheduler_HighPriorityPoll();
            sched_tasks[index].task_func();
            Scheduler_HighPriorityPoll();
        }
    }
}
