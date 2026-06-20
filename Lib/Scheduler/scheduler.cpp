/**
 * @file    scheduler.cpp
 * @brief   通用裸机协作式 WorkQueue-like 调度器实现
 *
 * @details
 * 调度器核心不依赖具体业务模块、外设驱动或平台 HAL。
 * 时间源与临界区能力通过 SchedulerPort 回调注入。
 */

#include "scheduler.h"

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
    SchedulerPriority  priority;            // 优先级（0 最高，255 最低）
    SchedulerEventMask event_mask;          // 本任务订阅的事件位掩码（0 = 不订阅事件）
    uint32_t           period_ms;           // 周期间隔（0 = 非周期任务）
    uint32_t           event_deadline_ms;   // event deadline 最大等待时间（0 = 无 deadline）

    // 下一个 time-triggered ready 的目标毫秒时间；用于 delayed / deadline / interval 到达判断。
    uint32_t           next_run_ms;
    // 已匹配到本任务但尚未交给 callback 的事件位集合。
    SchedulerEventMask pending_events;
    // 本任务已就绪但尚未被 dispatcher 取出执行的原因位掩码。
    // 可同时包含 EVENT / DEADLINE / INTERVAL / MANUAL 的任意组合。
    // EVENT / MANUAL 主要用于让 task ready，并传递给 callback / stats。
    // INTERVAL 表示 periodic time trigger，运行后按 period_ms re-arm。
    // DEADLINE 表示 non-periodic time trigger：
    //   - 对 Event+Deadline task，是 backup deadline，运行后按 event_deadline_ms re-arm；
    //   - 对 Manual/Delayed-only 或 Event task 的 ScheduleDelayed/ScheduleAt，
    //     是 one-shot delayed trigger，运行后清 scheduled。
    // ScheduleOnInterval() 只负责 INTERVAL reason bit。
    // ScheduleDelayed() / ScheduleAt() 只负责 DEADLINE reason bit。
    // 不同触发来源不应互相清除 pending_reason 中不属于自己的 bit。
    SchedulerRunReason pending_reason;

    // 任务从 not-ready 变为 ready 时记录该时刻，用于同优先级 tie-breaking。
    // 任务运行后清零，下一次 ready 时重新设置。
    uint32_t           ready_since_ms;

    uint8_t            registered;          // 槽位已被 RegisterTask 占用
    uint8_t            enabled;             // 使能标志
    uint8_t            in_progress;         // 任务 callback 正在执行（防同一任务重入）
    // scheduled = 1 表示 next_run_ms 当前有效（已设置了一个未来时间点）。
    // 不等于"任务已经 ready"；ready 由 pending_reason != NONE 决定。
    uint8_t            scheduled;

    SchedulerTaskStats stats;               // 运行时统计（累计值）
};

// 调度器全局状态
const SchedulerPort *scheduler_port = nullptr;                     // 平台适配回调表
bool scheduler_initialized = false;                                // Init() 是否成功
TaskSlot scheduler_tasks[SCHEDULER_MAX_TASKS];                     // 静态任务槽位数组
uint8_t scheduler_task_count = 0u;                                 // 已注册任务数
volatile SchedulerEventMask scheduler_pending_events = 0u;         // 全局待处理事件位图（ISR 可写）

// ############################################################################
// Generic WorkQueue-like Scheduler Core — 内部 helper
// ############################################################################

/**
 * @brief  wrap-safe 时间比较：处理 32-bit 毫秒回绕。
 * @param  now    当前时刻。
 * @param  target 目标时刻。
 * @retval true  now 已经到达或超过 target（考虑回绕）。
 */
bool TimeReached(const uint32_t now, const uint32_t target)
{
    return static_cast<int32_t>(now - target) >= 0;
}

/**
 * @brief  wrap-safe 时间比较，判断 a 是否早于 b。
 * @param  a  第一个时间戳（32-bit 毫秒）。
 * @param  b  第二个时间戳。
 * @retval true  a 早于 b（通过 int32_t 有符号差值处理 32-bit 回绕）。
 * @note   主要用于同优先级任务 ready_since_ms tie-break。
 */
bool TimeEarlier(const uint32_t a, const uint32_t b)
{
    return static_cast<int32_t>(a - b) < 0;
}

/**
 * @brief  task 从 not-ready 首次变为 ready 时记录 ready_since_ms。
 * @note   Locked：调用方必须持有 scheduler critical section。
 *         task 已经 ready 时不刷新 ready_since_ms（保留最早 ready 时刻），
 *         避免重复触发改变同优先级排序结果。
 */
void MarkTaskReadyLocked(TaskSlot &slot, const uint32_t now_ms)
{
    if (slot.pending_reason == SCHEDULER_REASON_NONE) {
        slot.ready_since_ms = now_ms;
    }
}

/**
 * @brief  如果 task 不再有任何 pending reason，清除 ready_since_ms。
 * @note   Locked：调用方必须持有 scheduler critical section。
 */
void ClearReadySinceIfNotReadyLocked(TaskSlot &slot)
{
    if (slot.pending_reason == SCHEDULER_REASON_NONE) {
        slot.ready_since_ms = 0u;
    }
}

/**
 * @brief  在临界区内原子取出并清空全局 pending events 位图。
 * @retval 取出的 events 位图（调用后全局位图已清零）。
 */
SchedulerEventMask TakePendingEvents()
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
bool TaskIdValid(const SchedulerTaskId task_id)
{
    return task_id < scheduler_task_count && scheduler_tasks[task_id].registered != 0u;
}

/**
 * @brief  将全局 events 位图匹配到所有订阅任务的 pending_events / pending_reason。
 * @param  events  已从全局位图取出的本周期事件掩码。
 * @note   调用方必须持有 scheduler critical section。
 *         只设置 EVENT reason 和 pending_events，不产生 DEADLINE/INTERVAL/MANUAL。
 *         任务从 no pending reason 首次变为 EVENT ready 时，记录 ready_since_ms。
 */
void DispatchEventsToTasks(const SchedulerEventMask events, const uint32_t now_ms)
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
            MarkTaskReadyLocked(slot, now_ms);
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
void UpdateTaskStatsLocked(TaskSlot &slot, const SchedulerRunReason reason,
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
        scheduler_port = nullptr;
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
 * @note   面向初始化阶段的静态任务注册，不支持调度运行期间动态注册。
 *         拒绝 period_ms 与 event_mask/event_deadline_ms 同时非 0。
 *         拒绝 event_deadline_ms 非 0 但 event_mask 为 0（不支持 pure deadline repeating task）。
 *         period_ms/event_mask/event_deadline_ms 全为 0 的 manual/delayed-only task 仍允许。
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

    // Periodic、Event、Event+Deadline 是互斥的调度类型。
    // 不支持 pure deadline repeating task（event_deadline_ms 非 0 时必须同时有 event_mask）。
    const bool has_period          = (config->period_ms != 0u);
    const bool has_event           = (config->event_mask != 0u);
    const bool has_event_deadline  = (config->event_deadline_ms != 0u);

    if (has_period && (has_event || has_event_deadline)) { return SCHEDULER_TASK_ID_INVALID; }
    if (has_event_deadline && !has_event)                 { return SCHEDULER_TASK_ID_INVALID; }
    // period/event/event_deadline 全为 0 的 manual/delayed-only task 仍允许注册。

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
    slot.ready_since_ms     = 0u;
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
 * @brief  注册事件 + deadline 任务。
 * @param  event_mask   主触发事件掩码；不可为 0。
 * @param  deadline_ms  event backup deadline，ms；不可为 0。
 * @param  fn           任务回调，不可为 nullptr。
 * @return 成功时返回任务 ID，失败时返回 SCHEDULER_TASK_ID_INVALID。
 * @note   event 是主触发源，deadline 是备份/兜底触发源。
 *         纯 event task 请使用 Scheduler_RegisterEventTask()。
 *         周期 task 请使用 Scheduler_RegisterPeriodicTask()。
 */
extern "C" SchedulerTaskId Scheduler_RegisterEventDeadlineTask(const char *name, const SchedulerEventMask event_mask,
                                                               const uint32_t deadline_ms, const SchedulerTaskFn fn,
                                                               void *context, const SchedulerPriority priority)
{
    if (fn == nullptr || event_mask == 0u || deadline_ms == 0u) { return SCHEDULER_TASK_ID_INVALID; }

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
 * @brief  PX4-like submit now：设置 MANUAL pending，下一轮 RunOnce 执行。
 * @note   不改写 next_run_ms，不取消 scheduled timer target。
 *         不清 DEADLINE/INTERVAL/EVENT/pending_events。不立即执行 callback。disabled task 下 no-op。
 */
extern "C" void Scheduler_ScheduleNow(const SchedulerTaskId task_id)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return;
    }

    TaskSlot &slot = scheduler_tasks[task_id];

    const SchedulerCriticalState state = scheduler_port->enter_critical();

    if (slot.enabled == 0u) {
        scheduler_port->exit_critical(state);
        return;
    }

    const uint32_t now_ms = scheduler_port->get_time_ms();
    MarkTaskReadyLocked(slot, now_ms);
    slot.pending_reason |= SCHEDULER_REASON_MANUAL;
    scheduler_port->exit_critical(state);
}

/**
 * @brief  retarget non-periodic task 的下一次 DEADLINE target。
 * @note   periodic task 被拒绝；对 event_deadline task retarget backup deadline，
 *         对其他 non-periodic task 设置一次性 delayed DEADLINE trigger。
 *         retarget 时只清旧 DEADLINE，保留 MANUAL/EVENT/pending_events。disabled task 下 no-op。
 */
extern "C" void Scheduler_ScheduleDelayed(const SchedulerTaskId task_id, const uint32_t delay_ms)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return;
    }

    TaskSlot &slot = scheduler_tasks[task_id];

    const SchedulerCriticalState state = scheduler_port->enter_critical();

    if (slot.enabled == 0u) {
        scheduler_port->exit_critical(state);
        return;
    }
    if (slot.period_ms != 0u) {
        scheduler_port->exit_critical(state);
        return;
    }

    const uint32_t now_ms = scheduler_port->get_time_ms();
    slot.next_run_ms = now_ms + delay_ms;
    slot.scheduled   = 1u;
    // 不立即设置 REASON_DEADLINE：由 Scheduler_RunOnce() 在 TimeReached 时设置。
    // 不碰 REASON_MANUAL（ScheduleNow submit 的 work 不被 timer retarget 取消）。
    // 不碰 REASON_EVENT / REASON_INTERVAL。
    slot.pending_reason &= ~SCHEDULER_REASON_DEADLINE;
    ClearReadySinceIfNotReadyLocked(slot);
    scheduler_port->exit_critical(state);
}

/**
 * @brief  retarget non-periodic task 的下一次 DEADLINE target（绝对时间版本）。
 * @note   periodic task 被拒绝；event_deadline task retarget backup deadline；
 *         其他 non-periodic task 设置一次性 delayed DEADLINE trigger。
 *         只清旧 DEADLINE，保留 MANUAL/EVENT/pending_events。disabled task 下 no-op。
 */
extern "C" void Scheduler_ScheduleAt(const SchedulerTaskId task_id, const uint32_t target_ms)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return;
    }

    TaskSlot &slot = scheduler_tasks[task_id];

    const SchedulerCriticalState state = scheduler_port->enter_critical();

    if (slot.enabled == 0u) {
        scheduler_port->exit_critical(state);
        return;
    }
    if (slot.period_ms != 0u) {
        scheduler_port->exit_critical(state);
        return;
    }

    slot.next_run_ms = target_ms;
    slot.scheduled   = 1u;
    // 清除旧 DEADLINE pending，保留 MANUAL/EVENT/pending_events。
    // 不立即设置 REASON_DEADLINE：由 Scheduler_RunOnce() 在 TimeReached 时设置。
    // 不碰 REASON_EVENT / REASON_INTERVAL。
    slot.pending_reason &= ~SCHEDULER_REASON_DEADLINE;
    ClearReadySinceIfNotReadyLocked(slot);
    scheduler_port->exit_critical(state);
}

/**
 * @brief  设置或修改 periodic interval。
 * @param  interval_ms       周期时间，ms；为 0 时拒绝。
 * @param  initial_delay_ms  相位调整：0 表示等一个 interval_ms；非 0 表示首次延迟为 initial_delay_ms，之后按 interval_ms。
 * @note   只用于已注册的 periodic task；只清 INTERVAL pending，保留 MANUAL/EVENT/pending_events。
 *         不把 non-periodic 动态改成 periodic。
 *         event_deadline task 被拒绝。disabled task 下 no-op。
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

    if (slot.enabled == 0u) {
        scheduler_port->exit_critical(state);
        return;
    }
    // 只允许已注册为 periodic 的 task 调整 interval；不把 non-periodic 动态改成 periodic。
    if (slot.period_ms == 0u) {
        scheduler_port->exit_critical(state);
        return;
    }
    // 不支持 period + event_deadline 混用。
    if (slot.event_deadline_ms != 0u) {
        scheduler_port->exit_critical(state);
        return;
    }

    const uint32_t now_ms = scheduler_port->get_time_ms();
    slot.period_ms  = interval_ms;
    slot.next_run_ms = now_ms + ((initial_delay_ms != 0u) ? initial_delay_ms : interval_ms);
    slot.scheduled  = 1u;
    // 只清理旧 INTERVAL pending reason；保留 MANUAL/EVENT/pending_events。
    slot.pending_reason &= ~SCHEDULER_REASON_INTERVAL;
    ClearReadySinceIfNotReadyLocked(slot);
    scheduler_port->exit_critical(state);
}

/**
 * @brief  full clear 本 task slot 内部 pending/scheduled work。
 * @note   清除 scheduled、pending_reason、pending_events、ready_since_ms。
 *         不清全局 scheduler_pending_events；尚未被 RunOnce 分发的旧全局事件后续仍可能匹配。
 *         不改变 registered/enabled/config。未来新的 PostEvent/Schedule 仍可触发。
 *         如需长期阻止 task 运行，请用 Scheduler_DisableTask()。
 */
extern "C" void Scheduler_Cancel(const SchedulerTaskId task_id)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return;
    }

    TaskSlot &slot = scheduler_tasks[task_id];

    const SchedulerCriticalState state = scheduler_port->enter_critical();
    slot.scheduled      = 0u;
    slot.pending_reason  = SCHEDULER_REASON_NONE;
    slot.pending_events  = 0u;
    slot.ready_since_ms  = 0u;
    scheduler_port->exit_critical(state);
}

// ── E. 调度控制（ISR 上下文） ──

/**
 * @brief  PX4-like submit now（ISR 上下文版本）。语义同 Scheduler_ScheduleNow()。
 * @note   disabled task 下 no-op。
 */
extern "C" void Scheduler_ScheduleNowFromISR(const SchedulerTaskId task_id)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return;
    }

    TaskSlot &slot = scheduler_tasks[task_id];

    const SchedulerCriticalState state = scheduler_port->enter_critical();

    if (slot.enabled == 0u) {
        scheduler_port->exit_critical(state);
        return;
    }

    const uint32_t now_ms = scheduler_port->get_time_ms();
    MarkTaskReadyLocked(slot, now_ms);
    slot.pending_reason |= SCHEDULER_REASON_MANUAL;
    scheduler_port->exit_critical(state);
}

/**
 * @brief  retarget non-periodic task 的下一次 DEADLINE target（ISR 上下文版本）。
 * @note   语义同 Scheduler_ScheduleDelayed()。periodic task 被拒绝；event_deadline task retarget
 *         backup deadline；其他 non-periodic task 设置一次性 delayed DEADLINE trigger。
 *         只清旧 DEADLINE，保留 MANUAL/EVENT/pending_events。disabled task 下 no-op。
 */
extern "C" void Scheduler_ScheduleDelayedFromISR(const SchedulerTaskId task_id, const uint32_t delay_ms)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return;
    }

    TaskSlot &slot = scheduler_tasks[task_id];

    const SchedulerCriticalState state = scheduler_port->enter_critical();

    if (slot.enabled == 0u) {
        scheduler_port->exit_critical(state);
        return;
    }
    if (slot.period_ms != 0u) {
        scheduler_port->exit_critical(state);
        return;
    }

    const uint32_t now_ms = scheduler_port->get_time_ms();
    slot.next_run_ms = now_ms + delay_ms;
    slot.scheduled   = 1u;
    slot.pending_reason &= ~SCHEDULER_REASON_DEADLINE;
    ClearReadySinceIfNotReadyLocked(slot);
    scheduler_port->exit_critical(state);
}

/**
 * @brief  retarget non-periodic task 的下一次 DEADLINE target（ISR 绝对时间版本）。
 * @note   语义同 Scheduler_ScheduleAt()。periodic task 被拒绝；event_deadline task retarget backup
 *         deadline；其他 non-periodic task 设置一次性 delayed DEADLINE trigger。
 *         只清旧 DEADLINE，保留 MANUAL/EVENT/pending_events。disabled task 下 no-op。
 */
extern "C" void Scheduler_ScheduleAtFromISR(const SchedulerTaskId task_id, const uint32_t target_ms)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return;
    }

    TaskSlot &slot = scheduler_tasks[task_id];

    const SchedulerCriticalState state = scheduler_port->enter_critical();

    if (slot.enabled == 0u) {
        scheduler_port->exit_critical(state);
        return;
    }
    if (slot.period_ms != 0u) {
        scheduler_port->exit_critical(state);
        return;
    }

    slot.next_run_ms = target_ms;
    slot.scheduled   = 1u;
    slot.pending_reason &= ~SCHEDULER_REASON_DEADLINE;
    ClearReadySinceIfNotReadyLocked(slot);
    scheduler_port->exit_critical(state);
}

/**
 * @brief  full clear 当前 task 内部 pending/scheduled work（ISR 上下文版本）。
 * @note   语义同 Scheduler_Cancel()；同样不清全局 scheduler_pending_events。
 */
extern "C" void Scheduler_CancelFromISR(const SchedulerTaskId task_id)
{
    if (!scheduler_initialized || !TaskIdValid(task_id)) {
        return;
    }

    TaskSlot &slot = scheduler_tasks[task_id];

    const SchedulerCriticalState state = scheduler_port->enter_critical();
    slot.scheduled      = 0u;
    slot.pending_reason  = SCHEDULER_REASON_NONE;
    slot.pending_events  = 0u;
    slot.ready_since_ms  = 0u;
    scheduler_port->exit_critical(state);
}

// ── F. 启用 / 禁用 ──

/**
 * @brief  启用任务：只设 enabled 标志，不恢复之前清除的 pending/scheduled state。
 * @note   Event task 后续收到新 event 仍可触发。
 *         Periodic task 如需恢复周期调度，应调用 Scheduler_ScheduleOnInterval()。
 *         Event+Deadline task 如需恢复 backup deadline，应调用
 *         Scheduler_ScheduleDelayed() 或 Scheduler_ScheduleAt()。
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
 * @note   只清本 task slot 内部状态，不清全局 scheduler_pending_events。
 *         如果任务很快重新 enable，尚未被 RunOnce 分发的旧全局事件仍可能匹配该任务。
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
    slot.ready_since_ms = 0u;
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
 * @brief  在临界区内选择当前最优 ready task。
 * @param  dispatched_mask  本轮已经 dispatched 的 task 位掩码（防止同一 task 在单次 RunOnce 内自旋）。
 * @retval 最优 ready task 的 task_id；无 ready task 时返回 SCHEDULER_TASK_ID_INVALID。
 * @note   Locked：调用方必须持有 scheduler critical section。
 *         选择规则：priority 数值小者优先 → ready_since_ms 早者优先 → task_id 小者优先。
 *         跳过 registered==0、enabled==0、pending_reason==NONE、in_progress 的 task。
 *         遇到 in_progress 且 ready 的 task 时递增 reentry_block_count。
 */
SchedulerTaskId SelectBestReadyTaskLocked(const uint32_t dispatched_mask)
{
    bool best_found = false;
    SchedulerTaskId best_id = SCHEDULER_TASK_ID_INVALID;
    SchedulerPriority best_prio = 0u;
    uint32_t best_ready_ms = 0u;

    for (uint8_t i = 0; i < scheduler_task_count; ++i) {
        // 本轮已经 dispatched 的 task 不再选择，避免同一 task 在一次 RunOnce() 内自旋。
        if ((dispatched_mask & (static_cast<uint32_t>(1u) << i)) != 0u) { continue; }
        TaskSlot &slot = scheduler_tasks[i];

        // 只选择 registered、enabled、ready 且未 in_progress 的 task。
        if (slot.registered == 0u || slot.enabled == 0u) { continue; }
        if (slot.pending_reason == SCHEDULER_REASON_NONE) { continue; }
        if (slot.in_progress != 0u) {
            ++slot.stats.reentry_block_count;
            continue;
        }

        // 比较顺序：priority 数值更小者优先；同 priority 下 ready_since_ms 更早者优先；
        // 若 ready_since_ms 也相同，则 task_id 更小者优先。
        if (!best_found
            || slot.priority < best_prio
            || (slot.priority == best_prio && TimeEarlier(slot.ready_since_ms, best_ready_ms))
            || (slot.priority == best_prio && slot.ready_since_ms == best_ready_ms && i < best_id)) {
            best_id        = i;
            best_found     = true;
            best_prio      = slot.priority;
            best_ready_ms  = slot.ready_since_ms;
        }
    }

    return best_id;
}

/**
 * @brief  运行一个调度周期（在 main while(1) 中反复调用）
 *
 * @note   调度语义：
 *         1. ISR/普通上下文投递的事件先汇总到全局 pending event mask；
 *         2. RunOnce 原子取出事件并分发到各订阅任务的 pending_events / pending_reason；
 *         3. 时间触发（delayed/deadline/interval）只在 TimeReached 到期后转成 pending_reason，
 *            不会提前触发；
 *         4. dispatcher 按数字 priority、ready_since_ms、task_id 的确定性规则选择 ready task；
 *         5. 任务 callback 在退出临界区后执行，不关中断运行用户代码；
 *         6. callback 运行期间产生的新 pending 状态不会导致同一 task 在本轮重入；
 *            本轮已执行 task 会被 dispatched_mask 跳过，同 task pending 保留到下一次 RunOnce；
 *            reentry_block_count 表示选择器遇到 in_progress 且 ready 的 task 时拦截次数；
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
        DispatchEventsToTasks(events, now_ms);
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
                MarkTaskReadyLocked(slot, now_ms);
                slot.pending_reason |= SCHEDULER_REASON_INTERVAL;
            }

            // 非周期任务的 time-triggered 到期（delayed / deadline / event_deadline backup）
            if (slot.period_ms == 0u && slot.scheduled != 0u && TimeReached(now_ms, slot.next_run_ms)) {
                MarkTaskReadyLocked(slot, now_ms);
                slot.pending_reason |= SCHEDULER_REASON_DEADLINE;
            }
        }

        scheduler_port->exit_critical(state);
    }

    // 3. 按优先级选择最优 ready task 并执行。
    //    每个 task 在本轮最多执行一次（dispatched_mask 防止同 task 自旋）。
    //    选择规则：priority 小者优先；同 priority 则 ready_since_ms 早者优先；同 ready_since_ms 则 task_id 小者优先。
    uint32_t dispatched_mask = 0u;

    while (true) {
        SchedulerTaskId best_id;
        SchedulerRunReason reason;
        SchedulerEventMask events;
        bool should_run = false;

        {
            const SchedulerCriticalState state = scheduler_port->enter_critical();
            best_id = SelectBestReadyTaskLocked(dispatched_mask);

            if (best_id == SCHEDULER_TASK_ID_INVALID) {
                scheduler_port->exit_critical(state);
                break;
            }

            dispatched_mask |= (static_cast<uint32_t>(1u) << best_id);
            TaskSlot &slot = scheduler_tasks[best_id];

            // 取出 pending 状态并标记 in_progress
            reason = slot.pending_reason;
            events = slot.pending_events;
            should_run = true;

            slot.pending_reason = SCHEDULER_REASON_NONE;
            slot.pending_events = 0u;
            slot.ready_since_ms = 0u;
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

            // pure one-shot task 因 DEADLINE 运行后才清除 scheduled；MANUAL run 不取消 timer target。
            if (slot.period_ms == 0u && slot.event_deadline_ms == 0u
                && (reason & SCHEDULER_REASON_DEADLINE) != 0u) {
                slot.scheduled = 0u;
            }

            scheduler_port->exit_critical(state);
        }

        // 4. 在临界区外执行任务 callback。
        if (should_run) {
            TaskSlot &slot = scheduler_tasks[best_id];
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
