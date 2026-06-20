/**
 * @file    scheduler.h
 * @brief   通用裸机协作式 WorkQueue-like 调度器接口（C ABI）
 *
 * @details
 * 调度器核心不依赖具体业务模块、外设驱动或平台 HAL。
 * 时间和临界区能力通过 SchedulerPort 回调注入，核心代码不直接访问平台接口。
 *
 * C/C++ 均可 include。所有公开类型使用 C-compatible 声明。
 *
 * ============================================================================
 * 使用说明
 * ============================================================================
 *
 * 一、调度器性质
 *     这是裸机协作式 WorkQueue-like scheduler，不是 RTOS。
 *     不创建线程，不分配独立栈，不抢占正在运行的 task。
 *     callback 必须 run-to-completion，不能在 callback 内 while 等待或长时间阻塞。
 *
 * 二、main.c 典型调用顺序
 *     Scheduler_Init(&scheduler_port);
 *     App_Init();
 *     SchedulerAppTasks_RegisterAll();
 *     while (1) { Scheduler_RunOnce(); }
 *
 * 三、任务注册
 *     Scheduler_RegisterTask(&config) 是完整注册接口。
 *     Scheduler_RegisterPeriodicTask / RegisterEventTask / RegisterEventDeadlineTask 是便捷接口。
 *     推荐应用任务集中放在 scheduler_app_tasks.cpp 的 kAppTasks[] 注册表中。
 *     scheduler core 不应包含具体业务知识；业务任务应放在 app 层注册表中。
 *
 * 四、最大任务数与事件数
 *     最大任务容量由 SCHEDULER_MAX_TASKS 决定（当前 32）。
 *     合法 task id 为 0 ~ SCHEDULER_MAX_TASKS-1，invalid 为 0xFF。
 *     最大事件种类由 SchedulerEventMask 的 32-bit 位宽决定（当前 32 种）。
 *     任务数量和事件数量是不同概念。
 *     Scheduler_Init() 清空的是静态任务表容量，实际注册任务数由 scheduler_task_count 决定。
 *     未注册槽位不会被调度，但仍占用静态 RAM。
 *     要降低内存占用，应修改 SCHEDULER_MAX_TASKS 并重新编译。
 *
 * 五、priority 语义
 *     SchedulerPriority 是 uint8_t。0 最高，255 最低。数值越小越先运行。
 *     不提供固定 HIGH/NORMAL/LOW 宏，用户自行按系统需要分配数字。
 *
 * 六、同优先级选择规则
 *     1. priority 数值小者优先；
 *     2. priority 相同，ready_since_ms 更早者优先；
 *     3. ready_since_ms 也相同，task_id 更小者优先；
 *        在当前注册表顺序注册方式下，可理解为先注册者优先。
 *
 * 七、callback 统一参数
 *     void callback(SchedulerRunReason reason, SchedulerEventMask events,
 *                   uint32_t now_ms, uint64_t now_us, void *context);
 *     reason  — 本次运行原因 bitmask（可同时含 EVENT/DEADLINE/INTERVAL/MANUAL）。
 *     events — 本次匹配到的事件位（仅当 reason 含 EVENT 时非零）。
 *     now_ms / now_us — 调度器在本轮 RunOnce 中读取的时间戳。
 *     context — 注册时传入的用户上下文指针。
 *
 * 八、ISR 使用规则
 *     ISR 中优先使用 Scheduler_PostEventFromISR() 投递事件。
 *     ISR 中不应执行 SPI/FIFO/printf/长逻辑。
 *     Scheduler_ScheduleNowFromISR / ScheduleDelayedFromISR / ScheduleAtFromISR /
 *     CancelFromISR 也是 public API，只应在确实需要从 ISR 直接 arm/cancel 时使用。
 *     FromISR API 不执行 callback，只修改调度状态。
 *
 * 九、Schedule API 速览
 *     Scheduler_ScheduleNow / ScheduleNowFromISR:
 *         PX4-like submit now。设置 MANUAL pending。
 *         不改写 next_run_ms，不取消 scheduled timer target。
 *         不清 DEADLINE/INTERVAL/EVENT/pending_events。disabled task 下 no-op。
 *
 *     Scheduler_ScheduleDelayed / ScheduleAt / FromISR:
 *         只用于 non-periodic task。对 event_deadline task retarget backup deadline；
 *         对其他 non-periodic task 设置一次性 delayed DEADLINE trigger。
 *         不新增独立 timer。拒绝 periodic task。
 *         设置/retarget 时只清 DEADLINE，保留 MANUAL/EVENT/pending_events。disabled task 下 no-op。
 *
 *     Scheduler_ScheduleOnInterval:
 *         只用于已注册的 periodic task。设置或修改 period_ms。
 *         initial_delay_ms 用于调整下一次触发相位。
 *         只清 INTERVAL pending，保留 MANUAL/EVENT/pending_events。
 *         不把 non-periodic task 动态改成 periodic。拒绝 event_deadline task。disabled no-op。
 *
 *     Scheduler_Cancel / CancelFromISR:
 *         full clear 该 task 内部当前 pending/scheduled work。
 *         清 scheduled、pending_reason、pending_events、ready_since_ms。
 *         不改变 enabled/config。不阻止未来再次触发。
 *         不清尚未分发的全局 scheduler_pending_events bit。
 *
 * 十、启用/禁用与统计接口
 *     Scheduler_EnableTask / DisableTask / IsTaskEnabled 是任务运行控制接口，
 *     可由上层系统按需调用。
 *     Scheduler_GetTaskStats / ClearTaskStats / ClearAllTaskStats / GetTaskCount
 *     是调试和运行观测接口，不应在高频路径中频繁调用。
 *
 * 十一、任务触发类型
 *     自动触发类型包括 Periodic、Event、Event+Deadline，三者互斥。
 *     Periodic task 不应同时订阅 event 或 event_deadline。
 *     Event + Deadline task 要求 event_mask 和 deadline_ms 均非 0，
 *     使用 event 作为主触发源，deadline 作为备份/兜底触发源。
 *     不支持 pure deadline repeating task（event_deadline_ms 非 0 时 event_mask 必须非 0）。
 *     period_ms/event_mask/event_deadline_ms 均为 0 的任务不会自动触发，
 *     但可由 ScheduleNow/ScheduleDelayed/ScheduleAt 手动调度。
 *     Scheduler_RegisterTask() 拒绝 period_ms 与 event_mask/event_deadline_ms 混用，
 *     同时拒绝 event_deadline_ms 非 0 但 event_mask 为 0 的 pure deadline 配置。
 *     Scheduler_RegisterEventDeadlineTask() 要求 event_mask 和 deadline_ms 均非 0。
 *     ScheduleOnInterval() 只允许已注册的 periodic task 调整周期。
 *     ScheduleDelayed/ScheduleAt 拒绝 periodic task。
 *
 * 十二、内部实现边界
 *     内部数组、selection helper、stats 更新、pending state 等属于内部实现。
 *     用户不应直接访问内部数组或 helper。
 *     用户可用的 scheduler core API 为本文件声明的函数。
 *     app 层入口由 scheduler_app_tasks.h 和 scheduler_app_events.h 声明。
 */

#ifndef ELECTROMAGNETICARTILLERY_SCHEDULER_H
#define ELECTROMAGNETICARTILLERY_SCHEDULER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Scheduler 静态配置
// ============================================================================

// 静态任务表容量（编译期常量，不是已注册任务数）。
// 修改此值后需重新编译，未使用的槽位仍占用 RAM。
// 运行期 Scheduler_Init() 不接受任务数量参数，因为任务表是静态分配的。
#ifdef SCHEDULER_MAX_TASKS
#error "SCHEDULER_MAX_TASKS must be defined only in scheduler.h."
#endif
#define SCHEDULER_MAX_TASKS 32u
#if SCHEDULER_MAX_TASKS == 0u || SCHEDULER_MAX_TASKS > 32u
#error "SCHEDULER_MAX_TASKS must be in range 1..32 in this build."
#endif

// ============================================================================
// 第一部分：通用 WorkQueue-like Scheduler 类型定义（C-compatible）
// ============================================================================

// ── 任务标识 ──
typedef uint8_t SchedulerTaskId;
#define SCHEDULER_TASK_ID_INVALID  0xFFu

// ── 事件掩码（32-bit，最多 32 种事件） ──
typedef uint32_t SchedulerEventMask;

// ── 运行原因（bitmask，可同时设置多个原因） ──
typedef uint32_t SchedulerRunReason;
#define SCHEDULER_REASON_NONE      0u
#define SCHEDULER_REASON_EVENT     (1u << 0)
#define SCHEDULER_REASON_DEADLINE  (1u << 1)
#define SCHEDULER_REASON_INTERVAL  (1u << 2)
#define SCHEDULER_REASON_MANUAL    (1u << 3)

// ── 优先级：0 最高（最先运行），255 最低 ──
typedef uint8_t SchedulerPriority;

// ── Token-based 临界区 ──
typedef uint32_t SchedulerCriticalState;

typedef SchedulerCriticalState (*SchedulerEnterCriticalFn)(void);
typedef void                   (*SchedulerExitCriticalFn)(SchedulerCriticalState state);

// ── SchedulerPort 回调表（初始化时注入，生命周期 >= 调度器生命周期） ──
typedef struct {
    uint32_t (*get_time_ms)(void);
    uint64_t (*get_time_us)(void);
    SchedulerEnterCriticalFn enter_critical;
    SchedulerExitCriticalFn  exit_critical;
} SchedulerPort;

// ── Task callback 签名 ──
typedef void (*SchedulerTaskFn)(
    SchedulerRunReason reason,
    SchedulerEventMask events,
    uint32_t            now_ms,
    uint64_t            now_us,
    void               *context
);

// ── 任务注册配置 ──
typedef struct {
    const char        *name;                  // 调试名称（可为 NULL）
    SchedulerTaskFn    fn;                    // 任务回调（不可为 NULL）
    void              *context;               // 用户上下文
    SchedulerPriority  priority;              // 优先级（0=最高, 255=最低）
    SchedulerEventMask event_mask;            // 订阅的事件掩码（0 = 不订阅事件）
    uint32_t           period_ms;             // 周期调度间隔（0 = 不周期调度）
    uint32_t           initial_delay_ms;      // 首次周期调度延迟；0 表示使用 period_ms 作为首次延迟
    uint32_t           event_deadline_ms;     // event deadline 最大等待时间（0 = 无 deadline）
    uint8_t            enable_on_register;    // 注册后立即启用（非 0 = 启用）
} SchedulerTaskConfig;

// ── 运行时统计 ──
typedef struct {
    uint32_t run_count;               // 累计被调度次数
    uint32_t event_run_count;         // 其中因 event 触发的次数
    uint32_t deadline_run_count;      // 其中因 deadline/delay 触发的次数
    uint32_t interval_run_count;      // 其中因周期 interval 触发的次数
    uint32_t manual_run_count;        // 其中因 ScheduleNow 触发的次数
    uint32_t deadline_miss_count;     // 预留：deadline 到期但任务未及时运行次数，后续定义统计语义后启用
    uint32_t reentry_block_count;     // 任务正运行中被再次触发而拦截的次数
    uint32_t last_runtime_us;         // 最近一次执行时间，微秒
    uint32_t max_runtime_us;          // 单次最大执行时间，微秒
    uint32_t min_runtime_us;          // 单次最小执行时间，微秒
    uint32_t last_run_ms;             // 最近一次执行时刻，调度器毫秒时间戳
    uint64_t last_run_us;             // 最近一次执行时刻，调度器微秒时间戳
} SchedulerTaskStats;

// ============================================================================
// 第二部分：通用 WorkQueue-like Scheduler API
// ============================================================================

// ── A. 初始化 ──

void Scheduler_Init(const SchedulerPort *port);

// ── B. 任务注册 ──

SchedulerTaskId Scheduler_RegisterTask(const SchedulerTaskConfig *config);

SchedulerTaskId Scheduler_RegisterPeriodicTask(const char *name, uint32_t period_ms, SchedulerTaskFn fn, void *context, SchedulerPriority priority);
SchedulerTaskId Scheduler_RegisterEventTask(const char *name, SchedulerEventMask event_mask, SchedulerTaskFn fn, void *context, SchedulerPriority priority);
SchedulerTaskId Scheduler_RegisterEventDeadlineTask(const char *name, SchedulerEventMask event_mask, uint32_t deadline_ms, SchedulerTaskFn fn, void *context, SchedulerPriority priority);

// ── C. 事件投递 ──

void Scheduler_PostEvent(SchedulerEventMask event_mask);
void Scheduler_PostEventFromISR(SchedulerEventMask event_mask);

// ── D. 调度控制（普通上下文） ──

void Scheduler_ScheduleNow(SchedulerTaskId task_id);
void Scheduler_ScheduleDelayed(SchedulerTaskId task_id, uint32_t delay_ms);
void Scheduler_ScheduleAt(SchedulerTaskId task_id, uint32_t target_ms);
void Scheduler_ScheduleOnInterval(SchedulerTaskId task_id, uint32_t interval_ms, uint32_t initial_delay_ms);
void Scheduler_Cancel(SchedulerTaskId task_id);

// ── E. 调度控制（ISR 上下文） ──

void Scheduler_ScheduleNowFromISR(SchedulerTaskId task_id);
void Scheduler_ScheduleDelayedFromISR(SchedulerTaskId task_id, uint32_t delay_ms);
void Scheduler_ScheduleAtFromISR(SchedulerTaskId task_id, uint32_t target_ms);
void Scheduler_CancelFromISR(SchedulerTaskId task_id);

// ── F. 启用 / 禁用 ──

void    Scheduler_EnableTask(SchedulerTaskId task_id);
void    Scheduler_DisableTask(SchedulerTaskId task_id);
uint8_t Scheduler_IsTaskEnabled(SchedulerTaskId task_id);

// ── G. 调度器入口 ──

void Scheduler_RunOnce(void);

// ── H. 运行时统计查询 ──

uint8_t Scheduler_GetTaskStats(SchedulerTaskId task_id, SchedulerTaskStats *stats);
void    Scheduler_ClearTaskStats(SchedulerTaskId task_id);
void    Scheduler_ClearAllTaskStats(void);
uint8_t Scheduler_GetTaskCount(void);

#ifdef __cplusplus
}
#endif

#endif //ELECTROMAGNETICARTILLERY_SCHEDULER_H
