/**
 * @file    scheduler.h
 * @brief   通用裸机协作式 WorkQueue-like 调度器接口（C ABI）
 *
 * @details
 * 新 Scheduler 参考 PX4 WorkQueue / ScheduledWorkItem 思想，但不引入 RTOS
 * 线程、独立任务栈、抢占、时间片、阻塞等待、信号量、互斥锁、动态内存。
 *
 * 调度器核心不依赖任何业务模块（ICM42688P、IMU、FIFO、EXIT、TimeBase、UART、
 * HAL_SPI、HAL_TIM 等）。时间和临界区能力通过 SchedulerPort 回调注入。
 *
 * 本头文件 C/C++ 均可 include。所有公开类型使用 C-compatible 声明。
 */

#ifndef ELECTROMAGNETICARTILLERY_SCHEDULER_H
#define ELECTROMAGNETICARTILLERY_SCHEDULER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 第一部分：通用 WorkQueue-like Scheduler 类型定义（C-compatible）
// ============================================================================

// ── 任务标识 ──
typedef uint8_t SchedulerTaskId;
#define SCHEDULER_TASK_ID_INVALID  0xFFu

// ── 事件掩码（最多 32 种事件） ──
typedef uint32_t SchedulerEventMask;

// ── 运行原因（bitmask，可同时设置多个原因） ──
typedef uint32_t SchedulerRunReason;
#define SCHEDULER_REASON_NONE      0u
#define SCHEDULER_REASON_EVENT     (1u << 0)
#define SCHEDULER_REASON_DEADLINE  (1u << 1)
#define SCHEDULER_REASON_INTERVAL  (1u << 2)
#define SCHEDULER_REASON_MANUAL    (1u << 3)

// ── 优先级（当前阶段三档） ──
typedef enum {
    SCHEDULER_PRIORITY_LOW    = 0,
    SCHEDULER_PRIORITY_NORMAL = 1,
    SCHEDULER_PRIORITY_HIGH   = 2,
} SchedulerPriority;

// ── 最大任务数（受 32-bit bookkeeping 限制） ──
#define SCHEDULER_MAX_TASKS 16u
#if SCHEDULER_MAX_TASKS > 32u
#error "SCHEDULER_MAX_TASKS must be <= 32 because scheduler ready/event bookkeeping uses 32-bit masks."
#endif

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
    SchedulerPriority  priority;              // 优先级
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

/**
 * @brief  初始化调度器
 * @param  port  平台适配回调表（不可为 NULL，必要回调不可为空函数指针）
 * @note   在 main() 中调用一次，先于所有 Register/Schedule/Run 操作。
 */
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

/**
 * @brief  运行一个调度周期
 * @note   在 main while(1) 中反复调用。内部取出 ready 任务并按优先级执行。
 */
void Scheduler_RunOnce(void);

// ── H. 运行时统计查询 ──

uint8_t Scheduler_GetTaskStats(SchedulerTaskId task_id, SchedulerTaskStats *stats);
void    Scheduler_ClearTaskStats(SchedulerTaskId task_id);
void    Scheduler_ClearAllTaskStats(void);
uint8_t Scheduler_GetTaskCount(void);

// ============================================================================
// 第三部分：Legacy Compatibility（阶段 7 移除）
// ============================================================================

// ── 旧高优先级事件位定义 ──
#define SCHED_HP_EVENT_IMU_DRDY (1u << 0)
#define TICK_PER_SECOND 1000

// ── 旧周期轮询任务表项 ──
typedef struct {
    void (*task_func)(void);
    float rate_hz;
    uint16_t interval_ticks;
    uint32_t last_run;
} sched_task_t;

// ── Legacy API（阶段 7 逐步移除） ──

void Scheduler_PostHighPriorityEventFromISR(uint32_t event);

#ifdef __cplusplus
}
#endif

#endif //ELECTROMAGNETICARTILLERY_SCHEDULER_H
