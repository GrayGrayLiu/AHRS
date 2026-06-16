# AHRS Project Instructions

## Project Overview

This project is a bare-metal STM32H723VGTX AHRS firmware project.

Development environment:

```text
Windows 11
CLion + CMake
STM32CubeMX generated HAL code
Bare-metal main loop, no RTOS
Target: AHRS.elf
```

The current active branch is usually:

```text
codex_icm42688p_hwtest_cleanup
```

The current work mainly focuses on the ICM42688P SPI IMU driver, scheduler, EXTI data-ready path, and later sensor fusion / INS modules.

## General Working Rules

Always work conservatively.

Before modifying code:

```text
1. Check git status.
2. Read the relevant files.
3. Explain the intended edit scope.
4. Modify only the files explicitly allowed by the user.
```

After modifying code:

```text
1. Run git diff --check.
2. Run git diff --name-only.
3. Build AHRS.elf.
4. Report modified files, build result, and git status.
```

Do not commit unless the user explicitly asks for a commit.

Never push unless the user explicitly asks for push.

## Build Command

Use this build command unless the user gives a different one:

```powershell
& 'C:\Users\Gray\AppData\Local\Programs\CLion\bin\cmake\win\x64\bin\cmake.exe' --build cmake-build-debug --target AHRS.elf -j 8
```

## File Modification Rules

Do not modify these files unless the user explicitly allows it:

```text
Core/Src/main.c
CMakeLists.txt
CMakeLists_template.txt
Drivers/
ThirdPartyLib/
Lib/TimeBase/
Lib/Timer/
Lib/ICM42688P_Template/
```

Do not modify STM32CubeMX generated code unless the user explicitly asks.

## ICM42688P Directory Rules

Current intended structure:

```text
Lib/ICM42688P/
├── ICM42688P_Registers.hpp
├── ICM42688P.hpp
├── ICM42688P.cpp
├── ICM42688_Service.hpp
└── ICM42688_Service.cpp
```

The register description file is protected:

```text
Lib/ICM42688P/ICM42688P_Registers.hpp
```

Do not modify it unless explicitly instructed.

The ICM42688P register initialization configuration table is protected.

Do not modify, reorder, reformat, rename, or optimize any register initialization table unless explicitly instructed.

Do not modify:

```text
FIFO watermark
FIFO packet format
20-bit FIFO reconstruction
ProcessGyro formula
ProcessAccel formula
delta_time_s algorithm
measured batch dt fallback threshold
coordinate transform sign
RunImpl state machine behavior
DataReady pending mechanism
ScheduleNow / ScheduleDelayed behavior
```

unless the user explicitly asks.

## Current ICM42688P Architecture

The current architecture is:

```text
EXTI ISR adapter
 -> icm42688_service::NotifyDataReadyFromISR(TimeBase_Micros())
 -> ICM42688P::DataReady(timestamp_us)
 -> Scheduler_PostHighPriorityEventFromISR(SCHED_HP_EVENT_IMU_DRDY)
 -> Scheduler_Run()
 -> Scheduler_HighPriorityPoll()
 -> icm42688_service::Run()
 -> ICM42688P::Update()
 -> ICM42688P::RunImpl()
 -> FIFO_READ
 -> ICM42688P::FIFORead(timestamp_us)
```

The 1 kHz scheduler path also calls:

```text
icm42688_service::Run()
```

This is currently a periodic backup / state-machine progress path. Do not change its frequency or location unless explicitly instructed.

## Recent Important Commits

Recent relevant commits include:

```text
3676687 Fold ICM42688 API layer into service
c3ae5cc Improve scheduler readability
738b298 Improve ICM42688 service readability
9f0e6f1 Improve ICM42688 driver readability
```

These commits should be treated as the current stable baseline.

## Verified Hardware Behavior

The expected ICM42688P serial output after successful hardware verification is:

```text
n=20
dt≈0.002472~0.002473
```

Do not assume a refactor is safe just because it builds. Hardware verification is required before committing behavior-changing code.

## Coding Style

Use Chinese comments for new explanatory comments.

For function header comments, use this style:

```cpp
/**
 * @brief  简要说明函数用途
 * @param  xxx 参数含义
 * @retval 返回值含义
 */
```

For internal logic comments, prefer block-level comments before a logical code section:

```cpp
// 1. 读取当前状态，判断是否需要进入更新流程。
...
// 2. 根据传感器数据更新时间戳和积分结果。
...
```

For variable declarations:

```cpp
MatrixXd P_;   // 协方差矩阵
MatrixXd q_;   // 系统噪声矩阵
MatrixXd dx_;  // 误差状态向量
```

If variables can be grouped logically, group them and place short inline Chinese comments on the same line.

Avoid vague comments. Comments should explain intent, timing, state-machine meaning, data ownership, or hardware constraints.

## Header and Formatting Style

1. 函数声明、函数调用、短 constexpr 表达式、短初始化语句，优先保持单行。
2. 不要机械套用 80/100 列换行规则。只有在一行明显过长、影响横向阅读，或语义上必须分行时才换行。
3. 对本项目而言，约 140～160 字符以内且可读性良好的声明/调用可以保持单行。
4. 形参列表不要频繁换行；除非参数很多、类型很长或一行已经明显不可读。
5. 头文件中的声明区域应按功能分块，并用空行分隔不同功能组。
6. 头文件中的 public/private 函数声明应优先作为"接口索引"保持紧凑，不要把简单声明写成多行。
7. 注释应解释职责、边界、状态含义、数据含义和关键不变量，不要重复代码表面含义。
8. 对字段很多的 struct / enum，应给关键字段或枚举项补充短行注释。
9. 行注释优先写在同一行；如果同一行太长，则把注释放在上一行。
10. 不要在头文件的全局作用域新增 using namespace。已有头文件如需减少长限定名，应优先使用类内 type alias 或局部限定名。
11. 在 .cpp 文件内部可以使用局部 using / using namespace，但不要让命名空间污染通过头文件传播给所有包含者。
12. 不要自动格式化整个文件；只格式化本轮明确修改的局部代码。
13. 如果用户已经手动整理了函数声明、形参列表或注释布局，不要未经请求重新换行或改成另一种风格。除非存在编译错误、明显格式错误或用户明确要求，否则保留用户的手动布局。

## Refactoring Rules

Deep refactoring is allowed only when the user explicitly requests it.

For deep refactoring:

```text
1. First produce a read-only refactor plan.
2. Do not modify code during the planning step.
3. Split refactoring into small steps.
4. One step should usually modify only one or two files.
5. Build after every step.
6. Do not commit until hardware verification passes and the user explicitly asks.
```

Prefer improving readability without changing runtime behavior.

Allowed readability improvements:

```text
1. Reduce unnecessary nesting.
2. Extract local helper functions if it makes data flow clearer.
3. Remove redundant wrappers if call depth is unnecessary.
4. Rename local variables if meaning becomes clearer.
5. Add structured Chinese comments.
6. Replace obsolete comments.
```

Forbidden unless explicitly requested:

```text
1. Changing sensor timing.
2. Changing scheduler policy.
3. Changing interrupt behavior.
4. Changing FIFO reading behavior.
5. Changing mathematical formulas.
6. Changing coordinate transform signs.
7. Changing public APIs used by other modules.
8. Moving driver recovery policy into scheduler or service.
```

## Interaction Rules

When asked to refactor, first state:

```text
1. Which files will be touched.
2. Which files will not be touched.
3. What behavior will remain unchanged.
4. What commands will be run for validation.
```

If the task scope is unsafe or too broad, stop and propose a smaller first step.

If unexpected diffs appear, stop and report before continuing.
