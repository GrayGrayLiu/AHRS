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
