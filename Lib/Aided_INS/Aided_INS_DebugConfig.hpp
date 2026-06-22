/**
 * @file    Aided_INS_DebugConfig.hpp
 * @brief   AHRS INS 开发阶段统一 compile-time 开关
 *
 * @details
 * 三个宏分别控制：
 *   - 启动阶段结构化矩阵数值验证
 *   - 运行期 1 Hz profiling printf 输出
 *   - 运行期 1 Hz diagnostic printf 输出
 *
 * 所有宏默认值为 1（开启）。收尾阶段可通过修改此文件或编译器 -D 选项统一关闭。
 */

#pragma once

#ifndef AIDED_INS_ENABLE_STARTUP_VERIFY
#define AIDED_INS_ENABLE_STARTUP_VERIFY 1
#endif

#ifndef AIDED_INS_ENABLE_PROFILING_PRINT
#define AIDED_INS_ENABLE_PROFILING_PRINT 1
#endif

#ifndef AIDED_INS_ENABLE_DEBUG_PRINT
#define AIDED_INS_ENABLE_DEBUG_PRINT 1
#endif
