/**
 * @file    Aided_INS_DebugConfig.hpp
 * @brief   AHRS INS 开发/调试阶段统一 compile-time 开关
 *
 * @details
 * 四个宏分别控制：
 *   AIDED_INS_ENABLE_STARTUP_VERIFY    — 启动阶段结构化矩阵数值验证
 *   AIDED_INS_ENABLE_SERVICE_PRINT     — 运行期 [INS_SVC] 基础统计 printf
 *   AIDED_INS_ENABLE_PROFILING_PRINT   — 运行期 [INS_PROF]~[INS_PROF4] printf
 *   AIDED_INS_ENABLE_DEBUG_PRINT       — 运行期 [ACC_DBG] / [EKF_DBG] / [ATT_DBG] printf
 *   AIDED_INS_ENABLE_COV_HEALTH_CHECK  — 运行期协方差健康检查（纯计数，无 printf）
 *   AIDED_INS_ENABLE_ATTITUDE_TELEMETRY — 运行期姿态遥测 printf（低优先级 task，默认关闭）
 *   AIDED_INS_ATTITUDE_TELEMETRY_MODE    — 遥测输出模式：0=四元数, 1=欧拉角deg（默认）
 *
 * 运行期 UART printf 会阻塞主循环，可能导致 false timestamp_errors；
 * 正式运行默认关闭运行期输出，需要调试时再临时打开。启动验证默认保留。
 *
 * 可通过修改此文件或编译器 -D 选项覆盖。
 */

#pragma once

#ifndef AIDED_INS_ENABLE_STARTUP_VERIFY
#define AIDED_INS_ENABLE_STARTUP_VERIFY 1
#endif

#ifndef AIDED_INS_ENABLE_SERVICE_PRINT
#define AIDED_INS_ENABLE_SERVICE_PRINT 0
#endif

#ifndef AIDED_INS_ENABLE_PROFILING_PRINT
#define AIDED_INS_ENABLE_PROFILING_PRINT 0
#endif

#ifndef AIDED_INS_ENABLE_DEBUG_PRINT
#define AIDED_INS_ENABLE_DEBUG_PRINT 0
#endif

#ifndef AIDED_INS_ENABLE_COV_HEALTH_CHECK
#define AIDED_INS_ENABLE_COV_HEALTH_CHECK 1
#endif

#ifndef AIDED_INS_ENABLE_ATTITUDE_TELEMETRY
#define AIDED_INS_ENABLE_ATTITUDE_TELEMETRY 0
#endif

// 姿态遥测输出模式常量
#define AIDED_INS_ATTITUDE_TELEMETRY_MODE_QUAT       0
#define AIDED_INS_ATTITUDE_TELEMETRY_MODE_EULER_DEG  1

#ifndef AIDED_INS_ATTITUDE_TELEMETRY_MODE
#define AIDED_INS_ATTITUDE_TELEMETRY_MODE AIDED_INS_ATTITUDE_TELEMETRY_MODE_EULER_DEG
#endif
