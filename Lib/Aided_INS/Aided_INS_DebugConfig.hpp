/**
 * @file    Aided_INS_DebugConfig.hpp
 * @brief   AHRS INS 开发/调试阶段统一 compile-time 开关
 *
 * @details
 * 宏分别控制：
 *   AIDED_INS_ENABLE_STARTUP_VERIFY    — 启动阶段结构化矩阵数值验证
 *   AIDED_INS_ENABLE_SERVICE_PRINT     — 运行期 [INS_SVC] 基础统计 printf
 *   AIDED_INS_ENABLE_PROFILING_PRINT   — 运行期 [INS_PROF]~[INS_PROF4] printf
 *   AIDED_INS_ENABLE_DEBUG_PRINT       — 运行期 [ACC_DBG] / [EKF_DBG] / [ATT_DBG] printf
 *   AIDED_INS_ENABLE_COV_HEALTH_CHECK  — 运行期协方差健康检查（纯计数，无 printf）
 *   AIDED_INS_ENABLE_ATTITUDE_TELEMETRY — 运行期姿态遥测 printf（低优先级 task，默认关闭）
 *   AIDED_INS_ATTITUDE_TELEMETRY_MODE    — 遥测输出模式：0=四元数, 1=欧拉角deg（默认）
 *   AIDED_INS_ENABLE_MAG_UPDATE        — 是否启用 IST8310 磁力计 yaw 更新，默认关闭
 *
 * 运行期 UART printf 会阻塞主循环，可能导致 false timestamp_errors；
 * 正式运行默认关闭运行期输出，需要调试时再临时打开。启动验证默认保留。
 *
 * 【性能基线（STM32H723 cortex-m7 + FPU fpv5-d16 硬浮点 + RelWithDebInfo，
 *   runtime printf 及 profiling/debug/telemetry 输出默认关闭条件下的典型观测值，非实时上界）】
 *   优化前：Aided_INS::Run() 整体约 3.4~3.6 ms，
 *           其中状态转移/传播噪声相关矩阵构造约 2.4 ms，EKFPredict() 协方差预测约 0.94 ms。
 *   结构化 predict 后：InsPropagation() 整体约 0.60 ms，
 *           其中 INS 机械编排/机械更新约 66~67 us，
 *           状态转移矩阵 Φ 与传播噪声 Q 构造约 0.16 ms，
 *           EKFPredict() 协方差预测约 0.37 ms。
 *   producer/consumer 解耦后：
 *           ins_consumer 中 Aided_INS::Run() 约 0.93 ms，
 *           IMU DRDY producer/service 路径约 4~5 us。
 *   这些数据为历史测量参考，不作为跨编译配置、跨平台或最坏情况实时上界；
 *   用于解释为什么 INS 不应在 IMU DRDY 路径同步运行，以及为什么 runtime
 *   UART printf 默认关闭。
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
#define AIDED_INS_ENABLE_ATTITUDE_TELEMETRY 1
#endif

// 姿态遥测输出模式常量
#define AIDED_INS_ATTITUDE_TELEMETRY_MODE_QUAT       0
#define AIDED_INS_ATTITUDE_TELEMETRY_MODE_EULER_DEG  1

#ifndef AIDED_INS_ATTITUDE_TELEMETRY_MODE
#define AIDED_INS_ATTITUDE_TELEMETRY_MODE AIDED_INS_ATTITUDE_TELEMETRY_MODE_EULER_DEG
#endif

#ifndef AIDED_INS_ENABLE_MAG_UPDATE
#define AIDED_INS_ENABLE_MAG_UPDATE 1
#endif

// ── 运行时诊断频率控制 ──
#ifndef AIDED_INS_ENABLE_MAG_EKF_PRINT
#define AIDED_INS_ENABLE_MAG_EKF_PRINT 0
#endif
#ifndef AIDED_INS_ENABLE_PROP_GYRO_PRINT
#define AIDED_INS_ENABLE_PROP_GYRO_PRINT 0
#endif
#ifndef AIDED_INS_ENABLE_IMU_FLOW_PRINT
#define AIDED_INS_ENABLE_IMU_FLOW_PRINT 0
#endif
#ifndef AIDED_INS_ENABLE_FB_DBG_PRINT
#define AIDED_INS_ENABLE_FB_DBG_PRINT 0
#endif
#ifndef AIDED_INS_ENABLE_SCHED_STATS_PRINT
#define AIDED_INS_ENABLE_SCHED_STATS_PRINT 0
#endif
#ifndef AIDED_INS_ENABLE_ACC_GATE_PRINT
#define AIDED_INS_ENABLE_ACC_GATE_PRINT 0
#endif
#ifndef AIDED_INS_ENABLE_INIT_MAG_YAW_PRINT
#define AIDED_INS_ENABLE_INIT_MAG_YAW_PRINT 0
#endif
