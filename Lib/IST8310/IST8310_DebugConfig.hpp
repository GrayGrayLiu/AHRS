/**
 * @file    IST8310_DebugConfig.hpp
 * @brief   IST8310 Service 调试 compile-time 开关
 *
 * @details
 * IST8310_ENABLE_MAG_DEBUG_PRINT — 运行期 [mag] debug printf（1 Hz，默认关闭）。
 *   启用后仅用于硬件验证，printf 可能阻塞 cooperative scheduler
 *   数 ms 到十几 ms（浮点格式化 + UART TX）；
 *   应同时观察 IMU FIFO/INS 是否受影响，不应长期在飞行运行路径中开启。
 */

#pragma once

#ifndef IST8310_ENABLE_MAG_DEBUG_PRINT
#define IST8310_ENABLE_MAG_DEBUG_PRINT 0
#endif
