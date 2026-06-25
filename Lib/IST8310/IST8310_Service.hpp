/**
 * @file    IST8310_Service.hpp
 * @brief   IST8310 磁力计应用层 Service 接口
 *
 * @details
 * Service 层职责：
 *   - 拥有并管理全工程唯一的 IST8310 驱动实例；
 *   - 提供 Init() 作为完整硬件初始化入口（含 driver.Init() 的 POR/Configure）；
 *   - 提供 Run() 作为非阻塞 scheduler 执行入口；
 *   - sensor-frame → board/body-frame 坐标映射；
 *   - 应用已配置的 hard-iron bias + per-axis scale 校准参数；
 *   - 提供 CopyLatest() 非破坏性缓存读取接口（返回 sensor-frame、
 *     未校准 body-frame、校准后 body-frame 三个坐标系的 sample）。
 *
 * 不负责：
 *   - 底层 HAL I2C 事务实现、数据字节拼接（由 IST8310 driver 负责）；
 *   - 校准参数估计（由 ist8310_calibration 模块负责）；
 *   - Flash / 文件系统保存校准参数；
 *   - Aided_INS 数学更新；
 *   - scheduler 调度策略。
 *
 * ISR 边界：
 *   当前版本无 ISR 桥接。IST8310 DRDY 引脚未接入 EXTI。
 */

#pragma once

#include <cstdint>

#include "IST8310.hpp"

namespace ist8310_service
{

/**
 * @brief  磁力计 Service 层采样缓存结构
 *
 * @note   trigger_timestamp_us 近似采样时刻（写 CNTL1 时记录），
 *         read_timestamp_us 为实际读出时刻（burst read 前记录）。
 *         调用方自行比较 sample_counter 去重。
 *
 *         raw_sensor / mag_uT_sensor:
 *           IST8310 芯片原生 sensor-frame 数据 (X_s/Y_s/Z_s)，
 *           仅用于调试和硬件追溯。
 *
 *         raw_body / mag_uT_body:
 *           board/body-frame 未校准磁场 (X_b=前/Y_b=右/Z_b=下)，
 *           由 service 内部坐标映射生成。
 *
 *         mag_uT_body_calibrated / calibration_applied:
 *           body-frame 校准后磁场，应用编译期 hard-iron bias + per-axis scale。
 *           后续 Aided_INS 接入磁力计时应优先使用此字段。
 */
struct MagSample
{
    uint64_t trigger_timestamp_us{0};   // 写 CNTL1 后的 MCU 微秒时刻
    uint64_t read_timestamp_us{0};      // burst read 前的 MCU 微秒时刻

    // IST8310 sensor-frame 原始数据（芯片原生 X_s/Y_s/Z_s，仅调试）
    int16_t raw_sensor[3]{0, 0, 0};
    float   mag_uT_sensor[3]{0.0F, 0.0F, 0.0F};

    // Board/body-frame 未校准磁场（X_b=前, Y_b=右, Z_b=下）
    int16_t raw_body[3]{0, 0, 0};
    float   mag_uT_body[3]{0.0F, 0.0F, 0.0F};

    // Board/body-frame 校准后磁场（应用 hard-iron bias + scale）
    float   mag_uT_body_calibrated[3]{0.0F, 0.0F, 0.0F};
    bool    calibration_applied{false};

    uint8_t  status{0};                 // 当前仅发布成功 sample，恒为 0；保留给后续扩展
    bool     valid{false};              // 本次 sample 数据有效
    uint32_t sample_counter{0};         // 累计成功 sample 数（调用方去重用）
    uint32_t error_counter{0};          // 累计 I2C/通信错误次数（采样时刻快照）
    uint32_t overrun_counter{0};        // 累计 DOR 超限次数（采样时刻快照）
};

/**
 * @brief  完整硬件初始化：绑定 I2C → placement new 构造 driver → driver.Init()
 * @param  hi2c          HAL I2C 句柄指针
 * @param  address_7bit  IST8310 7-bit I2C 地址
 * @param  reset_port    硬件复位 GPIO 端口（可选，无则传 nullptr）
 * @param  reset_pin     硬件复位 GPIO 引脚（可选，无则传 0）
 * @retval 0  成功
 * @retval -1 失败（I2C 句柄无效 / 地址非法 / GPIO 参数不一致 / driver.Init() 失败）
 *
 * @note   阻塞式调用。driver.Init() 包含 POR delay ≥50ms + Configure。
 *         只应在 App_Init 等初始化阶段调用，禁止在 scheduler task callback 中调用。
 */
int Init(I2C_HandleTypeDef *hi2c, uint8_t address_7bit,
         GPIO_TypeDef *reset_port, uint16_t reset_pin);

/**
 * @brief  非阻塞状态机推进
 *
 * @note   每次调用最多执行一次 I2C 事务或直接返回。
 *         禁止在 ISR 中调用。
 *         fault 状态下直接返回，不做任何 I2C 操作。
 *         不调用 HAL_Delay，不在 Run() 中调用 driver.Init()。
 *
 *         适合 periodic scheduler task（建议 1 ms 周期）。
 *         内部 sample_period_us 决定实际采样频率（初期 20000 μs = 50 Hz）。
 */
void Run();

/**
 * @brief  非破坏性拷贝最新有效 mag sample
 * @param  out 输出缓冲区（不可为 nullptr）
 * @retval true  拷贝成功
 * @retval false 无有效数据（service 未 started / fault / 尚无 sample / sample 不 valid /
 *               out == nullptr）
 *
 * @note   不跟踪 consumer，不保存消费计数。调用方自行比较 sample_counter 判断新数据。
 *         多次调用返回同一个 sample（在 sample_counter 不变时）。
 */
[[nodiscard]] bool CopyLatest(MagSample *out);

/**
 * @brief  Service 是否已完成初始化并正常运行（Init() 已成功，且非 fault 状态）
 */
[[nodiscard]] bool IsStarted();

/**
 * @brief  是否进入 fault 状态（连续 I2C 错误 ≥3，Run() 已停止数据采集）
 * @note   fault 后需外部在安全上下文调用 Init() 恢复。
 */
[[nodiscard]] bool IsFault();

} // namespace ist8310_service
