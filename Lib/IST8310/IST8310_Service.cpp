/**
 * @file    IST8310_Service.cpp
 * @brief   IST8310 磁力计应用层 Service 实现
 *
 * @details
 * Service 层职责：
 *   - 拥有并管理全工程唯一的 IST8310 驱动实例；
 *   - 提供 Init() 作为完整硬件初始化入口（placement new + driver.Init()）；
 *   - 提供 Run() 作为非阻塞 scheduler 执行入口（每次最多一次 I2C 事务）；
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
 */

#include "IST8310_Service.hpp"

#include <cstdint>
#include <new>

#include "IST8310.hpp"
#include "IST8310_Calibration_Config.hpp"
#include "TimeBase.h"

namespace ist8310_service
{

// ============================================================================
// Service 参数
// ============================================================================

enum
{
    MIN_WAIT_US          =  6000u,   // 低噪声配置下单次测量最小等待，us（手册 Section 3.1.1）
    DATA_TIMEOUT_US      = 20000u,   // DRDY 等待超时，us
    SAMPLING_PERIOD_US   = 20000u,   // trigger-to-trigger 采样周期，us（目标 50 Hz；实测 sample_rate 以上板统计为准）
    MAX_CONSECUTIVE_ERRORS = 3u,     // 连续错误阈值，超此值进入 fault
};

// ============================================================================
// 状态机
// ============================================================================

enum class State : uint8_t
{
    IDLE,       // 等待下一个采样窗口
    TRIGGERED,  // 已写 CNTL1，等待 ≥6 ms
    READY,      // 6 ms 已过且 DRDY=1，等待下次 Run() 读取数据
};

namespace
{

// ============================================================================
// 唯一 IST8310 实例与 Service 状态
// ============================================================================

// IST8310 需要在构造时绑定 I2C 和地址，不能默认构造。
// 裸机环境不使用动态内存，在静态存储上通过 placement new 构造全工程唯一实例。
alignas(IST8310) uint8_t ist8310_storage[sizeof(IST8310)]{};
IST8310 *ist8310 = nullptr;

bool ist8310_bound   = false;   // C++ 对象已构造并绑定 I2C/地址（placement new 完成）
bool ist8310_started = false;   // driver.Init() 已成功，硬件已配置
bool ist8310_fault   = false;   // 连续 I2C 错误 ≥阈值，Run() 已停止数据采集

// ── 状态机上下文 ──
State   state_{State::IDLE};
uint64_t trigger_timestamp_us_{0u};   // 写 CNTL1 时的 MCU 微秒时间戳

// ── 最新有效 sample 缓存 ──
MagSample latest_sample_{};

// ── 错误计数 ──
uint8_t  consecutive_errors_{0u};       // 连续错误计数，用于 fault 判定
uint32_t total_error_counter_{0u};      // 累计 I2C/通信错误数
uint32_t total_overrun_counter_{0u};    // 累计 DOR 超限数

// ============================================================================
// 通用小工具函数
// ============================================================================

void EnterFault()
{
    ist8310_fault = true;
}

void ResetConsecutiveErrors() { consecutive_errors_ = 0u; }

void RecordError()
{
    ++total_error_counter_;
    ++consecutive_errors_;
    if (consecutive_errors_ >= MAX_CONSECUTIVE_ERRORS) {
        EnterFault();
    }
}

// ============================================================================
// 坐标映射
// ============================================================================

// IST8310 sensor-frame → board/body-frame 安装映射（硬件确认）：
//   X_b =  Y_s    （sensor Y → board X，前向）
//   Y_b = -X_s    （-sensor X → board Y，右向）
//   Z_b = -Z_s    （-sensor Z → board Z，下向）
void TranslateSensorToBody(const int16_t sensor[3], int16_t body[3])
{
    body[0] =  sensor[1];  // X_b =  Y_s
    body[1] = -sensor[0];  // Y_b = -X_s
    body[2] = -sensor[2];  // Z_b = -Z_s
}

// ============================================================================
// 校准应用
// ============================================================================

// 将未校准 body-frame uT 应用 hard-iron bias + scale 得到校准后 body-frame uT。
void ApplyMagCalibration(const float in_body_uT[3], float out_body_uT[3])
{
#if IST8310_ENABLE_MAG_CALIBRATION
    for (int i = 0; i < 3; ++i) {
        out_body_uT[i] =
            (in_body_uT[i] - ist8310_calibration_config::kMagHardIronBiasBody_uT[i])
            * ist8310_calibration_config::kMagScaleBody[i];
    }
#else
    for (int i = 0; i < 3; ++i) {
        out_body_uT[i] = in_body_uT[i];
    }
#endif
}

// ============================================================================
// 非阻塞状态机辅助
// ============================================================================

// IDLE: 以 trigger-to-trigger 周期写 CNTL1 触发单次测量。
//       周期基准为上次 CNTL1 写入时刻 trigger_timestamp_us_（对齐 PX4 IST8310 的 MEASURE 状态语义）。
//       写 CNTL1 失败也视为一次 trigger attempt，用于限制 I2C 故障时的重试频率。
void HandleIdle()
{
    const uint64_t now_us = TimeBase_Micros();
    const uint64_t elapsed = now_us - trigger_timestamp_us_;

    if (elapsed < SAMPLING_PERIOD_US) {
        return;
    }

    // 写 CNTL1[3:0] = 0x01（Single Measurement），手册 Section 3.1.2。
    const IST8310::Status status = ist8310->RegisterWrite(
        IST8310_Regs::Register::CNTL1,
        static_cast<uint8_t>(IST8310_Regs::CNTL1_BITS::MODE_SINGLE_MEASUREMENT));

    // I2C 写失败也记录 trigger_timestamp_us_，避免故障时高频重试。
    trigger_timestamp_us_ = now_us;

    if (status != IST8310::Status::Ok) {
        RecordError();
        return;
    }

    state_ = State::TRIGGERED;
}

// TRIGGERED: 等待 6 ms → 读 STAT1.DRDY。
void HandleTriggered()
{
    const uint64_t now_us = TimeBase_Micros();
    const uint64_t elapsed = now_us - trigger_timestamp_us_;

    // 低噪声最小 6 ms 未到，直接返回。
    if (elapsed < MIN_WAIT_US) {
        return;
    }

    // 读 STAT1，检查 DRDY。
    uint8_t stat1{};
    const IST8310::Status status = ist8310->RegisterRead(IST8310_Regs::Register::STAT1, stat1);

    if (status != IST8310::Status::Ok) {
        RecordError();
        state_ = State::IDLE;
        return;
    }

    const bool drdy = (stat1 & static_cast<uint8_t>(IST8310_Regs::STAT1_BITS::DRDY_READY)) != 0u;

    if (!drdy) {
        // 超时判定：已等待超过 DATA_TIMEOUT_US 仍未就绪。
        if (elapsed > DATA_TIMEOUT_US) {
            RecordError();
            state_ = State::IDLE;
        }
        // 未超时则继续等待，下次 Run() 再查。
        return;
    }

    // DRDY=1，数据已就绪。下次 Run() 进入 READY 读取。
    state_ = State::READY;
}

// READY: 调用 ReadMeasurement() burst 读取并生成 sample。
void HandleReady()
{
    IST8310::RawMagData raw_data{};
    const IST8310::Status status = ist8310->ReadMeasurement(raw_data);
    const uint64_t read_us = TimeBase_Micros();

    // DOR 超限：sample 无效，丢弃；不计为连续错误（手册 Section 3.4：后续数据仍有效）。
    if (status == IST8310::Status::DataOverrun) {
        ++total_overrun_counter_;
        state_ = State::IDLE;
        return;
    }

    if (status != IST8310::Status::Ok) {
        RecordError();
        state_ = State::IDLE;
        return;
    }

    // ── sensor-frame 原始数据 ──
    latest_sample_.raw_sensor[0] = raw_data.x;
    latest_sample_.raw_sensor[1] = raw_data.y;
    latest_sample_.raw_sensor[2] = raw_data.z;

    constexpr float scale = IST8310_Regs::DATASHEET_DEFAULT_UT_PER_LSB;
    latest_sample_.mag_uT_sensor[0] = static_cast<float>(raw_data.x) * scale;
    latest_sample_.mag_uT_sensor[1] = static_cast<float>(raw_data.y) * scale;
    latest_sample_.mag_uT_sensor[2] = static_cast<float>(raw_data.z) * scale;

    // ── board/body-frame 数据 ──
    int16_t body_raw[3]{};
    TranslateSensorToBody(latest_sample_.raw_sensor, body_raw);
    latest_sample_.raw_body[0] = body_raw[0];
    latest_sample_.raw_body[1] = body_raw[1];
    latest_sample_.raw_body[2] = body_raw[2];
    latest_sample_.mag_uT_body[0] = static_cast<float>(body_raw[0]) * scale;
    latest_sample_.mag_uT_body[1] = static_cast<float>(body_raw[1]) * scale;
    latest_sample_.mag_uT_body[2] = static_cast<float>(body_raw[2]) * scale;

    // ── 校准后 body-frame 数据 ──
    ApplyMagCalibration(latest_sample_.mag_uT_body,
                        latest_sample_.mag_uT_body_calibrated);
#if IST8310_ENABLE_MAG_CALIBRATION
    latest_sample_.calibration_applied = true;
#else
    latest_sample_.calibration_applied = false;
#endif

    // ── 通用字段 ──
    latest_sample_.trigger_timestamp_us = trigger_timestamp_us_;
    latest_sample_.read_timestamp_us    = read_us;
    latest_sample_.status               = 0u;
    latest_sample_.valid                = true;
    latest_sample_.error_counter        = total_error_counter_;
    latest_sample_.overrun_counter      = total_overrun_counter_;
    ++latest_sample_.sample_counter;

    ResetConsecutiveErrors();
    state_ = State::IDLE;
}

} // namespace

// ============================================================================
// 对外 Service 接口
// ============================================================================

int Init(I2C_HandleTypeDef *const hi2c, const uint8_t address_7bit,
         GPIO_TypeDef *const reset_port, const uint16_t reset_pin)
{
    if (hi2c == nullptr
        || address_7bit < IST8310_Regs::I2C_ADDRESS_MIN_7BIT
        || address_7bit > IST8310_Regs::I2C_ADDRESS_MAX_7BIT
        || (reset_port == nullptr && reset_pin != 0u)
        || (reset_port != nullptr && reset_pin == 0u))
    {
        return -1;
    }

    if (ist8310_bound) {
        // 已绑定：重新调用 driver.Init() 完成硬件恢复。
        const IST8310::Status status = ist8310->Init();

        if (status != IST8310::Status::Ok) {
            ist8310_started = false;
            ist8310_fault   = true;
            return -1;
        }

        // driver.Init() 成功 → 清空旧 sample，重置所有状态。
        ist8310_fault   = false;
        ist8310_started = true;
        latest_sample_  = {};
        trigger_timestamp_us_ = 0u;
        state_          = State::IDLE;
        ResetConsecutiveErrors();
        return 0;
    }

    // 首次绑定：placement new 构造驱动实例。
    ist8310 = ::new (static_cast<void *>(ist8310_storage))
        IST8310(hi2c, address_7bit, reset_port, reset_pin);
    ist8310_bound = true;

    const IST8310::Status status = ist8310->Init();

    if (status != IST8310::Status::Ok) {
        ist8310_started = false;
        return -1;
    }

    // driver.Init() 成功 → 初始状态。
    ist8310_started       = true;
    ist8310_fault         = false;
    latest_sample_        = {};
    trigger_timestamp_us_ = 0u;
    state_                = State::IDLE;
    ResetConsecutiveErrors();

    return 0;
}

void Run()
{
    // fault 或未 started 时直接返回。
    if (ist8310_fault || !ist8310_started || ist8310 == nullptr) {
        return;
    }

    // 每次 Run() 只执行一个 I2C 阶段，不 fall through。
    switch (state_) {
    case State::IDLE:
        HandleIdle();
        break;
    case State::TRIGGERED:
        HandleTriggered();
        break;
    case State::READY:
        HandleReady();
        break;
    default:
        state_ = State::IDLE;
        break;
    }
}

bool CopyLatest(MagSample *const out)
{
    if (out == nullptr) {
        return false;
    }

    if (!ist8310_started || ist8310_fault || ist8310 == nullptr) {
        return false;
    }

    if (!latest_sample_.valid) {
        return false;
    }

    *out = latest_sample_;
    return true;
}

bool IsStarted()
{
    return ist8310_started && !ist8310_fault;
}

bool IsFault()
{
    return ist8310_fault;
}

} // namespace ist8310_service
