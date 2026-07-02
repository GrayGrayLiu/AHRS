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
 *   - 应用已配置的 hard-iron bias + symmetric 3×3 scale matrix 校准参数；
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
// 运行期配置寄存器检查（对齐 PX4 IST8310 的 runtime register check）
// 必须与 IST8310.hpp 中 register_cfg_[] 的 PDCNTL/AVGCNTL 配置保持一致。
// ============================================================================

struct RuntimeRegCheckEntry
{
    IST8310_Regs::Register reg;
    uint8_t mask;
    uint8_t expected;
};

// 仅检查 AVGCNTL(0x41) 和 PDCNTL(0x42)；
// 不检查 CNTL2（DREN/DRP 不影响 I2C 路径），不检查 CNTL3（AHRS 不使用 16-bit 输出）。
static constexpr RuntimeRegCheckEntry kRuntimeRegChecks[] = {
    {
        IST8310_Regs::Register::PDCNTL,
        static_cast<uint8_t>(IST8310_Regs::PDCNTL_BITS::PULSE_DURATION_MASK),
        static_cast<uint8_t>(IST8310_Regs::PDCNTL_BITS::PULSE_DURATION_NORMAL),
    },
    {
        IST8310_Regs::Register::AVGCNTL,
        static_cast<uint8_t>(IST8310_Regs::AVGCNTL_BITS::Y_AVG_MASK
                           | IST8310_Regs::AVGCNTL_BITS::XZ_AVG_MASK),
        static_cast<uint8_t>(IST8310_Regs::AVGCNTL_BITS::Y_AVG_16_TIMES
                           | IST8310_Regs::AVGCNTL_BITS::XZ_AVG_16_TIMES),
    },
};

static constexpr uint32_t RUNTIME_CHECK_INTERVAL = 100u;  // 每 100 次 trigger 检查 1 个寄存器
static constexpr size_t   RUNTIME_CHECK_COUNT =
    sizeof(kRuntimeRegChecks) / sizeof(kRuntimeRegChecks[0]);

uint32_t runtime_check_trigger_count_{0u};     // 采样窗口计数；每达到一次 trigger 条件（elapsed >= SAMPLING_PERIOD_US）递增
uint8_t  runtime_check_index_{0u};             // 下一轮要检查的寄存器下标
uint32_t total_register_mismatch_counter_{0u}; // 累计寄存器值不匹配次数（仅递增，不清零）

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

// 将未校准 body-frame uT 应用 hard-iron bias + symmetric 3×3 scale matrix 得到校准后 body-frame uT。
//
// 校准模型（对齐 PX4 calibration::Magnetometer::Correct 的 body-frame 版本）：
//   centered = mag_body_raw - bias_body
//   mag_body_cal = M_body * centered
//
// 其中 M_body 为 3×3 对称矩阵：
//   [ scale_x     offdiag_xy  offdiag_xz ]
//   [ offdiag_xy  scale_y     offdiag_yz ]
//   [ offdiag_xz  offdiag_yz  scale_z     ]
//
// 当前默认 offdiag = 0，数学上等价于原 per-axis diagonal scale。
// 无 PX4 的 rotation（AHRS 已在 sensor→body 映射中完成），无 power compensation。
void ApplyMagCalibration(const float in_body_uT[3], float out_body_uT[3])
{
#if IST8310_ENABLE_MAG_CALIBRATION
    const float centered[3] = {
        in_body_uT[0] - ist8310_calibration_config::kMagHardIronBiasBody_uT[0],
        in_body_uT[1] - ist8310_calibration_config::kMagHardIronBiasBody_uT[1],
        in_body_uT[2] - ist8310_calibration_config::kMagHardIronBiasBody_uT[2],
    };

    const float sx = ist8310_calibration_config::kMagScaleBody[0];
    const float sy = ist8310_calibration_config::kMagScaleBody[1];
    const float sz = ist8310_calibration_config::kMagScaleBody[2];
    const float oxy = ist8310_calibration_config::kMagOffDiagScaleBody[0];
    const float oxz = ist8310_calibration_config::kMagOffDiagScaleBody[1];
    const float oyz = ist8310_calibration_config::kMagOffDiagScaleBody[2];

    out_body_uT[0] = sx  * centered[0] + oxy * centered[1] + oxz * centered[2];
    out_body_uT[1] = oxy * centered[0] + sy  * centered[1] + oyz * centered[2];
    out_body_uT[2] = oxz * centered[0] + oyz * centered[1] + sz  * centered[2];
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

    // ── 运行期配置寄存器检查，每 100 次 trigger 轮转 1 个寄存器 ──
    // 检查作为独立 I2C 阶段处理：触发时只执行一次 RegisterRead 就返回，
    // 下一次 Run() 再写 CNTL1。保证每次 Run() 最多一个 I2C 事务。
    // 代价是每约 2 秒有一轮 trigger 延后约 1 ms，对 50 Hz 采样无实质影响。
    ++runtime_check_trigger_count_;
    if (runtime_check_trigger_count_ >= RUNTIME_CHECK_INTERVAL) {
        runtime_check_trigger_count_ = 0u;

        const auto &entry = kRuntimeRegChecks[runtime_check_index_];
        uint8_t value{};
        const IST8310::Status rd = ist8310->RegisterRead(entry.reg, value);

        if (rd != IST8310::Status::Ok) {
            RecordError();
            return;   // I2C 读失败，本轮不写 CNTL1
        }

        if ((value & entry.mask) != entry.expected) {
            ++total_register_mismatch_counter_;
            EnterFault();
            return;   // 寄存器不匹配，进入 fault，本轮不写 CNTL1
        }

        runtime_check_index_ = (runtime_check_index_ + 1u) % RUNTIME_CHECK_COUNT;
        return;   // 检查通过；本轮不写 CNTL1，下一次 Run() 再触发测量
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
        runtime_check_trigger_count_ = 0u;
        runtime_check_index_         = 0u;
        // total_register_mismatch_counter_ 不清零；与 total_error_counter_ 一样为累计 lifetime counter
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
    runtime_check_trigger_count_ = 0u;
    runtime_check_index_         = 0u;
    // total_register_mismatch_counter_ 不清零；与 total_error_counter_ 一样为累计 lifetime counter
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
