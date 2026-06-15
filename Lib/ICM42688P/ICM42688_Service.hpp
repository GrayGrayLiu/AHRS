#pragma once

#include <cstdint>

#include "ICM42688P.hpp"

namespace icm42688_service
{
struct DeltaSample
{
    uint64_t timestamp_us{0u};
    float delta_angle_rad[3]{};
    float delta_velocity_m_s[3]{};
    float delta_time_s{0.0F};
    uint16_t delta_samples{0u};
    uint32_t sample_counter{0u};
    float temperature_deg_c{0.0F};
};

void NotifyDataReadyFromISR(uint64_t timestamp_us);
void Run();

[[nodiscard]] ICM42688P::Status GetLatest(ICM42688P::Sample *sample);
[[nodiscard]] ICM42688P::Status GetDeltaLatest(DeltaSample *sample);
} // namespace icm42688_service
