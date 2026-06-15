#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void ICM42688_ServiceRun(void);
void ICM42688_ServiceNotifyDataReadyFromISR(uint64_t timestamp_us);

#ifdef __cplusplus
}
#endif
