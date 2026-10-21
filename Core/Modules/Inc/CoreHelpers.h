#ifndef CORE_HELPERS_H
#define CORE_HELPERS_H

#include "Core.h"

double CoreHelpers_GetTakeoffThrottleLimit(
    const DroneConfig_t *config);
uint32_t CoreHelpers_GetTakeoffLiftoffTimeoutMs(
    const DroneConfig_t *config);
uint32_t CoreHelpers_GetTakeoffTargetTimeoutMs(
    const DroneConfig_t *config);
uint8_t CoreHelpers_IsConfigValid(const DroneConfig_t *config);

#endif /* CORE_HELPERS_H */
