#include "CoreHelpers.h"

#include <math.h>
#include <stddef.h>

#define TAKEOFF_ALTITUDE_ACQUIRE_TIMEOUT_AFTER_RAMP_MS 2000U
#define TAKEOFF_CLIMB_TIMEOUT_AFTER_RAMP_MS 10000U

uint32_t CoreHelpers_GetTakeoffRampTimeMs(
    const DroneConfig_t *config)
{
  if (config == NULL)
  {
    return 0U;
  }

  const double ramp_range =
      fmax(0.0, config->maxThrottle - config->armThrottle);
  const double ramp_time_ms =
      1000.0 * ramp_range / config->takeoffThrottleRampPerSecond;
  return (uint32_t)ramp_time_ms;
}

uint32_t CoreHelpers_GetTakeoffAltitudeAcquireTimeoutMs(
    const DroneConfig_t *config)
{
  return CoreHelpers_GetTakeoffRampTimeMs(config) +
         TAKEOFF_ALTITUDE_ACQUIRE_TIMEOUT_AFTER_RAMP_MS;
}

uint32_t CoreHelpers_GetTakeoffTargetTimeoutMs(
    const DroneConfig_t *config)
{
  return CoreHelpers_GetTakeoffRampTimeMs(config) +
         TAKEOFF_CLIMB_TIMEOUT_AFTER_RAMP_MS;
}

uint8_t CoreHelpers_IsConfigValid(const DroneConfig_t *config)
{
  if (config == NULL || !isfinite(config->minThrottle) ||
      !isfinite(config->maxThrottle) ||
      !isfinite(config->armThrottle) ||
      !isfinite(config->hoverThrottle) ||
      !isfinite(config->maxAngle) ||
      !isfinite(config->takeoffThrottleRampPerSecond) ||
      !isfinite(config->takeoffAltitudeCm) ||
      config->minThrottle < MOTOR_ABSOLUTE_MIN_THROTTLE ||
      config->maxThrottle > MOTOR_ABSOLUTE_MAX_THROTTLE ||
      config->minThrottle > config->maxThrottle ||
      config->armThrottle < config->minThrottle ||
      config->armThrottle > config->maxThrottle ||
      config->hoverThrottle < config->armThrottle ||
      config->hoverThrottle > config->maxThrottle ||
      config->maxAngle <= 0.0 || config->maxAngle > 90.0 ||
      config->takeoffThrottleRampPerSecond <
          TAKEOFF_THROTTLE_RAMP_MIN_PER_SECOND ||
      config->takeoffThrottleRampPerSecond >
          TAKEOFF_THROTTLE_RAMP_MAX_PER_SECOND ||
      config->takeoffAltitudeCm < TAKEOFF_ALTITUDE_MIN_CM ||
      config->takeoffAltitudeCm > TAKEOFF_ALTITUDE_MAX_CM ||
      config->pidMaxIPart < 0.0 || config->pidMaxIPart > 100.0 ||
      config->pidMaxOutput < 0.0 ||
      config->pidMaxOutput > 100.0)
  {
    return 0U;
  }
  return 1U;
}
