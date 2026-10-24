#ifndef ATTITUDE_TRACE_H
#define ATTITUDE_TRACE_H

#include <stdint.h>

#define ATTITUDE_TRACE_VERSION 1u
#define ATTITUDE_TRACE_SIZE 68u
#define ATTITUDE_TRACE_CORRECTION_APPLIED 1u
#define ATTITUDE_TRACE_MAGNITUDE_REJECTED 2u
#define ATTITUDE_TRACE_INNOVATION_REJECTED 4u
#define ATTITUDE_TRACE_RECOVERY_WAITING 8u
#define ATTITUDE_TRACE_RECOVERY_APPLIED 16u
#define ATTITUDE_TRACE_STATE_RESET 32u
#define ATTITUDE_TRACE_ANGLE_WRAPPED 64u
#define ATTITUDE_TRACE_LEGACY_RESET 128u
#define ATTITUDE_TRACE_DT_ZERO_CLAMPED 1u
#define ATTITUDE_TRACE_DT_HIGH_CLAMPED 2u
#define ATTITUDE_TRACE_GYRO_CLIPPED 4u
#define ATTITUDE_TRACE_ACCEL_CLIPPED 8u

typedef struct
{
  float previous_deg;
  float predicted_deg;
  float accel_deg;
  float bias_dps; /* Residual Kalman bias used by this prediction. */
  float correction_deg; /* Final minus prediction, before angle wrapping. */
} AttitudeAxisTrace_t;

/* A single successful IMU read. Final angles and gyro rates are in the normal
 * flight record. These floats are diagnostic copies, not filter state. */
typedef struct
{
  uint32_t sequence;
  uint32_t timestamp_ms;
  float dt_ms;
  float accel_x_g;
  float accel_y_g;
  float accel_z_g;
  AttitudeAxisTrace_t roll;
  AttitudeAxisTrace_t pitch;
  uint8_t valid;
  uint8_t roll_flags;
  uint8_t pitch_flags;
  uint8_t timing_flags; /* Timestep clamping and raw sensor saturation. */
} AttitudeTrace_t;

_Static_assert(sizeof(AttitudeTrace_t) == ATTITUDE_TRACE_SIZE,
               "Attitude trace wire layout changed");

#endif
