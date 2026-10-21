#include "FlightControl.h"
#include "AppError.h"
#include "Core.h"
#include "CoreHelpers.h"
#include "Logger.h"
#include "MotorControl.h"
#include "SensorAcquisition.h"
#include "SensorConfig.h"
#include "Tools.h"
#include <math.h>

static double current_throttle_command;
static uint32_t takeoff_started_ms;
static uint8_t takeoff_baseline_valid;
static double takeoff_baseline_cm;
static double liftoff_confirm_s;
static uint8_t altitude_control_active;
static double altitude_target_cm;
static double takeoff_settled_s;

static void FlightControl_ResetTakeoff(void)
{
  takeoff_started_ms = 0U;
  takeoff_baseline_valid = 0U;
  takeoff_baseline_cm = 0.0;
  liftoff_confirm_s = 0.0;
  altitude_control_active = 0U;
  altitude_target_cm = 0.0;
  takeoff_settled_s = 0.0;
}

static double FlightControl_SlewValue(double current, double target,
                                      double max_step)
{
  if (current < target)
  {
    return fmin(current + max_step, target);
  }
  if (current > target)
  {
    return fmax(current - max_step, target);
  }
  return current;
}

void FlightControl_BeginTakeoff(void)
{
  FlightControl_ResetTakeoff();
  PID_Reset(&pid_roll);
  PID_Reset(&pid_pitch);
  PID_Reset(&pid_Gz);
  PID_Reset(&pid_altitude);
  /* Also handles ARM and TAKEOFF received before the first armed update. */
  current_throttle_command = Config.armThrottle;
  takeoff_started_ms = HAL_GetTick();
  flightData = SensorAcquisition_GetFlightData();
  if (flightData.takeoff_altitude_valid &&
      isfinite(flightData.takeoff_altitude))
  {
    takeoff_baseline_cm = flightData.takeoff_altitude;
    takeoff_baseline_valid = 1U;
  }
}

uint8_t FlightControl_IsTakeoffComplete(void)
{
  return altitude_control_active &&
         takeoff_settled_s >= TAKEOFF_SETTLE_TIME_S;
}

HAL_StatusTypeDef FlightControl_Init(void)
{
  LogInformation(1002, "Initializing Flight Control...");

  AppError_Clear();
  flightData = SensorAcquisition_GetFlightData();

  if (!SensorAcquisition_IsReady())
  {
    return AppError_Set(APP_ERROR_SENSOR_ACQUISITION, HAL_BUSY);
  }

  // Reset all motor speeds
  Motors_Reset(&Motors_Speed);
  if (Motors_SetSpeed(Motors_Speed) != HAL_OK)
  {
    return AppError_Set(APP_ERROR_ARM_MOTOR_OUTPUT, HAL_ERROR);
  }

  // Reset PID controllers
  PID_Reset(&pid_roll);
  PID_Reset(&pid_pitch);
  PID_Reset(&pid_Gz);
  PID_Reset(&pid_altitude);
  current_throttle_command = 0.0;
  FlightControl_ResetTakeoff();

  // Set initial target yaw from current yaw reading when available.
  if (SENSOR_QMC5883_ENABLED)
  {
    Config.yaw.target = flightData.yaw_deg;
  }
  else
  {
    Config.yaw.target = 0;
  }

  LogInformation(1001, "Flight Control initialized successfully");
  return HAL_OK;
}

HAL_StatusTypeDef FlightControl_Update(double dt)
{
  AppError_Clear();

  if (!isfinite(dt) || dt <= 0.0 || dt > 0.25)
  {
    return AppError_Set(APP_ERROR_INVALID_FLIGHT_TIMESTEP, HAL_ERROR);
  }

  // Sensor acquisition runs continuously in Core_loop(). Consume its latest
  // cached snapshot without touching the sensor buses from flight control.
  flightData = SensorAcquisition_GetFlightData();

  PID_ClearTerms(&pid_roll);
  PID_ClearTerms(&pid_pitch);
  PID_ClearTerms(&pid_Gz);
  PID_ClearTerms(&pid_altitude);

  double m1_pid = 0;
  double m2_pid = 0;
  double m3_pid = 0;
  double m4_pid = 0;

  double roll_modifier = 0;
  double pitch_modifier = 0;
  double Gz_modifier = 0;
  double altitude_modifier = 0;
  uint8_t mixer_limited = 0u;

  const uint8_t motors_running =
      droneState.state == DRONE_STATE_ARMED ||
      droneState.state == DRONE_STATE_TAKEOFF ||
      droneState.state == DRONE_STATE_FLYING;

  const uint8_t stabilization_active =
      SENSOR_MPU6050_ENABLED &&
      (droneState.state == DRONE_STATE_TAKEOFF ||
       droneState.state == DRONE_STATE_FLYING);

  if (stabilization_active &&
      (fabs(flightData.roll_deg) > Config.maxAngle ||
       fabs(flightData.pitch_deg) > Config.maxAngle))
  {
    FlightControl_Stop();
    return AppError_Set(APP_ERROR_MAXIMUM_FLIGHT_ANGLE, HAL_ERROR);
  }

  /* Sensor acquisition already tolerates brief echo gaps (300 ms). Once a
   * baseline is known, never resume a blind ramp after losing the sensor. */
  if ((droneState.state == DRONE_STATE_FLYING ||
       (droneState.state == DRONE_STATE_TAKEOFF && takeoff_baseline_valid)) &&
      (!flightData.takeoff_altitude_valid ||
       !isfinite(flightData.takeoff_altitude)))
  {
    FlightControl_Stop();
    return AppError_Set(APP_ERROR_ALTITUDE_UNAVAILABLE, HAL_ERROR);
  }

  if (droneState.state == DRONE_STATE_TAKEOFF)
  {
    const uint32_t elapsed_ms = HAL_GetTick() - takeoff_started_ms;
    if (!altitude_control_active &&
        elapsed_ms >= CoreHelpers_GetTakeoffLiftoffTimeoutMs(&Config))
    {
      FlightControl_Stop();
      return AppError_Set(APP_ERROR_TAKEOFF_LIFTOFF_TIMEOUT, HAL_ERROR);
    }
    if (elapsed_ms >= CoreHelpers_GetTakeoffTargetTimeoutMs(&Config))
    {
      FlightControl_Stop();
      return AppError_Set(APP_ERROR_TAKEOFF_TARGET_TIMEOUT, HAL_ERROR);
    }
  }

  if (motors_running)
  {

    if (stabilization_active)
    {
      const double roll_error = Config.roll.target - flightData.roll_deg;
      const double pitch_error = Config.pitch.target - flightData.pitch_deg;
      const double gz_error = Config.Gz.target - flightData.Gz;
      /*
       * P and D keep stabilizing during TAKEOFF, but I remains zero until the
       * configured takeoff phase is complete. This prevents ground-constrained
       * attitude error from being stored and released at liftoff.
       */
      const uint8_t attitude_integral_enabled =
          droneState.state == DRONE_STATE_FLYING;

      roll_modifier = PID_Calculate(&pid_roll, roll_error, dt,
                                    attitude_integral_enabled);
      pitch_modifier = PID_Calculate(&pid_pitch, pitch_error, dt,
                                     attitude_integral_enabled);
      Gz_modifier = PID_Calculate(&pid_Gz, gz_error, dt,
                                  attitude_integral_enabled);
    }

    if (droneState.state == DRONE_STATE_ARMED)
    {
      // Arming starts every motor equally at the configured safe idle.
      current_throttle_command = Config.armThrottle;
    }
    else if (droneState.state == DRONE_STATE_TAKEOFF &&
             !altitude_control_active)
    {
      if (flightData.takeoff_altitude_valid &&
          isfinite(flightData.takeoff_altitude))
      {
        if (!takeoff_baseline_valid)
        {
          /* Below the SRF05 range at startup: establish a baseline on the
           * first usable reading, then require a further measured rise. */
          takeoff_baseline_cm = flightData.takeoff_altitude;
          takeoff_baseline_valid = 1U;
        }
        if (flightData.takeoff_altitude >=
            takeoff_baseline_cm + TAKEOFF_MIN_CLIMB_CM)
        {
          liftoff_confirm_s += dt;
        }
        else
        {
          liftoff_confirm_s = 0.0;
        }
      }

      if (liftoff_confirm_s >= TAKEOFF_LIFTOFF_CONFIRM_S)
      {
        altitude_control_active = 1U;
        altitude_target_cm = flightData.takeoff_altitude;
        LogInformation(1001, "Liftoff confirmed: beginning controlled climb");
        /* Initial error is zero: keep the existing collective on this update. */
      }
      else if (liftoff_confirm_s == 0.0)
      {
        current_throttle_command = FlightControl_SlewValue(
            current_throttle_command,
            CoreHelpers_GetTakeoffThrottleLimit(&Config),
            Config.takeoffThrottleRampPerSecond * dt);
      }
    }
    else if (altitude_control_active)
    {
      const double previous_target_cm = altitude_target_cm;
      altitude_target_cm = FlightControl_SlewValue(
          altitude_target_cm, Config.takeoffAltitudeCm,
          TAKEOFF_CLIMB_RATE_CM_S * dt);
      /* D should damp measured vertical motion, not the moving target. */
      if (pid_altitude.previous_error_valid)
      {
        pid_altitude.previous_error += altitude_target_cm - previous_target_cm;
      }
      current_throttle_command = FlightControl_SlewValue(
          current_throttle_command, Config.hoverThrottle,
          Config.takeoffThrottleRampPerSecond * dt);
    }

    if (altitude_control_active)
    {
      altitude_modifier = PID_Calculate(
          &pid_altitude, altitude_target_cm - flightData.takeoff_altitude,
          dt, 1U);

      if (droneState.state == DRONE_STATE_TAKEOFF)
      {
        const uint8_t near_target =
            altitude_target_cm == Config.takeoffAltitudeCm &&
            fabs(flightData.takeoff_altitude - Config.takeoffAltitudeCm) <=
                TAKEOFF_SETTLE_TOLERANCE_CM &&
            (!flightData.Vz_valid ||
             (isfinite(flightData.Vz) &&
              fabs(flightData.Vz) <= TAKEOFF_SETTLE_SPEED_MS));
        takeoff_settled_s = near_target ? takeoff_settled_s + dt : 0.0;
      }
    }

    const double collective = current_throttle_command + altitude_modifier;

    // X-frame mixer: M1=front-left, M2=front-right,
    // M3=back-right, M4=back-left.
    m1_pid = pitch_modifier + roll_modifier - Gz_modifier;
    m2_pid = pitch_modifier - roll_modifier + Gz_modifier;
    m3_pid = -pitch_modifier - roll_modifier - Gz_modifier;
    m4_pid = -pitch_modifier + roll_modifier + Gz_modifier;

    Motors_Speed.front_left = collective + m1_pid;
    Motors_Speed.front_right = collective + m2_pid;
    Motors_Speed.back_right = collective + m3_pid;
    Motors_Speed.back_left = collective + m4_pid;

    const double output_limit =
        droneState.state == DRONE_STATE_TAKEOFF && !altitude_control_active
            ? CoreHelpers_GetTakeoffThrottleLimit(&Config)
            : Config.maxThrottle;
    FlightControl_DesaturateSpeeds(&Motors_Speed, Config.minThrottle,
                                   output_limit);
    mixer_limited =
        fabs(Motors_Speed.front_left - (collective + m1_pid)) > 0.001 ||
        fabs(Motors_Speed.front_right - (collective + m2_pid)) > 0.001 ||
        fabs(Motors_Speed.back_right - (collective + m3_pid)) > 0.001 ||
        fabs(Motors_Speed.back_left - (collective + m4_pid)) > 0.001;
  }
  else
  {
    current_throttle_command = 0.0;
    Motors_Reset(&Motors_Speed);
  }

  // Apply motor speeds
  if (Motors_SetSpeed(Motors_Speed) != HAL_OK)
  {
    FlightControl_Stop();
    return AppError_Set(APP_ERROR_MOTOR_OUTPUT, HAL_ERROR);
  }

  // Log flight data
  FlightLogData_t log_data = {.roll = flightData.roll_deg,
                              .pitch = flightData.pitch_deg,
                              .yaw = flightData.yaw_deg,
                              .Gz = flightData.Gz,
                              .roll_rate_dps = flightData.roll_rate_dps,
                              .pitch_rate_dps = flightData.pitch_rate_dps,
                              .Vz = flightData.Vz,
                              .Vz_valid = flightData.Vz_valid,
                              .altitude = flightData.takeoff_altitude,
                              .base_throttle = current_throttle_command,
                              .m1_speed = Motors_Speed.front_left,
                              .m2_speed = Motors_Speed.front_right,
                              .m3_speed = Motors_Speed.back_right,
                              .m4_speed = Motors_Speed.back_left,
                              .roll_p = pid_roll.p_term,
                              .roll_i = pid_roll.i_term,
                              .roll_d = pid_roll.d_term,
                              .roll_total = roll_modifier,
                              .pitch_p = pid_pitch.p_term,
                              .pitch_i = pid_pitch.i_term,
                              .pitch_d = pid_pitch.d_term,
                              .pitch_total = pitch_modifier,
                              .Gz_p = pid_Gz.p_term,
                              .Gz_i = pid_Gz.i_term,
                              .Gz_d = pid_Gz.d_term,
                              .altitude_p = pid_altitude.p_term,
                              .altitude_i = pid_altitude.i_term,
                              .altitude_d = pid_altitude.d_term,
                              .loop_dt_ms = dt * 1000.0,
                              .altitude_target = altitude_target_cm,
                              .altitude_control_active = altitude_control_active,
                              .mixer_limited = mixer_limited};
  Logger_LogFlightData(&log_data);
  return HAL_OK;
}

void FlightControl_DesaturateSpeeds(MotorSpeeds_t *speeds, float minThrottle,
                                    float maxThrottle)
{
  if (speeds == NULL)
  {
    return;
  }

  if (minThrottle < MOTOR_ABSOLUTE_MIN_THROTTLE)
  {
    minThrottle = MOTOR_ABSOLUTE_MIN_THROTTLE;
  }
  if (maxThrottle > MOTOR_ABSOLUTE_MAX_THROTTLE)
  {
    maxThrottle = MOTOR_ABSOLUTE_MAX_THROTTLE;
  }

  if (minThrottle > maxThrottle)
  {
    minThrottle = maxThrottle;
  }

  double *motor[4] = {&speeds->front_left, &speeds->front_right,
                      &speeds->back_right, &speeds->back_left};
  double minimum = *motor[0];
  double maximum = *motor[0];
  double center = 0.0;

  for (uint8_t index = 0U; index < 4U; index++)
  {
    center += *motor[index];
    minimum = fmin(minimum, *motor[index]);
    maximum = fmax(maximum, *motor[index]);
  }
  center *= 0.25;

  /*
   * If the requested attitude correction is wider than the available motor
   * range, scale all four corrections equally around their collective.
   */
  const double available_range =
      (double)maxThrottle - (double)minThrottle;
  const double requested_range = maximum - minimum;
  if (requested_range > available_range && requested_range > 0.0)
  {
    const double scale = available_range / requested_range;
    minimum = center;
    maximum = center;
    for (uint8_t index = 0U; index < 4U; index++)
    {
      *motor[index] = center + (*motor[index] - center) * scale;
      minimum = fmin(minimum, *motor[index]);
      maximum = fmax(maximum, *motor[index]);
    }
  }

  /*
   * Shift collective as one block. Unlike independent clipping, this keeps
   * the relative roll/pitch/yaw correction intact.
   */
  double shift = 0.0;
  if (maximum > maxThrottle)
  {
    shift = (double)maxThrottle - maximum;
  }
  if (minimum + shift < minThrottle)
  {
    shift += (double)minThrottle - (minimum + shift);
  }

  for (uint8_t index = 0U; index < 4U; index++)
  {
    const double shifted = *motor[index] + shift;
    *motor[index] = fmin((double)maxThrottle,
                         fmax((double)minThrottle, shifted));
  }
}

void FlightControl_Stop(void)
{
  current_throttle_command = 0.0;
  FlightControl_ResetTakeoff();
  PID_Reset(&pid_roll);
  PID_Reset(&pid_pitch);
  PID_Reset(&pid_Gz);
  PID_Reset(&pid_altitude);
  Motors_Reset(&Motors_Speed);
  (void)Motors_SetSpeed(Motors_Speed);
  flightData = SensorAcquisition_GetFlightData();
}

HAL_StatusTypeDef FlightControl_Start(void)
{
  return FlightControl_Init();
}

HAL_StatusTypeDef FlightControl_UpdateFlightData(double dt)
{
  (void)dt;
  flightData = SensorAcquisition_GetFlightData();
  if (!SensorAcquisition_IsReady())
  {
    return AppError_Set(APP_ERROR_SENSOR_ACQUISITION, HAL_BUSY);
  }
  return HAL_OK;
}
