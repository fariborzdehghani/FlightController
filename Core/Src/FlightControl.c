#include "FlightControl.h"
#include "AppError.h"
#include "Core.h"
#include "Logger.h"
#include "MotorControl.h"
#include "SensorAcquisition.h"
#include "SensorConfig.h"
#include "Tools.h"
#include <math.h>

static double current_throttle_command;

static double FlightControl_SlewThrottle(double current, double target,
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

  if (droneState.state == DRONE_STATE_FLYING &&
      !flightData.takeoff_altitude_valid)
  {
    FlightControl_Stop();
    return AppError_Set(APP_ERROR_ALTITUDE_UNAVAILABLE, HAL_ERROR);
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

    if (SENSOR_SRF05_ENABLED && flightData.takeoff_altitude_valid &&
        (droneState.state == DRONE_STATE_TAKEOFF ||
         droneState.state == DRONE_STATE_FLYING))
    {
      const double altitude_error =
          Config.takeoffAltitudeCm - flightData.takeoff_altitude;

      if (droneState.state == DRONE_STATE_FLYING)
      {
        altitude_modifier =
            PID_Calculate(&pid_altitude, altitude_error, dt, 1U);
      }
      else
      {
        /* Remember the approach error without running the altitude PID. On
         * the first FLYING update, D can therefore react to upward motion
         * instead of being forced to zero by an uninitialized history. */
        pid_altitude.previous_error = altitude_error;
        pid_altitude.previous_error_valid = 1U;
      }
    }

    if (droneState.state == DRONE_STATE_ARMED)
    {
      // Arming starts every motor equally at the configured safe idle.
      current_throttle_command = Config.armThrottle;
    }
    else if (droneState.state == DRONE_STATE_TAKEOFF)
    {
      /* Search upward for liftoff, but do not add another ramp step on the
       * sample that has already reached the target altitude. */
      const double throttle_step = Config.takeoffThrottleRampPerSecond * dt;

      if ((!flightData.takeoff_altitude_valid ||
           flightData.takeoff_altitude < Config.takeoffAltitudeCm) &&
          current_throttle_command < Config.maxThrottle)
      {
        current_throttle_command += throttle_step;
        if (current_throttle_command > Config.maxThrottle)
        {
          current_throttle_command = Config.maxThrottle;
        }
      }
    }
    else if (droneState.state == DRONE_STATE_FLYING)
    {
      /* Preserve output continuity at the state change, then remove the
       * takeoff-ramp bias gradually while the altitude PID takes control. */
      const double throttle_step = Config.takeoffThrottleRampPerSecond * dt;
      current_throttle_command = FlightControl_SlewThrottle(
          current_throttle_command, Config.hoverThrottle, throttle_step);
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

    FlightControl_DesaturateSpeeds(&Motors_Speed, Config.minThrottle,
                                   Config.maxThrottle);
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
                              .altitude_d = pid_altitude.d_term};
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
