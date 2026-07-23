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

HAL_StatusTypeDef FlightControl_Init(void)
{
  LogInformation(1002, "Initializing Flight Control...");

  AppError_Clear();
  SensorAcquisition_Init();
  flightData = SensorAcquisition_GetFlightData();

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

  const SensorAcquisitionResult_t sensor_result =
      SensorAcquisition_Start();
  flightData = SensorAcquisition_GetFlightData();
  const HAL_StatusTypeDef status =
      AppError_SetSensorAcquisitionResult(sensor_result);
  if (status != HAL_OK)
  {
    return status;
  }

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

  // Update flightData from sensors each cycle
  HAL_StatusTypeDef status = FlightControl_UpdateFlightData(dt);
  if (status != HAL_OK)
  {
    return status;
  }

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

      roll_modifier = PID_Calculate(&pid_roll, roll_error, dt);
      pitch_modifier = PID_Calculate(&pid_pitch, pitch_error, dt);
      Gz_modifier = PID_Calculate(&pid_Gz, gz_error, dt);
    }

    if (SENSOR_SRF05_ENABLED &&
        droneState.state == DRONE_STATE_FLYING)
    {
      const double altitude_error = TAKEOFF_TARGET_ALTITUDE_CM - flightData.takeoff_altitude;
      altitude_modifier = PID_Calculate(&pid_altitude, altitude_error, dt);
    }

    if (droneState.state == DRONE_STATE_ARMED)
    {
      // Arming starts every motor equally at the configured safe idle.
      current_throttle_command = Config.armThrottle;
    }
    else if (droneState.state == DRONE_STATE_TAKEOFF)
    {
      /* Search upward for the throttle needed to reach 50 cm. FLYING
       * preserves the achieved command by no longer changing it. */
      const double throttle_step = Config.takeoffThrottleRampPerSecond * dt;

      if (current_throttle_command < Config.maxSpeed)
      {
        current_throttle_command += throttle_step;
        if (current_throttle_command > Config.maxSpeed)
        {
          current_throttle_command = Config.maxSpeed;
        }
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

    FlightControl_ClampSpeeds(&Motors_Speed, Config.minSpeed, Config.maxSpeed);
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
                              .pitch_p = pid_pitch.p_term,
                              .pitch_i = pid_pitch.i_term,
                              .pitch_d = pid_pitch.d_term,
                              .Gz_p = pid_Gz.p_term,
                              .Gz_i = pid_Gz.i_term,
                              .Gz_d = pid_Gz.d_term,
                              .altitude_p = pid_altitude.p_term,
                              .altitude_i = pid_altitude.i_term,
                              .altitude_d = pid_altitude.d_term};
  Logger_LogFlightData(&log_data);
  return HAL_OK;
}

void FlightControl_ClampSpeeds(MotorSpeeds_t *speeds, float minSpeed, float maxSpeed)
{
  // Ensure speeds are within absolute limits first
  if (minSpeed < MOTOR_ABSOLUTE_MIN_SPEED)
    minSpeed = MOTOR_ABSOLUTE_MIN_SPEED;
  if (maxSpeed > MOTOR_ABSOLUTE_MAX_SPEED)
    maxSpeed = MOTOR_ABSOLUTE_MAX_SPEED;

  // Clamp all motor speeds
  speeds->front_left = (speeds->front_left < minSpeed)   ? minSpeed
                       : (speeds->front_left > maxSpeed) ? maxSpeed
                                                         : speeds->front_left;
  speeds->front_right = (speeds->front_right < minSpeed) ? minSpeed
                        : (speeds->front_right > maxSpeed)
                            ? maxSpeed
                            : speeds->front_right;
  speeds->back_left = (speeds->back_left < minSpeed)   ? minSpeed
                      : (speeds->back_left > maxSpeed) ? maxSpeed
                                                       : speeds->back_left;
  speeds->back_right = (speeds->back_right < minSpeed)   ? minSpeed
                       : (speeds->back_right > maxSpeed) ? maxSpeed
                                                         : speeds->back_right;
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
  SensorAcquisition_Stop();
  flightData = SensorAcquisition_GetFlightData();
}

HAL_StatusTypeDef FlightControl_Start(void)
{
  return FlightControl_Init();
}

HAL_StatusTypeDef FlightControl_UpdateFlightData(double dt)
{
  const SensorAcquisitionResult_t result =
      SensorAcquisition_Update(dt);
  flightData = SensorAcquisition_GetFlightData();
  return AppError_SetSensorAcquisitionResult(result);
}
