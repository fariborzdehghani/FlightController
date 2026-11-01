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

#define TAKEOFF_CONFIRM_MIN_ECHOES 3U
#define TAKEOFF_MAX_ECHO_GAP_MS 300U

static double current_throttle_command;
static uint8_t manual_throttle_active;
static uint8_t manual_height_seen;
static double hover_throttle_estimate;
static double hover_learning_dwell_s;
static uint32_t hover_learning_last_echo_ms;
static uint8_t previous_mixer_limited;
static uint32_t takeoff_started_ms;
static uint8_t takeoff_baseline_valid;
static double takeoff_baseline_cm;
static uint8_t takeoff_height_seen;
static uint8_t takeoff_recovery_active;
static uint8_t liftoff_confirm_echoes;
static uint8_t liftoff_confirming_rise;
static uint32_t liftoff_first_echo_ms;
static uint32_t liftoff_last_echo_ms;
static uint8_t takeoff_collective_slew_active;
static uint8_t altitude_control_active;
static double altitude_target_cm;
static double takeoff_settled_s;
static uint8_t landing_active;
static uint8_t landing_complete;
static uint8_t landing_descent_active;
static uint32_t landing_descent_timeout_ms;
static uint32_t landing_started_ms;
static double landing_initial_throttle;
static double landing_initial_output_limit;
static AppErrorCode_t landing_reason;

static void FlightControl_ResetTakeoff(void)
{
  manual_throttle_active = 0U;
  manual_height_seen = 0U;
  takeoff_started_ms = 0U;
  takeoff_baseline_valid = 0U;
  takeoff_baseline_cm = 0.0;
  takeoff_height_seen = 0U;
  takeoff_recovery_active = 0U;
  liftoff_confirm_echoes = 0U;
  liftoff_confirming_rise = 0U;
  liftoff_first_echo_ms = 0U;
  liftoff_last_echo_ms = 0U;
  takeoff_collective_slew_active = 0U;
  altitude_control_active = 0U;
  altitude_target_cm = 0.0;
  takeoff_settled_s = 0.0;
  hover_learning_dwell_s = 0.0;
  hover_learning_last_echo_ms = 0U;
  previous_mixer_limited = 0U;
  landing_active = 0U;
  landing_complete = 0U;
  landing_descent_active = 0U;
  landing_descent_timeout_ms = 0U;
  landing_started_ms = 0U;
  landing_initial_throttle = 0.0;
  landing_initial_output_limit = 0.0;
  landing_reason = APP_ERROR_NONE;
}

static void FlightControl_BeginLanding(AppErrorCode_t reason)
{
  if (landing_active && !landing_descent_active) return;
  landing_active = 1U;
  manual_throttle_active = 0U;
  landing_descent_active = 0U;
  landing_reason = reason;
  landing_started_ms = HAL_GetTick();
  /* Continue from the applied collective, including altitude PID and mixer
   * limiting. Substituting a hover estimate could cause a throttle jump. */
  landing_initial_throttle = 0.25 *
      (Motors_Speed.front_left + Motors_Speed.front_right +
       Motors_Speed.back_right + Motors_Speed.back_left);
  landing_initial_output_limit = fmax(
      fmax(Motors_Speed.front_left, Motors_Speed.front_right),
      fmax(Motors_Speed.back_right, Motors_Speed.back_left));
  current_throttle_command = landing_initial_throttle;
  altitude_control_active = 0U;
  takeoff_recovery_active = 0U;
  takeoff_collective_slew_active = 0U;
  takeoff_settled_s = 0.0;
  PID_Reset(&pid_altitude);
  if (reason == APP_ERROR_NONE)
  {
    LogInformation(1001, "LAND: gradual throttle-down landing started");
  }
  else
  {
    LogError(2007, reason == APP_ERROR_ALTITUDE_UNAVAILABLE
                     ? "Altitude lost: gradual throttle-down landing started"
                     : "Takeoff timeout: gradual throttle-down landing started");
  }
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

static uint8_t FlightControl_HasFreshTakeoffHeight(void)
{
  const Srf05Trace_t *trace = &flightData.srf05_trace;
  return flightData.takeoff_altitude_valid &&
      isfinite(flightData.takeoff_altitude) &&
      flightData.takeoff_altitude >= 0.0 && trace->valid &&
      (trace->flags & SRF05_TRACE_HAVE_ACCEPTED) &&
      HAL_GetTick() - trace->last_accepted_ms <= TAKEOFF_MAX_ECHO_GAP_MS;
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
  hover_throttle_estimate = Config.armThrottle;
  takeoff_started_ms = HAL_GetTick();
  flightData = SensorAcquisition_GetFlightData();
  takeoff_height_seen = FlightControl_HasFreshTakeoffHeight();
  if (takeoff_height_seen &&
      flightData.takeoff_altitude < Config.takeoffAltitudeCm)
  {
    takeoff_baseline_cm = flightData.takeoff_altitude;
    takeoff_baseline_valid = 1U;
  }
}

uint8_t FlightControl_IsTakeoffComplete(void)
{
  return !landing_active && altitude_control_active &&
         takeoff_settled_s >= TAKEOFF_SETTLE_TIME_S;
}

uint8_t FlightControl_IsLanding(void) { return landing_active; }

uint8_t FlightControl_IsManualThrottle(void) { return manual_throttle_active; }

uint8_t FlightControl_AdjustThrottle(int8_t direction)
{
  if ((direction != 1 && direction != -1) || landing_active ||
      (droneState.state != DRONE_STATE_ARMED &&
       droneState.state != DRONE_STATE_TAKEOFF &&
       droneState.state != DRONE_STATE_FLYING)) return 0U;

  if (!manual_throttle_active)
  {
    /* Include the applied altitude correction and mixer limits at takeover.
     * ARM can be followed by this command before the first motor update. */
    current_throttle_command = 0.25 *
        (Motors_Speed.front_left + Motors_Speed.front_right +
         Motors_Speed.back_right + Motors_Speed.back_left);
    if (droneState.state != DRONE_STATE_FLYING && current_throttle_command == 0.0)
      current_throttle_command = Config.armThrottle;
    manual_throttle_active = 1U;
    manual_height_seen = altitude_control_active || takeoff_height_seen ||
                         droneState.state == DRONE_STATE_FLYING;
    altitude_control_active = 0U;
    takeoff_recovery_active = 0U;
    takeoff_collective_slew_active = 0U;
    takeoff_settled_s = 0.0;
    PID_Reset(&pid_altitude);
    LogInformation(1001, "Manual throttle: automatic climb and altitude hold stopped");
  }
  current_throttle_command = fmax(Config.minThrottle,
      fmin(Config.maxThrottle, current_throttle_command + direction));
  return 1U;
}

double FlightControl_GetHoverThrottleEstimate(void)
{
  return hover_throttle_estimate;
}

static void FlightControl_LearnHoverThrottle(double dt)
{
  /* A held final target and low measured speed also allow a small steady
   * height error to be trimmed out when altitude Ki is zero. Requiring zero
   * height error here would prevent learning that very correction. */
  const double pid_sum = pid_altitude.p_term + pid_altitude.i_term +
                         pid_altitude.d_term;
  const uint8_t steady = (droneState.state == DRONE_STATE_TAKEOFF ||
      droneState.state == DRONE_STATE_FLYING) &&
      altitude_control_active && !landing_active &&
      !takeoff_collective_slew_active && !previous_mixer_limited &&
      altitude_target_cm == Config.takeoffAltitudeCm &&
      FlightControl_HasFreshTakeoffHeight() && flightData.Vz_valid &&
      isfinite(flightData.Vz) &&
      fabs(flightData.Vz) <= HOVER_LEARN_MAX_SPEED_MS &&
      fabs(flightData.roll_deg) <= HOVER_LEARN_MAX_TILT_DEG &&
      fabs(flightData.pitch_deg) <= HOVER_LEARN_MAX_TILT_DEG &&
      fabs(flightData.roll_rate_dps) <= 5.0 &&
      fabs(flightData.pitch_rate_dps) <= 5.0 &&
      fabs(flightData.Gz) <= 5.0 &&
      pid_sum > pid_altitude.min_output && pid_sum < pid_altitude.max_output;
  if (!steady)
  {
    hover_learning_dwell_s = 0.0;
    hover_learning_last_echo_ms = 0U;
    return;
  }

  /* Only distinct accepted echoes build the dwell; cached control updates
   * cannot make one measurement look like a second of steady hover. */
  const uint32_t echo_ms = flightData.srf05_trace.last_accepted_ms;
  if (hover_learning_last_echo_ms != echo_ms)
  {
    if (hover_learning_dwell_s > 0.0 &&
        (uint32_t)(echo_ms - hover_learning_last_echo_ms) <= TAKEOFF_MAX_ECHO_GAP_MS)
      hover_learning_dwell_s += (uint32_t)(echo_ms - hover_learning_last_echo_ms) * 0.001;
    else
      hover_learning_dwell_s = dt;
    hover_learning_last_echo_ms = echo_ms;
  }
  if (hover_learning_dwell_s < HOVER_LEARN_DWELL_S) return;

  const double applied = 0.25 * (Motors_Speed.front_left +
      Motors_Speed.front_right + Motors_Speed.back_right + Motors_Speed.back_left);
  const double filtered = current_throttle_command +
      (applied - current_throttle_command) * dt / (HOVER_LEARN_TIME_CONSTANT_S + dt);
  const double learned = fmax(Config.minThrottle, fmin(Config.maxThrottle,
      FlightControl_SlewValue(current_throttle_command, filtered,
                             HOVER_LEARN_RATE_PER_SECOND * dt)));
  const double change = learned - current_throttle_command;

  /* Move accumulated I into the baseline without counting it twice. With no
   * I (or at its bound), the remaining change is still limited to 1 pp/s.
   * Run before the next PID calculation so telemetry matches applied output. */
  if (pid_altitude.Ki > 0.0)
  {
    pid_altitude.i_term = fmax(pid_altitude.min_i_term,
        fmin(pid_altitude.max_i_term, pid_altitude.i_term - change));
    pid_altitude.integral = pid_altitude.i_term / pid_altitude.Ki;
  }
  current_throttle_command = learned;
  hover_throttle_estimate = learned;
}

uint8_t FlightControl_IsTakeoffRecovery(void)
{
  return droneState.state == DRONE_STATE_TAKEOFF && !landing_active &&
      !altitude_control_active && takeoff_recovery_active;
}

void FlightControl_RequestLanding(void)
{
  if (landing_active) return;
  if (droneState.state == DRONE_STATE_ARMED ||
      droneState.state == DRONE_STATE_TAKEOFF ||
      droneState.state == DRONE_STATE_FLYING)
  {
    flightData = SensorAcquisition_GetFlightData();
    if ((altitude_control_active || manual_throttle_active) &&
        flightData.takeoff_altitude_valid &&
        isfinite(flightData.takeoff_altitude) &&
        flightData.takeoff_altitude > LANDING_NEAR_GROUND_CM)
    {
      landing_active = 1U;
      manual_throttle_active = 0U;
      altitude_control_active = 1U;
      landing_descent_active = 1U;
      landing_reason = APP_ERROR_NONE;
      landing_started_ms = HAL_GetTick();
      landing_descent_timeout_ms = (uint32_t)fmin(LANDING_DESCENT_MAX_TIME_MS,
          (flightData.takeoff_altitude - LANDING_NEAR_GROUND_CM) * 1000.0 /
              LANDING_DESCENT_RATE_CM_S + LANDING_DESCENT_MARGIN_MS);
      // Absorb the applied altitude correction into the base and start at
      // measured height, avoiding a throttle jump or a continued takeoff climb.
      current_throttle_command = 0.25 *
          (Motors_Speed.front_left + Motors_Speed.front_right +
           Motors_Speed.back_right + Motors_Speed.back_left);
      altitude_target_cm = flightData.takeoff_altitude;
      takeoff_settled_s = 0.0;
      takeoff_collective_slew_active = 0U;
      PID_Reset(&pid_altitude);
      LogInformation(1001, "LAND requested: controlled altitude descent started");
      return;
    }
    FlightControl_BeginLanding(APP_ERROR_NONE);
  }
}

uint8_t FlightControl_IsLandingComplete(void) { return landing_complete; }

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
  hover_throttle_estimate = Config.armThrottle;
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
    FlightControl_Stop();
    return AppError_Set(APP_ERROR_INVALID_FLIGHT_TIMESTEP, HAL_ERROR);
  }

  // Sensor acquisition runs continuously in Core_loop(). Consume its latest
  // cached snapshot without touching the sensor buses from flight control.
  flightData = SensorAcquisition_GetFlightData();

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
      SENSOR_MPU6050_ENABLED && motors_running &&
      (manual_throttle_active || landing_active ||
       droneState.state == DRONE_STATE_TAKEOFF ||
       droneState.state == DRONE_STATE_FLYING);

  if (stabilization_active &&
      (!isfinite(flightData.roll_deg) || !isfinite(flightData.pitch_deg) ||
       !isfinite(flightData.roll_rate_dps) || !isfinite(flightData.pitch_rate_dps)))
  {
    FlightControl_Stop();
    return AppError_Set(APP_ERROR_MPU6050_READ, HAL_ERROR);
  }

  if (stabilization_active &&
      (fabs(flightData.roll_deg) > Config.maxAngle ||
       fabs(flightData.pitch_deg) > Config.maxAngle))
  {
    FlightControl_Stop();
    return AppError_Set(APP_ERROR_MAXIMUM_FLIGHT_ANGLE, HAL_ERROR);
  }

  /* Near-ground echoes can appear/disappear before liftoff. Only confirmed
   * liftoff engages the in-flight altitude-loss policy. Acquisition already
   * tolerates 300 ms without an accepted echo. */
  if (manual_throttle_active && FlightControl_HasFreshTakeoffHeight())
    manual_height_seen = 1U;
  if (!landing_active &&
      (droneState.state == DRONE_STATE_FLYING || altitude_control_active ||
       (manual_throttle_active && manual_height_seen)) &&
      (!flightData.takeoff_altitude_valid ||
       !isfinite(flightData.takeoff_altitude)))
  {
    if (!Config.disableSrf05Failsafe)
    {
      FlightControl_BeginLanding(APP_ERROR_ALTITUDE_UNAVAILABLE);
    }
    else if (altitude_control_active)
    {
      /* Hold the applied collective with attitude stabilization. Never feed
       * invalid/cached altitude into the PID, or resume it on a sudden echo. */
      current_throttle_command = 0.25 *
          (Motors_Speed.front_left + Motors_Speed.front_right +
           Motors_Speed.back_right + Motors_Speed.back_left);
      altitude_control_active = 0U;
      takeoff_recovery_active = 1U;
      takeoff_collective_slew_active = 0U;
      takeoff_settled_s = 0.0;
      PID_Reset(&pid_altitude);
      LogInformation(1001, "SRF05 failsafe disabled: altitude lost; holding collective");
    }
  }

  if (droneState.state == DRONE_STATE_TAKEOFF && !landing_active &&
      !manual_throttle_active &&
      !Config.disableSrf05Failsafe)
  {
    const uint32_t elapsed_ms = HAL_GetTick() - takeoff_started_ms;
    if (!altitude_control_active &&
        elapsed_ms >= CoreHelpers_GetTakeoffLiftoffTimeoutMs(&Config))
    {
      FlightControl_BeginLanding(APP_ERROR_TAKEOFF_LIFTOFF_TIMEOUT);
    }
    if (!landing_active && elapsed_ms >= CoreHelpers_GetTakeoffTargetTimeoutMs(&Config))
    {
      FlightControl_BeginLanding(APP_ERROR_TAKEOFF_TARGET_TIMEOUT);
    }
  }

  if (landing_descent_active)
  {
    const uint8_t altitude_usable = flightData.takeoff_altitude_valid &&
                                   isfinite(flightData.takeoff_altitude);
    if (!altitude_usable ||
        flightData.takeoff_altitude <= LANDING_NEAR_GROUND_CM ||
        (uint32_t)(HAL_GetTick() - landing_started_ms) >= landing_descent_timeout_ms)
    {
      LogInformation(1001, !altitude_usable
          ? "LAND: altitude unavailable; finishing with gradual throttle-down"
          : flightData.takeoff_altitude <= LANDING_NEAR_GROUND_CM
              ? "LAND: near ground; finishing with gradual throttle-down"
              : "LAND: descent timeout; finishing with gradual throttle-down");
      FlightControl_BeginLanding(APP_ERROR_NONE);
    }
  }

  if (landing_active && !landing_descent_active)
  {
    const double elapsed_s = (uint32_t)(HAL_GetTick() - landing_started_ms) * 0.001;
    current_throttle_command = fmax(0.0, landing_initial_throttle -
        LANDING_THROTTLE_RAMP_DOWN_PER_SECOND * elapsed_s);
    if (current_throttle_command <= 0.0)
    {
      const AppErrorCode_t reason = landing_reason;
      FlightControl_Stop();
      LogInformation(1004, "Gradual throttle-down complete: motors stopped");
      if (reason == APP_ERROR_NONE)
      {
        landing_complete = 1U;
        return HAL_OK;
      }
      return AppError_Set(reason, HAL_ERROR);
    }
  }

  FlightControl_LearnHoverThrottle(dt);
  PID_ClearTerms(&pid_roll);
  PID_ClearTerms(&pid_pitch);
  PID_ClearTerms(&pid_Gz);
  PID_ClearTerms(&pid_altitude);

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
          landing_active ? PID_INTEGRAL_HOLD : droneState.state == DRONE_STATE_FLYING;

      /* These rates already have startup bias calibration and the MPU DLPF.
       * Fusion angle corrections must not be interpreted as physical rotation. */
      roll_modifier = PID_CalculateWithRate(&pid_roll, roll_error,
          flightData.roll_rate_dps, dt, attitude_integral_enabled);
      pitch_modifier = PID_CalculateWithRate(&pid_pitch, pitch_error,
          flightData.pitch_rate_dps, dt, attitude_integral_enabled);
      Gz_modifier = PID_Calculate(&pid_Gz, gz_error, dt,
                                  attitude_integral_enabled);
    }

    /* Final throttle-down never re-enters takeoff or altitude control, even
     * if valid sensor readings return. Controlled LAND only lowers its target. */
    if (!landing_active && !manual_throttle_active &&
        droneState.state == DRONE_STATE_ARMED)
    {
      // Arming starts every motor equally at the configured safe idle.
      current_throttle_command = Config.armThrottle;
    }
    else if (!landing_active && !manual_throttle_active &&
             droneState.state == DRONE_STATE_TAKEOFF &&
             !altitude_control_active)
    {
      const uint8_t height_usable = FlightControl_HasFreshTakeoffHeight();
      if (height_usable)
      {
        takeoff_height_seen = 1U;
      }
      else if (takeoff_height_seen && !takeoff_recovery_active)
      {
        takeoff_recovery_active = 1U;
        /* Keep the existing base command. Attitude corrections and mixer
         * limits can still change individual motor outputs while paused. */
        LogInformation(1001, "Takeoff height lost: throttle ramp paused for echo recovery");
      }
      const uint8_t at_target = height_usable &&
          flightData.takeoff_altitude >= Config.takeoffAltitudeCm;
      uint8_t rise_observed = 0U;
      if (height_usable)
      {
        if (!takeoff_baseline_valid && !at_target)
        {
          /* Below the SRF05 range at startup: establish a baseline on the
           * first usable reading, then require a further measured rise. */
          takeoff_baseline_cm = flightData.takeoff_altitude;
          takeoff_baseline_valid = 1U;
        }
        rise_observed = at_target || (takeoff_baseline_valid &&
            flightData.takeoff_altitude >=
                takeoff_baseline_cm + TAKEOFF_MIN_CLIMB_CM);
        const Srf05Trace_t *trace = &flightData.srf05_trace;
        if (rise_observed || takeoff_recovery_active)
        {
          /* Several control loops may reuse one accepted echo. Only new
           * measurements may establish the 150 ms liftoff dwell. */
          if (liftoff_confirm_echoes == 0U ||
              liftoff_confirming_rise != rise_observed ||
              trace->last_accepted_ms - liftoff_last_echo_ms > TAKEOFF_MAX_ECHO_GAP_MS)
          {
            liftoff_first_echo_ms = trace->last_accepted_ms;
            liftoff_confirm_echoes = 1U;
          }
          else if (trace->last_accepted_ms != liftoff_last_echo_ms &&
                   liftoff_confirm_echoes < TAKEOFF_CONFIRM_MIN_ECHOES)
          {
            liftoff_confirm_echoes++;
          }
          liftoff_confirming_rise = rise_observed;
          liftoff_last_echo_ms = trace->last_accepted_ms;
        }
        else
        {
          liftoff_confirm_echoes = 0U;
        }
      }
      else
      {
        liftoff_confirm_echoes = 0U;
      }

      if (liftoff_confirm_echoes >= TAKEOFF_CONFIRM_MIN_ECHOES &&
          (double)(liftoff_last_echo_ms - liftoff_first_echo_ms) * 0.001 >=
              TAKEOFF_LIFTOFF_CONFIRM_S)
      {
        takeoff_recovery_active = 0U;
        if (rise_observed)
        {
          altitude_control_active = 1U;
          altitude_target_cm = fmin(flightData.takeoff_altitude,
                                    Config.takeoffAltitudeCm);
          takeoff_collective_slew_active =
              flightData.takeoff_altitude > Config.takeoffAltitudeCm;
          /* Seed from applied collective, including any mixer limiting. */
          current_throttle_command = 0.25 *
              (Motors_Speed.front_left + Motors_Speed.front_right +
               Motors_Speed.back_right + Motors_Speed.back_left);
          hover_throttle_estimate = current_throttle_command;
          LogInformation(1001, "Liftoff confirmed: beginning controlled climb");
          /* Below the goal the initial error is zero. Above it, correct toward
           * the real goal with a bounded collective change, never a higher goal. */
        }
        else
        {
          /* Consistent echoes returned near the baseline. Resume the bounded
           * ramp only after fresh confirmation, not on one cached recovery. */
          liftoff_confirm_echoes = 0U;
          LogInformation(1001, "Takeoff height recovered: bounded throttle ramp resumed");
        }
      }
      else if (liftoff_confirm_echoes == 0U && !at_target && !takeoff_recovery_active)
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
          altitude_target_cm,
          landing_descent_active ? LANDING_NEAR_GROUND_CM : Config.takeoffAltitudeCm,
          (landing_descent_active ? LANDING_DESCENT_RATE_CM_S : TAKEOFF_CLIMB_RATE_CM_S) * dt);
      /* D should damp measured vertical motion, not the moving target. */
      if (pid_altitude.previous_error_valid)
      {
        pid_altitude.previous_error += altitude_target_cm - previous_target_cm;
      }
      /* Hold the measured liftoff baseline through the climb. Height feedback
       * supplies the climb/braking correction; steady flight refines the base. */
    }

    if (altitude_control_active)
    {
      altitude_modifier = PID_Calculate(
          &pid_altitude, altitude_target_cm - flightData.takeoff_altitude,
          dt, takeoff_collective_slew_active ? PID_INTEGRAL_HOLD : 1U);

      if (droneState.state == DRONE_STATE_TAKEOFF && !landing_active)
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

    double collective = current_throttle_command + altitude_modifier;
    if (takeoff_collective_slew_active && !landing_active)
    {
      const double previous_collective = 0.25 *
          (Motors_Speed.front_left + Motors_Speed.front_right +
           Motors_Speed.back_right + Motors_Speed.back_left);
      const double requested_collective = collective;
      collective = FlightControl_SlewValue(previous_collective,
          requested_collective, Config.takeoffThrottleRampPerSecond * dt);
      if (collective == requested_collective)
        takeoff_collective_slew_active = 0U;
    }

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

    const uint8_t throttle_down_active = landing_active && !landing_descent_active;
    const double output_limit = throttle_down_active
        ? landing_initial_output_limit * current_throttle_command / landing_initial_throttle
        : droneState.state == DRONE_STATE_TAKEOFF && !altitude_control_active
            ? CoreHelpers_GetTakeoffThrottleLimit(&Config)
            : Config.maxThrottle;
    FlightControl_DesaturateSpeeds(&Motors_Speed, throttle_down_active ? 0.0f : Config.minThrottle,
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

  previous_mixer_limited = mixer_limited;

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
                              .mixer_limited = mixer_limited,
                              .landing_active = landing_active,
                              .attitude_trace = flightData.attitude_trace,
                              .srf05_trace = flightData.srf05_trace};
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
