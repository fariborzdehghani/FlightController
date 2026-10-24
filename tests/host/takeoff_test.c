#include "FlightControl.h"
#include "AppError.h"
#include "CoreHelpers.h"
#include "Logger.h"
#include "MotorControl.h"
#include "SensorAcquisition.h"
#include "Tools.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

DroneConfig_t Config;
DroneStateManager_t droneState;
FlightData_t flightData;
MotorSpeeds_t Motors_Speed;
PID_t pid_roll, pid_pitch, pid_Gz, pid_altitude;
static FlightData_t sensor;
static MotorSpeeds_t output;
static FlightLogData_t log_data;
static uint32_t now_ms;
static unsigned liftoff_messages;
static unsigned landing_messages;

uint32_t HAL_GetTick(void) { return now_ms; }
uint8_t SensorAcquisition_IsReady(void) { return 1U; }
FlightData_t SensorAcquisition_GetFlightData(void) { return sensor; }
void Motors_Reset(MotorSpeeds_t *speeds) { memset(speeds, 0, sizeof(*speeds)); }
HAL_StatusTypeDef Motors_SetSpeed(MotorSpeeds_t speeds)
{
  output = speeds;
  assert(isfinite(output.front_left) && isfinite(output.front_right));
  assert(isfinite(output.back_left) && isfinite(output.back_right));
  return HAL_OK;
}
void Logger_LogFlightData(const FlightLogData_t *data) { log_data = *data; }
bool LogInformation(int code, const char *message)
{
  (void)code;
  if (strstr(message, "Liftoff confirmed")) liftoff_messages++;
  return true;
}

bool LogError(int code, const char *message)
{
  (void)message;
  if (code == 2007) landing_messages++;
  return true;
}

static void close_to(double actual, double expected)
{
  assert(fabs(actual - expected) < 0.00001);
}

static void stopped(void)
{
  close_to(output.front_left, 0.0);
  close_to(output.front_right, 0.0);
  close_to(output.back_left, 0.0);
  close_to(output.back_right, 0.0);
  assert(!FlightControl_IsTakeoffComplete());
  assert(!FlightControl_IsLanding());
}

static HAL_StatusTypeDef step(void)
{
  now_ms += 10U;
  return FlightControl_Update(0.01);
}

static void run(unsigned count)
{
  for (unsigned i = 0; i < count; i++) assert(step() == HAL_OK);
}

static void setup(uint8_t altitude_valid)
{
  Config = (DroneConfig_t){
      .minThrottle = 10, .armThrottle = 10, .hoverThrottle = 40,
      .maxThrottle = 80, .maxAngle = 30, .takeoffAltitudeCm = 50,
      .takeoffThrottleRampPerSecond = 10, .pidMaxIPart = 10,
      .pidMaxOutput = 20};
  sensor = (FlightData_t){.takeoff_altitude = 10,
      .takeoff_altitude_valid = altitude_valid, .Vz_valid = 1};
  now_ms = 0;
  liftoff_messages = 0;
  landing_messages = 0;
  droneState.state = DRONE_STATE_ARMED;
  PID_Init(&pid_roll, 1, 0.1, 0);
  PID_Init(&pid_pitch, 1, 0.1, 0);
  PID_Init(&pid_Gz, 1, 0.1, 0);
  PID_Init(&pid_altitude, 0.5, 0.1, 0.2);
  assert(FlightControl_Start() == HAL_OK);
  FlightControl_BeginTakeoff();
  droneState.state = DRONE_STATE_TAKEOFF;
}

static double max_output(void)
{
  return fmax(fmax(output.front_left, output.front_right),
              fmax(output.back_left, output.back_right));
}

static void finish_landing(AppErrorCode_t reason)
{
  assert(FlightControl_IsLanding());
  unsigned count = 0;
  double previous_max = max_output();
  while (FlightControl_IsLanding())
  {
    const HAL_StatusTypeDef status = step();
    assert(max_output() <= previous_max + 0.00001);
    previous_max = max_output();
    assert(output.front_left >= 0 && output.front_right >= 0 &&
           output.back_left >= 0 && output.back_right >= 0);
    assert(++count <= 1001);
    if (status != HAL_OK)
    {
      assert(status == HAL_ERROR && AppError_GetCode() == reason);
      stopped();
      return;
    }
    assert(log_data.landing_active && !log_data.altitude_control_active);
    close_to(pid_altitude.p_term, 0);
    close_to(pid_altitude.i_term, 0);
    close_to(pid_altitude.d_term, 0);
    assert(!FlightControl_IsTakeoffComplete());
  }
  assert(0 && "Landing must report its original fault after stopping");
}

static void lift(void)
{
  run(300); /* Throttle reaches the configured hover estimate. */
  sensor.takeoff_altitude += 5;
  run(15);
  assert(liftoff_messages == 1);
}

static double current_target(void)
{
  /* Recover the setpoint from the actual controller P term, not a test getter. */
  return sensor.takeoff_altitude + pid_altitude.p_term / pid_altitude.Kp;
}

static void test_bounded_ramp_and_no_liftoff_timeout(void)
{
  for (unsigned valid = 0; valid < 2; valid++)
  {
    setup(valid);
    sensor.roll_deg = 10; /* Exercise motor limits with attitude correction. */
    const uint32_t deadline = CoreHelpers_GetTakeoffLiftoffTimeoutMs(&Config);
    while (now_ms + 10 < deadline)
    {
      assert(step() == HAL_OK);
      assert(output.front_left <= 45.00001);
      assert(output.front_right <= 45.00001);
      assert(output.back_left <= 45.00001);
      assert(output.back_right <= 45.00001);
      close_to(pid_altitude.i_term, 0);
      close_to(pid_roll.i_term, 0);
    }
    assert(step() == HAL_OK);
    assert(landing_messages == 1);
    finish_landing(APP_ERROR_TAKEOFF_LIFTOFF_TIMEOUT);
  }
  setup(0);
  Config.takeoffThrottleRampPerSecond = 1;
  assert(CoreHelpers_GetTakeoffLiftoffTimeoutMs(&Config) == 10000);
  run(999);
  assert(step() == HAL_OK);
  finish_landing(APP_ERROR_TAKEOFF_LIFTOFF_TIMEOUT);
}

static void test_first_reading_and_transient_rise(void)
{
  setup(0);
  run(20);
  sensor.takeoff_altitude_valid = 1;
  sensor.takeoff_altitude = 8;
  run(20);
  assert(liftoff_messages == 0); /* A usable echo alone is not liftoff. */
  sensor.takeoff_altitude = 13;
  run(10);
  const double paused_throttle = log_data.base_throttle;
  sensor.takeoff_altitude = 8;
  run(1);
  assert(log_data.base_throttle > paused_throttle);
  assert(liftoff_messages == 0);
  sensor.takeoff_altitude = 13;
  run(14);
  assert(liftoff_messages == 0); /* Confirmation restarted after the dip. */
  run(1);
  assert(liftoff_messages == 1);
}

static void test_climb_engagement_and_derivative(void)
{
  setup(1);
  lift();
  close_to(log_data.base_throttle, 40);
  close_to(pid_altitude.p_term, 0);
  close_to(pid_altitude.d_term, 0);
  close_to(output.front_left, 40); /* No output jump on controller engagement. */
  run(1);
  close_to(current_target(), 15.2);
  close_to(pid_altitude.d_term, 0); /* Moving target introduces no D kick. */
  sensor.takeoff_altitude += 0.1;
  run(1);
  close_to(current_target(), 15.4);
  close_to(pid_altitude.d_term, -2); /* Upward motion is damped in cm/s. */
}

static void test_settling_and_continuous_hover(void)
{
  setup(1);
  lift();
  sensor.takeoff_altitude = 50;
  run(50);
  assert(!FlightControl_IsTakeoffComplete()); /* Actual height alone is insufficient. */
  run(130); /* Let the moving target finish its climb. */
  sensor.Vz = 0.3;
  run(60);
  assert(!FlightControl_IsTakeoffComplete()); /* Crossing the band while climbing. */
  sensor.Vz = 0;
  run(30);
  sensor.takeoff_altitude = 56;
  run(1);
  sensor.takeoff_altitude = 50;
  run(49);
  assert(!FlightControl_IsTakeoffComplete()); /* A height excursion resets dwell. */
  run(1);
  assert(FlightControl_IsTakeoffComplete());
  const double collective = output.front_left;
  const double integral = pid_altitude.integral;
  droneState.state = DRONE_STATE_FLYING;
  run(1);
  close_to(output.front_left, collective);
  close_to(pid_altitude.integral, integral);
  close_to(current_target(), 50);
}

static void test_full_height_budget_and_target_timeout(void)
{
  setup(1);
  Config.takeoffAltitudeCm = 300;
  lift();
  for (unsigned i = 0; i < 1430; i++)
  {
    sensor.takeoff_altitude = fmin(300, current_target() + 0.2);
    run(1);
  }
  run(50);
  assert(FlightControl_IsTakeoffComplete());
  close_to(current_target(), 300);

  setup(1);
  lift();
  const uint32_t deadline = CoreHelpers_GetTakeoffTargetTimeoutMs(&Config);
  now_ms = deadline - 10;
  assert(step() == HAL_OK);
  finish_landing(APP_ERROR_TAKEOFF_TARGET_TIMEOUT);
}

static void test_initial_echo_loss_has_four_second_grace(void)
{
  for (unsigned initial_echo = 0; initial_echo < 2; initial_echo++)
  {
    setup(initial_echo);
    Config.armThrottle = 20;
    Config.hoverThrottle = Config.maxThrottle = 80;
    Config.takeoffThrottleRampPerSecond = 30;
    sensor.takeoff_altitude = 8.1;
    FlightControl_BeginTakeoff();
    assert(CoreHelpers_GetTakeoffLiftoffTimeoutMs(&Config) == 4000);
    run(20);
    sensor.takeoff_altitude_valid = 0;
    sensor.takeoff_altitude = 99; /* Invalid cached data must not confirm lift. */
    run(379);
    assert(now_ms == 3990 && !FlightControl_IsLanding());
    assert(liftoff_messages == 0 && max_output() > 0);
    assert(step() == HAL_OK);
    assert(FlightControl_IsLanding());
    finish_landing(APP_ERROR_TAKEOFF_LIFTOFF_TIMEOUT);
  }

  setup(0);
  Config.takeoffThrottleRampPerSecond = 30;
  run(350); /* First usable echo can arrive late without an early abort. */
  sensor.takeoff_altitude_valid = 1;
  sensor.takeoff_altitude = 8;
  run(10);
  sensor.takeoff_altitude = 13;
  run(15);
  assert(liftoff_messages == 1 && !FlightControl_IsLanding());

  setup(1);
  sensor.takeoff_altitude = 15;
  run(10); /* Partly accumulated confirmation. */
  sensor.takeoff_altitude_valid = 0;
  const double before = log_data.base_throttle;
  run(10);
  assert(liftoff_messages == 0 && !FlightControl_IsLanding());
  assert(log_data.base_throttle > before); /* No stalled ramp on invalid height. */
}

static void test_sensor_loss_and_restart(void)
{
  for (unsigned phase = 0; phase < 2; phase++)
  {
    setup(1);
    lift();
    if (phase == 1) droneState.state = DRONE_STATE_FLYING;
    const double collective = output.front_left;
    const uint32_t lost_at = now_ms + 10;
    sensor.takeoff_altitude_valid = 0;
    sensor.Vz_valid = 0;
    sensor.Vz = NAN;
    sensor.takeoff_altitude = NAN;
    assert(step() == HAL_OK);
    assert(FlightControl_IsLanding() && landing_messages == 1);
    close_to(output.front_left, collective); /* No throttle discontinuity. */
    run(100);
    close_to(log_data.base_throttle, collective - 10);
    assert(now_ms == lost_at + 1000);
    // Recovery must not raise throttle or restart altitude control.
    sensor.takeoff_altitude_valid = 1;
    sensor.takeoff_altitude = 1;
    run(1);
    assert(FlightControl_IsLanding() && !log_data.altitude_control_active);
    assert(landing_messages == 1);
    finish_landing(APP_ERROR_ALTITUDE_UNAVAILABLE);
  }

  setup(1);
  lift();
  sensor.takeoff_altitude = NAN; /* Invalid numeric input with valid flag set. */
  assert(step() == HAL_OK && FlightControl_IsLanding());
  finish_landing(APP_ERROR_ALTITUDE_UNAVAILABLE);

  setup(1);
  lift();
  sensor.takeoff_altitude_valid = 0;
  assert(step() == HAL_OK && FlightControl_IsLanding());
  FlightControl_Stop(); /* Explicit DISARM remains immediate during landing. */
  stopped();
  close_to(pid_altitude.integral, 0);
  droneState.state = DRONE_STATE_ARMED;
  sensor.takeoff_altitude_valid = 1;
  sensor.takeoff_altitude = 10;
  FlightControl_BeginTakeoff();
  droneState.state = DRONE_STATE_TAKEOFF;
  run(1);
  close_to(log_data.base_throttle, 10.1);
  close_to(pid_altitude.p_term, 0);
  assert(!FlightControl_IsTakeoffComplete());
}

static void test_landing_uses_applied_collective_and_keeps_stabilization(void)
{
  setup(1); lift();
  droneState.state = DRONE_STATE_FLYING;
  sensor.roll_deg = 2;
  run(100); /* Build a real integral before the sensor loss. */
  const double held_integral = pid_roll.integral;
  const MotorSpeeds_t before = output;
  assert(fabs(held_integral) > 1);
  sensor.takeoff_altitude_valid = 0;
  run(1);
  close_to(pid_roll.integral, held_integral);
  close_to(output.front_left, before.front_left);
  close_to(output.front_right, before.front_right);
  run(50);
  close_to(pid_roll.integral, held_integral); /* No landing windup. */
  FlightControl_Stop(); stopped();

  setup(1);
  lift();
  droneState.state = DRONE_STATE_FLYING;
  sensor.takeoff_altitude = 50; /* Nonzero altitude PID contribution. */
  run(1);
  const double actual = 0.25 * (output.front_left + output.front_right +
                                output.back_left + output.back_right);
  assert(fabs(actual - Config.hoverThrottle) > 1);
  sensor.takeoff_altitude_valid = 0;
  assert(step() == HAL_OK && FlightControl_IsLanding());
  close_to(log_data.base_throttle, actual);
  sensor.roll_deg = 2;
  sensor.roll_rate_dps = 10;
  PID_Init(&pid_roll, 0.4, 0, 0.02);
  run(1);
  close_to(pid_roll.p_term, -0.8);
  close_to(pid_roll.d_term, -0.2);
  assert(output.front_left < output.front_right);
  FlightControl_Stop(); stopped();

  for (unsigned fault = 0; fault < 3; fault++)
  {
    setup(1); lift();
    sensor.takeoff_altitude_valid = 0;
    run(1);
    if (fault == 0) sensor.roll_deg = Config.maxAngle + 1;
    if (fault == 1) sensor.roll_rate_dps = NAN;
    HAL_StatusTypeDef status = fault == 2 ? FlightControl_Update(0.251) : step();
    assert(status == HAL_ERROR);
    assert(AppError_GetCode() == (fault == 0 ? APP_ERROR_MAXIMUM_FLIGHT_ANGLE :
                                 fault == 1 ? APP_ERROR_MPU6050_READ :
                                              APP_ERROR_INVALID_FLIGHT_TIMESTEP));
    stopped();
  }
}

static void test_tick_wrap_and_back_to_back_arm_takeoff(void)
{
  setup(0);
  now_ms = UINT32_MAX - 100U;
  FlightControl_BeginTakeoff();
  run(1);
  close_to(log_data.base_throttle, 10.1);
  now_ms += CoreHelpers_GetTakeoffLiftoffTimeoutMs(&Config) - 10U;
  assert(FlightControl_Update(0.01) == HAL_OK);
  finish_landing(APP_ERROR_TAKEOFF_LIFTOFF_TIMEOUT);

  setup(1); lift();
  now_ms = UINT32_MAX - 100;
  sensor.takeoff_altitude_valid = 0;
  run(1);
  finish_landing(APP_ERROR_ALTITUDE_UNAVAILABLE); /* Landing crosses tick wrap. */
}

static void test_recorded_attitude_rates(void)
{
  setup(1);
  sensor.roll_rate_dps = 12.5;
  sensor.pitch_rate_dps = -34.25;
  sensor.attitude_trace.valid = 1;
  sensor.attitude_trace.sequence = 123;
  sensor.attitude_trace.roll.correction_deg = 5.5f;
  sensor.srf05_trace = (Srf05Trace_t){.last_echo_ms = 1, .last_accepted_ms = 1,
      .valid = 1, .flags = 3, .result = SRF05_TRACE_ACCEPTED};
  assert(step() == HAL_OK);
  close_to(log_data.roll_rate_dps, 12.5);
  close_to(log_data.pitch_rate_dps, -34.25);
  assert(log_data.attitude_trace.valid && log_data.attitude_trace.sequence == 123);
  close_to(log_data.attitude_trace.roll.correction_deg, 5.5);
  assert(log_data.srf05_trace.last_accepted_ms == 1 && log_data.srf05_trace.flags == 3);
}

static void test_gyro_damping(void)
{
  setup(0);
  PID_Init(&pid_roll, 0.4, 0, 0.02);
  PID_Init(&pid_pitch, 0.4, 0, 0.02);
  /* Reproduce the logged disagreement: fused angle recovering while the
   * gyro still measures negative rotation. D must oppose the gyro. */
  sensor.roll_deg = -8.775588;
  sensor.roll_rate_dps = -14.626107;
  sensor.pitch_rate_dps = 25;
  assert(step() == HAL_OK); /* D applies even on the first update. */
  close_to(pid_roll.d_term, 0.29252214);
  close_to(pid_pitch.d_term, -0.5);
  close_to(pid_roll.p_term, 3.5102352);
  /* A target step and a large fusion correction cannot create a D kick. */
  Config.roll.target = 5;
  sensor.roll_deg = 12;
  sensor.pitch_deg = -10;
  assert(step() == HAL_OK);
  close_to(pid_roll.d_term, 0.29252214);
  close_to(pid_pitch.d_term, -0.5);
  now_ms += 15;
  assert(FlightControl_Update(0.015) == HAL_OK);
  close_to(pid_roll.d_term, 0.29252214); /* Rates already have deg/s units. */
  sensor.roll_rate_dps = sensor.pitch_rate_dps = 0;
  assert(step() == HAL_OK);
  close_to(pid_roll.d_term, 0);
  close_to(pid_pitch.d_term, 0);

  for (unsigned axis = 0; axis < 2; axis++)
  {
    setup(0);
    if (axis == 0) sensor.roll_rate_dps = NAN;
    else sensor.pitch_rate_dps = INFINITY;
    assert(step() == HAL_ERROR);
    assert(AppError_GetCode() == APP_ERROR_MPU6050_READ);
    stopped();
  }
}

int main(void)
{
  test_bounded_ramp_and_no_liftoff_timeout();
  test_first_reading_and_transient_rise();
  test_climb_engagement_and_derivative();
  test_settling_and_continuous_hover();
  test_full_height_budget_and_target_timeout();
  test_initial_echo_loss_has_four_second_grace();
  test_sensor_loss_and_restart();
  test_landing_uses_applied_collective_and_keeps_stabilization();
  test_tick_wrap_and_back_to_back_arm_takeoff();
  test_recorded_attitude_rates();
  test_gyro_damping();
  puts("PASS: 11 takeoff/landing regression groups");
  return 0;
}
