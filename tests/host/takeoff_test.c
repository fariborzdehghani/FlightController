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
  droneState.state = DRONE_STATE_ARMED;
  PID_Init(&pid_roll, 1, 0.1, 0);
  PID_Init(&pid_pitch, 1, 0.1, 0);
  PID_Init(&pid_Gz, 1, 0.1, 0);
  PID_Init(&pid_altitude, 0.5, 0.1, 0.2);
  assert(FlightControl_Start() == HAL_OK);
  FlightControl_BeginTakeoff();
  droneState.state = DRONE_STATE_TAKEOFF;
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
    assert(step() == HAL_ERROR);
    assert(AppError_GetCode() == APP_ERROR_TAKEOFF_LIFTOFF_TIMEOUT);
    stopped();
  }
  setup(0);
  Config.takeoffThrottleRampPerSecond = 1;
  assert(CoreHelpers_GetTakeoffLiftoffTimeoutMs(&Config) == 10000);
  now_ms = 9990;
  assert(step() == HAL_ERROR);
  stopped();
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
  assert(step() == HAL_ERROR);
  assert(AppError_GetCode() == APP_ERROR_TAKEOFF_TARGET_TIMEOUT);
  stopped();
}

static void test_sensor_loss_and_restart(void)
{
  for (unsigned phase = 0; phase < 3; phase++)
  {
    setup(1);
    if (phase > 0) lift();
    if (phase == 2) droneState.state = DRONE_STATE_FLYING;
    sensor.takeoff_altitude_valid = 0;
    assert(step() == HAL_ERROR);
    assert(AppError_GetCode() == APP_ERROR_ALTITUDE_UNAVAILABLE);
    stopped();
  }
  setup(1);
  lift();
  sensor.takeoff_altitude = NAN;
  assert(step() == HAL_ERROR);
  stopped();

  setup(1);
  lift();
  FlightControl_Stop(); /* The DISARM and heartbeat-loss paths use this function. */
  stopped();
  close_to(pid_altitude.integral, 0);
  droneState.state = DRONE_STATE_ARMED;
  sensor.takeoff_altitude = 10;
  FlightControl_BeginTakeoff();
  droneState.state = DRONE_STATE_TAKEOFF;
  run(1);
  close_to(log_data.base_throttle, 10.1);
  close_to(pid_altitude.p_term, 0);
  assert(!FlightControl_IsTakeoffComplete());
}

static void test_tick_wrap_and_back_to_back_arm_takeoff(void)
{
  setup(0);
  now_ms = UINT32_MAX - 100U;
  FlightControl_BeginTakeoff();
  run(1);
  close_to(log_data.base_throttle, 10.1);
  now_ms += CoreHelpers_GetTakeoffLiftoffTimeoutMs(&Config) - 10U;
  assert(FlightControl_Update(0.01) == HAL_ERROR);
  stopped();
}

static void test_recorded_attitude_rates(void)
{
  setup(1);
  sensor.roll_rate_dps = 12.5;
  sensor.pitch_rate_dps = -34.25;
  assert(step() == HAL_OK);
  close_to(log_data.roll_rate_dps, 12.5);
  close_to(log_data.pitch_rate_dps, -34.25);
}

int main(void)
{
  test_bounded_ramp_and_no_liftoff_timeout();
  test_first_reading_and_transient_rise();
  test_climb_engagement_and_derivative();
  test_settling_and_continuous_hover();
  test_full_height_budget_and_target_timeout();
  test_sensor_loss_and_restart();
  test_tick_wrap_and_back_to_back_arm_takeoff();
  test_recorded_attitude_rates();
  puts("PASS: 8 takeoff regression groups");
  return 0;
}
