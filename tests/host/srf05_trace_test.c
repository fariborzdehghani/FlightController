#include "SensorAcquisition.h"
#include "MPU6050.h"
#include "VelocityEstimator.h"
#include "Logger.h"
#include "FlightRecorder.h"
#include "Tools.h"
#include "srf05.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

DroneConfig_t Config;
DroneStateManager_t droneState;
FlightData_t flightData;
MotorSpeeds_t Motors_Speed;
static uint32_t now, echo_time;
static float echo_cm;
static HAL_StatusTypeDef read_status = HAL_BUSY, start_status = HAL_OK;

uint32_t HAL_GetTick(void) { return now; }
void HAL_Delay(uint32_t ms) { now += ms; }
uint8_t MPU6050_IsReady(void) { return 1; }
HAL_StatusTypeDef MPU6050_Calibrate(void) { return HAL_OK; }
HAL_StatusTypeDef MPU6050_ReadAll(MPU6050_t *data)
{
  memset(data, 0, sizeof(*data));
  data->Az_ms = 9.80665;
  return HAL_OK;
}
void SRF05_CancelMeasurement(void) { }
HAL_StatusTypeDef SRF05_StartMeasurement(void) { return start_status; }
HAL_StatusTypeDef SRF05_ReadMeasurement(float *cm, uint32_t *timestamp)
{
  if (read_status == HAL_OK) { *cm = echo_cm; *timestamp = echo_time; }
  const HAL_StatusTypeDef result = read_status;
  read_status = HAL_BUSY;
  return result;
}
bool LogError(int code, const char *message) { (void)code; (void)message; return true; }
bool LogInformation(int code, const char *message) { (void)code; (void)message; return true; }
bool LogStringToPC(const char *text) { (void)text; return true; }

static FlightData_t update(uint32_t tick, HAL_StatusTypeDef status,
                            float cm, uint32_t measured)
{
  now = tick; read_status = status; echo_cm = cm; echo_time = measured;
  assert(SensorAcquisition_Update(0.015).hal_status == HAL_OK);
  flightData = SensorAcquisition_GetFlightData();
  return flightData;
}

static void setup(uint32_t tick)
{
  now = tick; start_status = HAL_OK; read_status = HAL_BUSY;
  Config = (DroneConfig_t){0};
  memset(&Motors_Speed, 0, sizeof(Motors_Speed));
  droneState.state = DRONE_STATE_TAKEOFF;
  SensorAcquisition_Init();
  assert(SensorAcquisition_Start().hal_status == HAL_OK);
  (void)update(now, HAL_BUSY, 0, 0); /* Start the first echo request. */
  const uint32_t base = now;
  update(base + 20, HAL_OK, 100, base + 17);
  update(base + 60, HAL_BUSY, 0, 0);
  update(base + 70, HAL_OK, 100, base + 67);
  update(base + 120, HAL_BUSY, 0, 0); /* Third echo will establish the median. */
}

static uint8_t response[64];
static void exchange(uint8_t operation, uint32_t sequence)
{
  uint8_t request[64] = {PACKET_TYPE_LOG_REQUEST, operation, FLIGHT_LOG_VERSION};
  request[3] = operation == FLIGHT_LOG_BEGIN ? FLIGHT_LOG_PROFILE_FULL : 0;
  FlightLog_Write32(request + 4, 123);
  FlightLog_Write32(request + 8, sequence);
  FlightLog_Seal(request);
  assert(FlightRecorder_Respond(request, response, now, false));
  assert(response[1] == FLIGHT_LOG_OK);
}

int main(void)
{
  setup(1000);
  const uint32_t base = now;
  assert(flightData.srf05_trace.valid && flightData.srf05_trace.flags == SRF05_TRACE_HAVE_ECHO);
  assert(flightData.srf05_trace.filter_result == VELOCITY_ESTIMATOR_SRF05_MEDIAN_WARMUP);
  const uint32_t accepted = base + 17;
  update(base + 20, HAL_OK, 100, accepted);
  assert(flightData.takeoff_altitude_valid);
  assert(flightData.srf05_trace.raw_distance_cm == 100);
  assert(flightData.srf05_trace.filter_result == VELOCITY_ESTIMATOR_SRF05_ACCEPTED);
  assert(flightData.srf05_trace.last_echo_ms == accepted);
  assert(flightData.srf05_trace.last_accepted_ms == accepted);
  assert((uint32_t)(now - flightData.srf05_trace.last_accepted_ms) == 3);
  update(base + 60, HAL_BUSY, 0, 0);
  update(base + 80, HAL_OK, 300, base + 76); /* Raw echo, rejected by filter. */
  assert(flightData.srf05_trace.last_echo_ms == base + 76);
  assert(flightData.srf05_trace.last_accepted_ms == accepted);
  assert((uint32_t)(now - flightData.srf05_trace.last_accepted_ms) == 63);
  assert(flightData.srf05_trace.result == SRF05_TRACE_REJECTED);
  assert(flightData.srf05_trace.raw_distance_cm == 300);
  assert(flightData.srf05_trace.filter_result == VELOCITY_ESTIMATOR_SRF05_MOTION_REJECTED);
  update(base + 120, HAL_BUSY, 0, 0);
  update(base + 130, HAL_TIMEOUT, 0, 0);
  assert(flightData.takeoff_altitude_valid); /* One missed response is tolerated. */
  assert(flightData.srf05_trace.result == SRF05_TRACE_TIMEOUT);
  assert(flightData.srf05_trace.raw_distance_cm == 300);
  assert(flightData.srf05_trace.filter_result == VELOCITY_ESTIMATOR_SRF05_MOTION_REJECTED);
  Logger_BeginFlight();
  update(accepted + 300, HAL_BUSY, 0, 0);
  assert(flightData.takeoff_altitude_valid); /* Exact grace boundary. */
  FlightLogData_t log = {.srf05_trace = flightData.srf05_trace, .landing_active = 1};
  Logger_LogFlightData(&log);
  update(accepted + 301, HAL_BUSY, 0, 0);
  assert(!flightData.takeoff_altitude_valid);
  assert((uint32_t)(now - flightData.srf05_trace.last_accepted_ms) == 301);
  Logger_EndFlight(FLIGHT_LOG_STOP_FAULT, 9, HAL_ERROR, DRONE_STATE_FAULT);
  exchange(FLIGHT_LOG_BEGIN, 0);
  uint32_t size = FlightLog_Read32(response + 12);
  uint8_t blob[128 + 3 * FLIGHT_RECORDER_RECORD_SIZE];
  assert(size == sizeof(blob));
  for (uint32_t offset = 0, sequence = 0; offset < size; sequence++)
  {
    exchange(FLIGHT_LOG_READ, sequence);
    memcpy(blob + offset, response + 12, response[3]); offset += response[3];
  }
  const uint8_t *final = blob + 128 + 2 * FLIGHT_RECORDER_RECORD_SIZE;
  assert(final[4] == DRONE_STATE_FAULT);
  assert((blob[128 + FLIGHT_RECORDER_RECORD_SIZE + 5] & 32u) != 0);
  assert((final[5] & 32u) == 0); /* Final event represents stopped motors. */
  assert(FlightLog_Read32(final + 196) == accepted);
  float raw_cm; memcpy(&raw_cm, final + 200, sizeof(raw_cm));
  assert(raw_cm == 300);
  assert(final[207] == VELOCITY_ESTIMATOR_SRF05_MOTION_REJECTED);
  assert(final[204] == 1 && final[205] == 3);
  exchange(FLIGHT_LOG_END, 0);
  update(accepted + 350, HAL_OK, 100, accepted + 348);
  assert(flightData.srf05_trace.result == SRF05_TRACE_ACCEPTED);
  assert((uint32_t)(now - flightData.srf05_trace.last_accepted_ms) == 2);
  SensorAcquisition_Stop();
  assert(SensorAcquisition_GetFlightData().srf05_trace.valid == 0);

  setup(UINT32_MAX - 600);
  update(UINT32_MAX - 50, HAL_OK, 100, UINT32_MAX - 55);
  update(50, HAL_BUSY, 0, 0);
  assert((uint32_t)(now - flightData.srf05_trace.last_accepted_ms) == 106);
  assert(flightData.takeoff_altitude_valid);
  setup(UINT32_MAX - 600);
  update(0, HAL_OK, 100, 0);
  assert(flightData.srf05_trace.last_accepted_ms == 0);
  assert(flightData.srf05_trace.flags == 3); /* Tick zero is a real measurement. */
  update(20, HAL_BUSY, 0, 0);
  assert((uint32_t)(now - flightData.srf05_trace.last_accepted_ms) == 20);
  puts("PASS: SRF05 freshness through acquisition, timeout, logger and recorder");
}
