#include "FlightRecorder.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t request[64], response[64];
static uint8_t blob[128 + FLIGHT_RECORDER_CAPACITY * FLIGHT_RECORDER_RECORD_SIZE +
                    FLIGHT_RECORDER_EVENT_CAPACITY * FLIGHT_RECORDER_EVENT_SIZE];
static uint32_t now = 1000;

static uint8_t exchange(uint8_t operation, uint32_t id, uint32_t sequence, bool armed)
{
  memset(request, 0, sizeof(request));
  request[0] = PACKET_TYPE_LOG_REQUEST;
  request[1] = operation;
  request[2] = FLIGHT_LOG_VERSION;
  if (operation == FLIGHT_LOG_BEGIN) request[3] = FLIGHT_LOG_PROFILE_FULL;
  FlightLog_Write32(request + 4, id);
  FlightLog_Write32(request + 8, sequence);
  FlightLog_Seal(request);
  assert(FlightRecorder_Respond(request, response, now, armed));
  assert(FlightLog_IsResponseValid(response));
  return response[1];
}

static uint32_t download(uint32_t id)
{
  assert(exchange(FLIGHT_LOG_BEGIN, id, 0, false) == FLIGHT_LOG_OK);
  uint32_t size = FlightLog_Read32(response + 12);
  assert(size <= sizeof(blob));
  for (uint32_t offset = 0, seq = 0; offset < size; seq++)
  {
    assert(exchange(FLIGHT_LOG_READ, id, seq, false) == FLIGHT_LOG_OK);
    memcpy(blob + offset, response + 12, response[3]);
    offset += response[3];
  }
  return size;
}

int main(int argc, char **argv)
{
  uint8_t config[64] = {0}; /* Download works even before SET CONFIG. */
  FlightRecorderRecord_t sample = {.timestamp_ms = 1000, .state = 0, .flags = 2};
  sample.values[0] = -1.25f;
  sample.values[5] = 12.0f;
  sample.values[25] = 3.0f;
  FlightRecorder_SetLoopTiming(1.0f);
  FlightRecorder_SetLoopTiming(3.0f);
  for (unsigned i = 0; i < 20; i++)
    FlightRecorder_LogMessage(i * 10, 1001, false, "Startup diagnostic");
  const char *rejection = "TAKEOFF rejected: tilt; roll=-18.25 pitch=2.5 limit=15 deg";
  FlightRecorder_LogMessage(999, 2002, true, rejection);
  FlightRecorder_Snapshot(config, &sample);
  assert(!FlightRecorder_HasFlight());
  uint32_t size = download(1);
  assert(size == 128 + FLIGHT_RECORDER_RECORD_SIZE + 16 * 104);
  assert(blob[4] == FLIGHT_RECORDER_FORMAT_VERSION && blob[18] == FLIGHT_LOG_STOP_SNAPSHOT && blob[64] == 0);
  assert(blob[48] == 16 && blob[50] == 104);
  float average;
  memcpy(&average, blob + 40, 4);
  assert(average == 2.0f && FlightLog_Read32(blob + 44) == 2);
  unsigned event_offset = 128 + FLIGHT_RECORDER_RECORD_SIZE;
  assert(FlightLog_Read32(blob + event_offset) == 50); /* oldest retained diagnostic */
  assert(blob[event_offset + 15 * 104 + 6] == 1);
  assert(blob[event_offset + 15 * 104 + 7] == strlen(rejection));
  assert(memcmp(blob + event_offset + 15 * 104 + 8, rejection, strlen(rejection)) == 0);
  if (argc > 1)
  {
    FILE *file = fopen(argv[1], "wb"); assert(file);
    assert(fwrite(blob, 1, size, file) == size);
    assert(fclose(file) == 0);
  }

  uint8_t frozen[sizeof(blob)];
  memcpy(frozen, blob, size);
  FlightRecorder_SetLoopTiming(8.0f);
  FlightRecorder_LogMessage(now, 1001, false, "Ignored while frozen");
  sample.timestamp_ms = 1100;
  FlightRecorder_Snapshot(config, &sample);
  assert(download(1) == size && memcmp(blob, frozen, size) == 0);
  for (uint8_t operation = 1; operation <= 3; operation++)
    assert(exchange(operation, 1, 0, true) == FLIGHT_LOG_BUSY);
  assert(exchange(FLIGHT_LOG_END, 1, 0, false) == FLIGHT_LOG_OK);

  /* A startup failure has no applied configuration or real flight. Its
   * diagnostic snapshot must retain the fault state and application code. */
  now = 1150;
  sample.timestamp_ms = now;
  sample.state = 5;
  sample.error = 2; /* MPU6050 calibration. */
  FlightRecorder_LogMessage(now, 2003, true, "MPU6050 calibration timed out");
  FlightRecorder_Snapshot(config, &sample);
  download(99);
  assert(blob[18] == FLIGHT_LOG_STOP_SNAPSHOT && blob[20] == 2 && blob[64] == 0);
  assert(blob[128 + 4] == 5 && blob[128 + 6] == 2);
  assert(exchange(FLIGHT_LOG_END, 99, 0, false) == FLIGHT_LOG_OK);

  now = 1200;
  sample.timestamp_ms = now;
  sample.state = 1;
  sample.error = 0;
  config[0] = 1;
  FlightRecorder_Snapshot(config, &sample);
  download(2);
  memcpy(&average, blob + 40, 4);
  assert(average == 4.0f && FlightLog_Read32(blob + 44) == 3);
  assert(FlightLog_Read32(blob + 128) == 1200 && blob[64] == 1);
  assert(exchange(FLIGHT_LOG_END, 2, 0, false) == FLIGHT_LOG_OK);

  /* A later diagnostic request preserves a real flight's samples/config. */
  now = 1500;
  FlightRecorder_Begin(config, now);
  sample.timestamp_ms = now; sample.state = 3;
  FlightRecorder_Record(&sample, true);
  FlightRecorder_End(FLIGHT_LOG_STOP_OPERATOR, 0, 0, 1600);
  assert(FlightRecorder_HasFlight());
  sample.timestamp_ms = 1800;
  config[1] = 99;
  FlightRecorder_Snapshot(config, &sample);
  download(3);
  assert(blob[18] == FLIGHT_LOG_STOP_OPERATOR && blob[65] == 0);
  assert(FlightLog_Read32(blob + 128) == 1500);
  puts("disarmed diagnostics tests passed");
}
