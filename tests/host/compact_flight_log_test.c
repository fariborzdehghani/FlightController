#include "FlightRecorder.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint8_t reply[64], blob[128 + FLIGHT_RECORDER_CAPACITY * FLIGHT_RECORDER_RECORD_SIZE + 16 * 104];
static uint32_t now = 1100;

static void exchange(uint8_t op, uint32_t id, uint32_t seq, uint8_t profile)
{
  uint8_t request[64] = {PACKET_TYPE_LOG_REQUEST, op, FLIGHT_LOG_VERSION, profile};
  FlightLog_Write32(request + 4, id);
  FlightLog_Write32(request + 8, seq);
  FlightLog_Seal(request);
  assert(FlightRecorder_Respond(request, reply, now++, false));
  assert(FlightLog_IsResponseValid(reply));
  assert(reply[1] == FLIGHT_LOG_OK);
}

static uint32_t crc32(const uint8_t *data, uint32_t length)
{
  uint32_t crc = UINT32_MAX;
  for (uint32_t i = 0; i < length; i++)
  {
    crc ^= data[i];
    for (unsigned b = 0; b < 8; b++)
      crc = (crc >> 1) ^ ((crc & 1u) ? 0xedb88320u : 0u);
  }
  return ~crc;
}

static uint32_t download(uint32_t id, uint8_t profile)
{
  exchange(FLIGHT_LOG_BEGIN, id, 0, profile);
  uint32_t size = FlightLog_Read32(reply + 12), crc = FlightLog_Read32(reply + 16);
  assert(size <= sizeof(blob));
  /* A retry, even with another profile, cannot change the frozen transfer. */
  exchange(FLIGHT_LOG_BEGIN, id, 0, profile == 0 ? 1 : 0);
  assert(FlightLog_Read32(reply + 12) == size && FlightLog_Read32(reply + 16) == crc);
  for (uint32_t offset = 0, seq = 0; offset < size; seq++)
  {
    exchange(FLIGHT_LOG_READ, id, seq, 0);
    assert(reply[3] == (size - offset < 48 ? size - offset : 48));
    memcpy(blob + offset, reply + 12, reply[3]);
    offset += reply[3];
  }
  assert(crc32(blob, size) == crc);
  exchange(FLIGHT_LOG_END, id, 0, 0);
  return size;
}

static uint16_t u16(const uint8_t *p) { return p[0] | (p[1] << 8); }

int main(int argc, char **argv)
{
  uint8_t config[64] = {1, 10, 10, 80, 30};
  config[13] = 50; config[41] = 100; config[44] = 81; config[45] = 5;
  FlightRecorderRecord_t sample = {.timestamp_ms = 1000, .state = 3, .flags = 31};
  for (unsigned j = 0; j < 29; j++) sample.values[j] = j + 0.25f;
  sample.values[11] = 11.123456f;
  sample.values[13] = -0.00449f;
  FlightRecorder_Begin(config, 1000);
  FlightRecorder_SetLoopTiming(15);
  FlightRecorder_LogMessage(1000, 1001, false, "Flight started");
  for (unsigned i = 0; i < 3; i++)
  {
    sample.timestamp_ms = 1000 + i * 10;
    sample.state = 3 + i;
    sample.error = i == 2 ? 7 : 0;
    FlightRecorder_Record(&sample, false);
  }
  FlightRecorder_End(FLIGHT_LOG_STOP_FAULT, 7, 1, 1020);
  uint32_t size = download(1, FLIGHT_LOG_PROFILE_COMPACT);
  assert(size == 128 + 3 * 50 + 104 && blob[4] == 4 && u16(blob + 14) == 50);
  assert(blob[6] == 4 && blob[16] == 10 && blob[48] == 1);
  assert(FlightLog_Read32(blob + 128) == 1000);
  assert(FlightLog_Read32(blob + 128 + 100) == 1020 && blob[128 + 100 + 4] == 5);
  assert(u16(blob + 128 + 100 + 6) == 7);
  assert(memcmp(blob + 128 + 150 + 8, "Flight started", 14) == 0);
  const unsigned fields[] = {0, 1, 27, 28, 11, 12, 13, 14, 15, 16, 17, 18, 7, 8, 9, 10, 6, 5, 26, 4, 25};
  const float scales[] = {100, 100, 100, 100, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                          100, 100, 100, 100, 100, 10, 10, 1000, 100};
  for (unsigned j = 0; j < 21; j++)
  {
    float decoded = (int16_t)u16(blob + 128 + 8 + 2 * j) / scales[j];
    assert(fabsf(decoded - sample.values[fields[j]]) <= 0.5001f / scales[j]);
  }
  if (argc > 1)
  {
    FILE *file = fopen(argv[1], "wb"); assert(file);
    assert(fwrite(blob, 1, size, file) == size);
    assert(fclose(file) == 0);
  }
  /* The same onboard samples still offer all original floating-point values. */
  size = download(2, FLIGHT_LOG_PROFILE_FULL);
  assert(size == 128 + 3 * FLIGHT_RECORDER_RECORD_SIZE + 104 &&
         blob[4] == FLIGHT_RECORDER_FORMAT_VERSION && blob[6] == 4);
  assert(memcmp(blob + 128 + 8, sample.values, sizeof(sample.values)) == 0);

  /* Full ring, wraparound, diagnostics, flags and end-of-chunk boundaries. */
  FlightRecorder_Begin(config, 2000);
  for (unsigned i = 0; i < 556; i++)
  {
    sample.timestamp_ms = 2000 + i * 10;
    sample.values[0] = (float)i / 10;
    FlightRecorder_Record(&sample, false);
  }
  FlightRecorder_End(FLIGHT_LOG_STOP_OPERATOR, 0, 0, sample.timestamp_ms);
  size = download(3, FLIGHT_LOG_PROFILE_COMPACT);
  assert(size == 128 + 512 * 50 + 104 && blob[4] == 4 && blob[6] == 5);
  assert(FlightLog_Read32(blob + 32) == 44);
  assert(FlightLog_Read32(blob + 128) == 2440);
  assert(FlightLog_Read32(blob + 128 + 511 * 50) == 7550);
  assert((int16_t)u16(blob + 128 + 511 * 50 + 8) == 5550);

  /* Bad/out-of-range source values must survive, never saturate to look normal. */
  const float overflow[] = {33, -33, NAN, INFINITY, -INFINITY};
  for (unsigned i = 0; i < sizeof(overflow) / sizeof(overflow[0]); i++)
  {
    FlightRecorder_Begin(config, 9000);
    sample.timestamp_ms = 9000;
    sample.values[11] = overflow[i];
    FlightRecorder_Record(&sample, true);
    FlightRecorder_End(FLIGHT_LOG_STOP_FAULT, 1, 0, 9000);
    download(10 + i, FLIGHT_LOG_PROFILE_COMPACT);
    assert(blob[4] == FLIGHT_RECORDER_FORMAT_VERSION && blob[6] == 6 &&
           u16(blob + 14) == FLIGHT_RECORDER_RECORD_SIZE);
    assert(memcmp(blob + 128 + 8 + 11 * 4, &overflow[i], 4) == 0);
  }
  download(20, FLIGHT_LOG_PROFILE_FULL);
  assert(blob[4] == FLIGHT_RECORDER_FORMAT_VERSION && blob[6] == 4); /* Explicit full download resets fallback flag. */
  puts("compact flight log tests passed");
  return 0;
}
