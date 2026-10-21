#include "FlightRecorder.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t request[64], response[64];
static uint8_t blob[128u + FLIGHT_RECORDER_CAPACITY * FLIGHT_RECORDER_RECORD_SIZE];
static uint32_t clock_ms;

static void make_request(uint8_t operation, uint32_t id, uint32_t sequence)
{
  memset(request, 0, sizeof(request));
  request[0] = PACKET_TYPE_LOG_REQUEST;
  request[1] = operation;
  request[2] = FLIGHT_LOG_VERSION;
  if (operation == FLIGHT_LOG_BEGIN) request[3] = FLIGHT_LOG_PROFILE_FULL;
  FlightLog_Write32(request + 4, id);
  FlightLog_Write32(request + 8, sequence);
  FlightLog_Seal(request);
}

static uint8_t exchange(uint8_t operation, uint32_t id, uint32_t sequence, bool armed)
{
  make_request(operation, id, sequence);
  assert(FlightRecorder_Respond(request, response, clock_ms, armed));
  assert(FlightLog_IsResponseValid(response));
  assert(FlightLog_Read32(response + 4) == id);
  return response[1];
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

static uint32_t download(uint32_t id)
{
  assert(exchange(FLIGHT_LOG_BEGIN, id, 0, false) == FLIGHT_LOG_OK);
  assert(FlightLog_Read32(response + 8) == FLIGHT_LOG_BEGIN_SEQUENCE);
  uint32_t size = FlightLog_Read32(response + 12);
  uint32_t crc = FlightLog_Read32(response + 16);
  assert(size <= sizeof(blob));
  for (uint32_t offset = 0, seq = 0; offset < size; seq++)
  {
    assert(exchange(FLIGHT_LOG_READ, id, seq, false) == FLIGHT_LOG_OK);
    assert(FlightLog_Read32(response + 8) == seq);
    assert(response[3] > 0 && response[3] <= 48);
    assert(offset + response[3] <= size);
    memcpy(blob + offset, response + 12, response[3]);
    offset += response[3];
    clock_ms += 100;
  }
  assert(crc32(blob, size) == crc);
  return size;
}

int main(int argc, char **argv)
{
  assert(FlightLog_Crc16((const uint8_t *)"123456789", 9) == 0x29b1);
  assert(crc32((const uint8_t *)"123456789", 9) == 0xcbf43926);
  assert(exchange(FLIGHT_LOG_BEGIN, 10, 0, false) == FLIGHT_LOG_EMPTY);
  make_request(FLIGHT_LOG_READ, 1, 0);
  request[18] = 1; FlightLog_Seal(request);
  assert(!FlightLog_IsRequestValid(request));
  assert(!FlightRecorder_Respond(request, response, clock_ms, false));
  make_request(FLIGHT_LOG_BEGIN, 1, 0); request[8] = 1; FlightLog_Seal(request);
  assert(!FlightLog_IsRequestValid(request));
  make_request(FLIGHT_LOG_READ, 1, 0); request[62] ^= 1;
  assert(!FlightLog_IsRequestValid(request));

  uint8_t config[64] = {1, 10, 10, 80, 30};
  config[13] = 50; config[41] = 100; config[44] = 81; config[45] = 5;
  clock_ms = 1000;
  FlightRecorder_Begin(config, clock_ms);
  config[1] = 99; /* A later configuration cannot rewrite this session. */
  FlightRecorderRecord_t sample = {.timestamp_ms = 1000, .state = 3, .flags = 31};
  for (unsigned j = 0; j < 29; j++) sample.values[j] = (float)j + 0.25f;
  FlightRecorder_Record(&sample, false);
  sample.timestamp_ms = 1001; FlightRecorder_Record(&sample, false); /* skipped */
  assert(exchange(FLIGHT_LOG_BEGIN, 10, 0, false) == FLIGHT_LOG_BUSY);
  sample.timestamp_ms = 1000 + FLIGHT_RECORDER_INTERVAL_MS; sample.state = 4; sample.values[0] = 3.5f;
  FlightRecorder_Record(&sample, false);
  sample.timestamp_ms = 1000 + 2 * FLIGHT_RECORDER_INTERVAL_MS; sample.state = 5; sample.error = 7;
  sample.values[6] = sample.values[7] = sample.values[8] = sample.values[9] = sample.values[10] = 0;
  FlightRecorder_Record(&sample, true);
  FlightRecorder_End(FLIGHT_LOG_STOP_FAULT, 7, 1, sample.timestamp_ms);
  assert(!FlightRecorder_IsRecording());
  clock_ms = 1200;
  assert(exchange(FLIGHT_LOG_BEGIN, 10, 0, true) == FLIGHT_LOG_BUSY);
  uint32_t size = download(10);
  assert(size == 128 + 3 * FLIGHT_RECORDER_RECORD_SIZE && blob[12] == 3);
  assert(blob[4] == 3 && blob[16] == FLIGHT_RECORDER_INTERVAL_MS);
  assert(blob[18] == FLIGHT_LOG_STOP_FAULT && blob[20] == 7 && blob[19] == 1);
  assert(blob[65] == 10 && blob[108] == 81 && blob[109] == 5);
  assert(FlightLog_Read32(blob + 128) == 1000);
  assert(FlightLog_Read32(blob + 128 + FLIGHT_RECORDER_RECORD_SIZE) == 1010);
  assert(blob[128 + 2 * FLIGHT_RECORDER_RECORD_SIZE + 4] == 5);
  float roll_rate, pitch_rate;
  memcpy(&roll_rate, blob + 128 + 8 + 27 * 4, 4);
  memcpy(&pitch_rate, blob + 128 + 8 + 28 * 4, 4);
  assert(roll_rate == 27.25f && pitch_rate == 28.25f);
  if (argc > 1)
  {
    FILE *file = fopen(argv[1], "wb"); assert(file);
    assert(fwrite(blob, 1, size, file) == size); assert(fclose(file) == 0);
  }
  assert(exchange(FLIGHT_LOG_READ, 99, 0, false) == FLIGHT_LOG_SESSION_MISMATCH);
  assert(exchange(FLIGHT_LOG_READ, 10, UINT32_MAX, false) == FLIGHT_LOG_RANGE_ERROR);
  assert(exchange(FLIGHT_LOG_END, 10, 0, false) == FLIGHT_LOG_OK);
  assert(exchange(FLIGHT_LOG_END, 10, 0, false) == FLIGHT_LOG_OK);
  assert(!FlightRecorder_IsTransferring(clock_ms));

  clock_ms = 2000;
  FlightRecorder_Begin(config, clock_ms);
  for (unsigned i = 0; i < FLIGHT_RECORDER_CAPACITY + 44u; i++)
  {
    sample.timestamp_ms = 2000 + i * FLIGHT_RECORDER_INTERVAL_MS; sample.values[0] = (float)i;
    FlightRecorder_Record(&sample, false);
  }
  FlightRecorder_End(FLIGHT_LOG_STOP_OPERATOR, 0, 0, sample.timestamp_ms);
  size = download(20);
  assert(size == sizeof(blob) && blob[6] == 1);
  assert(FlightLog_Read32(blob + 32) == 44);
  assert(FlightLog_Read32(blob + 128) == 2440); /* oldest retained sample */
  uint32_t original_crc = crc32(blob, size);
  sample.timestamp_ms = 20000; FlightRecorder_Record(&sample, true);
  assert(download(20) == size && crc32(blob, size) == original_crc); /* frozen */
  clock_ms += FLIGHT_RECORDER_TRANSFER_TIMEOUT_MS;
  assert(!FlightRecorder_IsTransferring(clock_ms));
  assert(exchange(FLIGHT_LOG_READ, 20, 0, false) == FLIGHT_LOG_SESSION_MISMATCH);

  /* Tick rollover is handled by unsigned interval arithmetic. */
  clock_ms = UINT32_MAX - FLIGHT_RECORDER_INTERVAL_MS / 2u;
  FlightRecorder_Begin(config, clock_ms);
  sample.timestamp_ms = clock_ms; FlightRecorder_Record(&sample, false);
  sample.timestamp_ms += FLIGHT_RECORDER_INTERVAL_MS; FlightRecorder_Record(&sample, false);
  FlightRecorder_End(FLIGHT_LOG_STOP_LINK_TIMEOUT, 0, 0, sample.timestamp_ms);
  assert(download(30) == 128 + 2 * FLIGHT_RECORDER_RECORD_SIZE && blob[12] == 2);
  puts("flight recorder tests passed");
  return 0;
}
