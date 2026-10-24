#include "FlightRecorder.h"
#include "SensorConfig.h"
#include <string.h>
#include <math.h>

_Static_assert(sizeof(float) == 4u, "Log format requires 32-bit floats");
_Static_assert(sizeof(FlightRecorderRecord_t) == FLIGHT_RECORDER_RECORD_SIZE,
               "Log record wire layout changed");
/* Format 7 reuses the format-6 trace bytes; the sample RAM budget is unchanged. */
_Static_assert(FLIGHT_RECORDER_CAPACITY * FLIGHT_RECORDER_RECORD_SIZE <= 104u * 1024u,
               "Recorder exceeds the 104 KB sample budget");
#ifndef FLIGHT_LOG_ESTIMATOR_ID
#define FLIGHT_LOG_ESTIMATOR_ID 0u
#endif

static FlightRecorderRecord_t records[FLIGHT_RECORDER_CAPACITY];
static uint8_t header[FLIGHT_RECORDER_HEADER_SIZE];
static uint8_t configuration[64];
static uint16_t count, head;
static uint32_t session, overwritten, last_sample, transfer_id, last_request;
static bool recording, transferring;
static uint32_t blob_crc;
/* Capture full floats during flight. Only encode compact data after disarm. */
static uint16_t wire_record_size = FLIGHT_RECORDER_RECORD_SIZE;
static uint8_t compact_record[FLIGHT_RECORDER_COMPACT_RECORD_SIZE];
static uint16_t compact_index;
static bool compact_cached;
/* Format 4: signed little-endian int16 values after the unchanged 8-byte prefix.
 * Order/scales must match FlightLogSession.cs. No value is silently clipped. */
static const uint8_t compact_fields[] = {
  0, 1, 27, 28, 11, 12, 13, 14, 15, 16, 17, 18,
  7, 8, 9, 10, 6, 5, 26, 4, 25
};
static const float compact_scales[] = {
  100, 100, 100, 100, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
  100, 100, 100, 100, 100, 10, 10, 1000, 100
};
_Static_assert(sizeof(compact_fields) * 2u + 8u == FLIGHT_RECORDER_COMPACT_RECORD_SIZE,
               "Compact record layout changed");
/* Kept from boot, independently of flight samples. Frozen during downloads. */
static uint8_t events[FLIGHT_RECORDER_EVENT_CAPACITY][FLIGHT_RECORDER_EVENT_SIZE];
static uint16_t event_count, event_head;
static uint32_t loop_count;
static double average_loop_ms;
static bool has_flight;

static void write16(uint8_t *p, uint16_t value)
{
  p[0] = (uint8_t)value;
  p[1] = (uint8_t)(value >> 8);
}

static uint32_t blob_size(void)
{
  return FLIGHT_RECORDER_HEADER_SIZE + 64u + count * wire_record_size +
         event_count * FLIGHT_RECORDER_EVENT_SIZE;
}

static bool can_compact(void)
{
  uint32_t oldest = (head + FLIGHT_RECORDER_CAPACITY - count) % FLIGHT_RECORDER_CAPACITY;
  for (unsigned i = 0; i < count; i++)
  {
    const FlightRecorderRecord_t *record = &records[(oldest + i) % FLIGHT_RECORDER_CAPACITY];
    for (unsigned j = 0; j < sizeof(compact_fields); j++)
    {
      float value = record->values[compact_fields[j]] * compact_scales[j];
      if (!isfinite(value) || value < -32767.0f || value > 32767.0f) return false;
    }
  }
  return true;
}

static void select_encoding(uint8_t profile)
{
  bool compact = profile == FLIGHT_LOG_PROFILE_COMPACT && can_compact();
  wire_record_size = compact ? FLIGHT_RECORDER_COMPACT_RECORD_SIZE : FLIGHT_RECORDER_RECORD_SIZE;
  write16(header + 4, compact ? FLIGHT_RECORDER_COMPACT_FORMAT_VERSION : FLIGHT_RECORDER_FORMAT_VERSION);
  write16(header + 14, wire_record_size);
  /* Header flag bit 1 explains an automatic full-precision fallback. */
  header[6] = (uint8_t)((header[6] & ~2u) |
                       (!compact && profile == FLIGHT_LOG_PROFILE_COMPACT ? 2u : 0u));
  compact_cached = false;
}

static const uint8_t *encode_compact(uint16_t index)
{
  if (!compact_cached || compact_index != index)
  {
    const FlightRecorderRecord_t *record = &records[index];
    FlightLog_Write32(compact_record, record->timestamp_ms);
    compact_record[4] = record->state;
    compact_record[5] = record->flags;
    write16(compact_record + 6, record->error);
    for (unsigned j = 0; j < sizeof(compact_fields); j++)
    {
      int16_t value = (int16_t)lroundf(record->values[compact_fields[j]] * compact_scales[j]);
      write16(compact_record + 8u + j * 2u, (uint16_t)value);
    }
    compact_index = index;
    compact_cached = true;
  }
  return compact_record;
}

/* Read the frozen ring in chronological order without allocating another copy. */
static uint8_t blob_byte(uint32_t offset)
{
  if (offset < sizeof(header)) return header[offset];
  offset -= sizeof(header);
  if (offset < sizeof(configuration)) return configuration[offset];
  offset -= sizeof(configuration);
  if (offset >= count * wire_record_size)
  {
    offset -= count * wire_record_size;
    uint32_t oldest_event = (event_head + FLIGHT_RECORDER_EVENT_CAPACITY - event_count) %
                            FLIGHT_RECORDER_EVENT_CAPACITY;
    return events[(oldest_event + offset / FLIGHT_RECORDER_EVENT_SIZE) %
                  FLIGHT_RECORDER_EVENT_CAPACITY][offset % FLIGHT_RECORDER_EVENT_SIZE];
  }
  uint32_t index = offset / wire_record_size;
  uint32_t oldest = (head + FLIGHT_RECORDER_CAPACITY - count) % FLIGHT_RECORDER_CAPACITY;
  uint16_t ring_index = (uint16_t)((oldest + index) % FLIGHT_RECORDER_CAPACITY);
  const uint8_t *record = wire_record_size == FLIGHT_RECORDER_COMPACT_RECORD_SIZE ?
                          encode_compact(ring_index) : (const uint8_t *)&records[ring_index];
  return record[offset % wire_record_size];
}

static uint32_t calculate_crc(void)
{
  uint32_t crc = 0xffffffffu;
  for (uint32_t i = 0; i < blob_size(); i++)
  {
    crc ^= blob_byte(i);
    for (unsigned bit = 0; bit < 8; bit++)
      crc = (crc >> 1) ^ ((crc & 1u) ? 0xedb88320u : 0u);
  }
  return ~crc;
}

void FlightRecorder_Begin(const uint8_t config[64], uint32_t now)
{
  has_flight = true;
  FlightRecorder_CancelTransfer();
  wire_record_size = FLIGHT_RECORDER_RECORD_SIZE;
  compact_cached = false;
  memcpy(configuration, config, sizeof(configuration));
  memset(header, 0, sizeof(header));
  memcpy(header, "FLOG", 4u);
  write16(header + 4, FLIGHT_RECORDER_FORMAT_VERSION);
  if (++session == 0u) session = 1u;
  FlightLog_Write32(header + 8, session);
  write16(header + 14, FLIGHT_RECORDER_RECORD_SIZE);
  write16(header + 16, FLIGHT_RECORDER_INTERVAL_MS);
  FlightLog_Write32(header + 24, now);
  write16(header + 36, sizeof(configuration));
  write16(header + 38, sizeof(header));
  header[52] = ATTITUDE_TRACE_VERSION;
  header[53] = MPU6050_ACCEL_REJECTION_ENABLED;
  header[54] = MPU6050_DLPF_HZ;
  header[55] = MPU6050_ACCEL_RANGE_G;
  write16(header + 56, 250u); /* MPU gyro range: +/-250 deg/s. */
  write16(header + 58, ATTITUDE_TRACE_SIZE);
  FlightLog_Write32(header + 60, FLIGHT_LOG_ESTIMATOR_ID);
  head = count = 0u;
  overwritten = 0u;
  last_sample = now - FLIGHT_RECORDER_INTERVAL_MS;
  recording = true;
}

void FlightRecorder_Record(const FlightRecorderRecord_t *record, bool force)
{
  if (!recording || record == NULL ||
      (!force && record->timestamp_ms - last_sample < FLIGHT_RECORDER_INTERVAL_MS))
    return;
  last_sample = record->timestamp_ms;
  records[head] = *record;
  head = (uint16_t)((head + 1u) % FLIGHT_RECORDER_CAPACITY);
  if (count < FLIGHT_RECORDER_CAPACITY) count++;
  else overwritten++;
}

void FlightRecorder_End(uint8_t reason, uint16_t error, uint8_t hal_status, uint32_t now)
{
  if (!recording) return;
  recording = false;
  /* Bit 2 records gyro-based roll/pitch damping, including compact downloads. */
  write16(header + 6, (overwritten > 0u ? 1u : 0u) | 4u);
  write16(header + 12, count);
  header[18] = reason;
  header[19] = hal_status;
  write16(header + 20, error);
  FlightLog_Write32(header + 28, now);
  FlightLog_Write32(header + 32, overwritten);
  /* All checksum work happens after motors have been stopped, on BEGIN. */
}

bool FlightRecorder_IsRecording(void) { return recording; }
bool FlightRecorder_IsTransferring(uint32_t now)
{
  if (transferring && now - last_request >= FLIGHT_RECORDER_TRANSFER_TIMEOUT_MS)
    transferring = false;
  return transferring;
}
void FlightRecorder_CancelTransfer(void) { transferring = false; }

bool FlightRecorder_HasFlight(void) { return has_flight; }

void FlightRecorder_Snapshot(const uint8_t config[64], const FlightRecorderRecord_t *sample)
{
  if (has_flight || FlightRecorder_IsTransferring(sample->timestamp_ms)) return;
  FlightRecorder_Begin(config, sample->timestamp_ms);
  has_flight = false;
  FlightRecorder_Record(sample, true);
  FlightRecorder_End(FLIGHT_LOG_STOP_SNAPSHOT, sample->error, 0u, sample->timestamp_ms);
}

void FlightRecorder_SetLoopTiming(float dt_ms)
{
  if (loop_count == UINT32_MAX) return;
  ++loop_count;
  average_loop_ms += ((double)dt_ms - average_loop_ms) / loop_count;
}

void FlightRecorder_LogMessage(uint32_t now, uint16_t code, bool error, const char *message)
{
  if (message == NULL || FlightRecorder_IsTransferring(now)) return;
  uint8_t *event = events[event_head];
  memset(event, 0, FLIGHT_RECORDER_EVENT_SIZE);
  FlightLog_Write32(event, now);
  write16(event + 4, code);
  event[6] = error ? 1u : 0u;
  unsigned length = 0;
  while (length < FLIGHT_RECORDER_EVENT_SIZE - 8u && message[length] != '\0') length++;
  event[7] = (uint8_t)length;
  memcpy(event + 8, message, length);
  event_head = (event_head + 1u) % FLIGHT_RECORDER_EVENT_CAPACITY;
  if (event_count < FLIGHT_RECORDER_EVENT_CAPACITY) event_count++;
}

bool FlightRecorder_Respond(const uint8_t request[64], uint8_t response[64],
                           uint32_t now, bool motors_active)
{
  if (!FlightLog_IsRequestValid(request)) return false;
  memset(response, 0, 64u);
  response[0] = PACKET_TYPE_LOG_RESPONSE;
  response[2] = FLIGHT_LOG_VERSION;
  uint32_t id = FlightLog_Read32(request + 4);
  uint32_t sequence = FlightLog_Read32(request + 8);
  if (request[1] == FLIGHT_LOG_BEGIN) sequence = FLIGHT_LOG_BEGIN_SEQUENCE;
  if (request[1] == FLIGHT_LOG_END) sequence = FLIGHT_LOG_END_SEQUENCE;
  FlightLog_Write32(response + 4, id);
  FlightLog_Write32(response + 8, sequence);

  if (motors_active || recording) response[1] = FLIGHT_LOG_BUSY;
  else if (request[1] == FLIGHT_LOG_BEGIN)
  {
    if (count == 0u) response[1] = FLIGHT_LOG_EMPTY;
    else
    {
      if (!FlightRecorder_IsTransferring(now) || id != transfer_id)
      {
        float average = (float)average_loop_ms;
        uint32_t bits;
        memcpy(&bits, &average, sizeof(bits));
        FlightLog_Write32(header + 40, bits);
        FlightLog_Write32(header + 44, loop_count);
        write16(header + 48, event_count);
        write16(header + 50, FLIGHT_RECORDER_EVENT_SIZE);
        select_encoding(request[3]);
        blob_crc = calculate_crc();
      }
      transfer_id = id;
      transferring = true;
      last_request = now;
      response[3] = 12u;
      FlightLog_Write32(response + 12, blob_size());
      FlightLog_Write32(response + 16, blob_crc);
      FlightLog_Write32(response + 20, session);
    }
  }
  else if (request[1] == FLIGHT_LOG_END)
  {
    /* END is idempotent so its lost response can be retried. */
    if (id != transfer_id) response[1] = FLIGHT_LOG_SESSION_MISMATCH;
    else transferring = false;
  }
  else if (!FlightRecorder_IsTransferring(now) || id != transfer_id)
    response[1] = FLIGHT_LOG_SESSION_MISMATCH;
  else if (sequence >= (blob_size() + FLIGHT_LOG_PAYLOAD_SIZE - 1u) / FLIGHT_LOG_PAYLOAD_SIZE)
    response[1] = FLIGHT_LOG_RANGE_ERROR;
  else
  {
    last_request = now;
    uint32_t offset = sequence * FLIGHT_LOG_PAYLOAD_SIZE;
    uint32_t remaining = blob_size() - offset;
    response[3] = remaining < FLIGHT_LOG_PAYLOAD_SIZE ? (uint8_t)remaining : FLIGHT_LOG_PAYLOAD_SIZE;
    for (unsigned i = 0; i < response[3]; i++) response[12u + i] = blob_byte(offset + i);
  }
  FlightLog_Seal(response);
  return true;
}
