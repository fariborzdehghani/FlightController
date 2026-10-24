#ifndef FLIGHT_RECORDER_H
#define FLIGHT_RECORDER_H

#include "FlightLogProtocol.h"
#include "AttitudeTrace.h"
#include "Srf05Trace.h"

#define FLIGHT_RECORDER_CAPACITY 512u
/* Capture each typical 15 ms control update, with a 100 Hz upper bound. */
#define FLIGHT_RECORDER_INTERVAL_MS 10u
#define FLIGHT_RECORDER_HEADER_SIZE 64u
#define FLIGHT_RECORDER_RECORD_SIZE 208u
#define FLIGHT_RECORDER_TRANSFER_TIMEOUT_MS 8000u
#define FLIGHT_RECORDER_EVENT_CAPACITY 16u
#define FLIGHT_RECORDER_EVENT_SIZE 104u
#define FLIGHT_RECORDER_FORMAT_VERSION 7u
#define FLIGHT_RECORDER_COMPACT_FORMAT_VERSION 4u
#define FLIGHT_RECORDER_COMPACT_RECORD_SIZE 50u

#define FLIGHT_LOG_STOP_OPERATOR 1u
#define FLIGHT_LOG_STOP_LINK_TIMEOUT 2u
#define FLIGHT_LOG_STOP_FAULT 3u
#define FLIGHT_LOG_STOP_SNAPSHOT 4u

typedef struct
{
  uint32_t timestamp_ms;
  uint8_t state;
  uint8_t flags; /* Vz valid, altitude valid, altitude control, mixer limited, gyro rates, landing */
  uint16_t error;
  float values[29]; /* v2's 27 values, then roll/pitch gyro rates in deg/s */
  AttitudeTrace_t attitude_trace; /* Format 5; compact format 4 omits this. */
  Srf05Trace_t srf05_trace; /* Format 7; compact format 4 omits this. */
} FlightRecorderRecord_t;

void FlightRecorder_Begin(const uint8_t configuration[64], uint32_t now);
void FlightRecorder_Record(const FlightRecorderRecord_t *record, bool force);
void FlightRecorder_End(uint8_t reason, uint16_t error, uint8_t hal_status, uint32_t now);
bool FlightRecorder_IsRecording(void);
bool FlightRecorder_IsTransferring(uint32_t now);
void FlightRecorder_CancelTransfer(void);
bool FlightRecorder_HasFlight(void);
void FlightRecorder_Snapshot(const uint8_t configuration[64], const FlightRecorderRecord_t *sample);
void FlightRecorder_LogMessage(uint32_t now, uint16_t code, bool error, const char *message);
void FlightRecorder_SetLoopTiming(float dt_ms);
bool FlightRecorder_Respond(const uint8_t request[64], uint8_t response[64],
                           uint32_t now, bool motors_active);

#endif
