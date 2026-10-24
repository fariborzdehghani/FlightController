#ifndef SRF05_TRACE_H
#define SRF05_TRACE_H

#include <stdint.h>

#define SRF05_TRACE_SIZE 16u
#define SRF05_TRACE_HAVE_ECHO 1u
#define SRF05_TRACE_HAVE_ACCEPTED 2u

typedef enum
{
  SRF05_TRACE_WAITING = 0,
  SRF05_TRACE_ACCEPTED,
  SRF05_TRACE_REJECTED,
  SRF05_TRACE_TILT_REJECTED,
  SRF05_TRACE_TIMEOUT,
  SRF05_TRACE_READ_ERROR,
  SRF05_TRACE_START_ERROR
} Srf05TraceResult_t;

/* Format 7 keeps the format-6 size: age is derived from the record timestamp
 * and last_accepted_ms, freeing four bytes for the last raw echo distance.
 * Raw distance and filter_result both belong to last_echo_ms, even when a
 * later attempt times out. Presence flags distinguish tick zero from no echo. */
typedef struct
{
  uint32_t last_echo_ms;
  uint32_t last_accepted_ms;
  float raw_distance_cm;
  uint8_t valid;
  uint8_t flags;
  uint8_t result;
  uint8_t filter_result; /* VelocityEstimatorSrf05Result_t for last_echo_ms. */
} Srf05Trace_t;

_Static_assert(sizeof(Srf05Trace_t) == SRF05_TRACE_SIZE, "SRF05 trace layout changed");

#endif
