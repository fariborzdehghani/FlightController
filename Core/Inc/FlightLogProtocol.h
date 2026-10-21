#ifndef FLIGHT_LOG_PROTOCOL_H
#define FLIGHT_LOG_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define PACKET_TYPE_LOG_REQUEST 4u
#define PACKET_TYPE_LOG_RESPONSE 5u
#define FLIGHT_LOG_VERSION 1u
/* Transport version stays 1. The opaque downloaded blob has its own version;
 * format 4 is the compact roll/pitch tuning download. */
#define FLIGHT_LOG_BEGIN 1u
#define FLIGHT_LOG_READ 2u
#define FLIGHT_LOG_END 3u
/* BEGIN byte 3 selects the download; READ/END must leave it zero. */
#define FLIGHT_LOG_PROFILE_COMPACT 0u
#define FLIGHT_LOG_PROFILE_FULL 1u
#define FLIGHT_LOG_PAYLOAD_SIZE 48u
#define FLIGHT_LOG_OK 0u
#define FLIGHT_LOG_EMPTY 1u
#define FLIGHT_LOG_BUSY 2u
#define FLIGHT_LOG_SESSION_MISMATCH 3u
#define FLIGHT_LOG_RANGE_ERROR 4u
#define FLIGHT_LOG_BEGIN_SEQUENCE UINT32_MAX
#define FLIGHT_LOG_END_SEQUENCE (UINT32_MAX - 1u)
/* Optional DISARM byte 2 distinguishes bridge operator-link loss from a pilot stop. */
#define CONTROL_DISARM_REASON_LINK_TIMEOUT 1u

static inline uint32_t FlightLog_Read32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline void FlightLog_Write32(uint8_t *p, uint32_t value)
{
  for (unsigned i = 0; i < 4; i++) p[i] = (uint8_t)(value >> (8u * i));
}

/* CRC-16/CCITT-FALSE covers bytes 0..61 on requests and responses. */
static inline uint16_t FlightLog_Crc16(const uint8_t *data, unsigned length)
{
  uint16_t crc = 0xffffu;
  for (unsigned i = 0; i < length; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (unsigned bit = 0; bit < 8; bit++)
      crc = (uint16_t)(((uint32_t)crc << 1) ^ ((crc & 0x8000u) ? 0x1021u : 0u));
  }
  return crc;
}

static inline void FlightLog_Seal(uint8_t *packet)
{
  uint16_t crc = FlightLog_Crc16(packet, 62u);
  packet[62] = (uint8_t)crc;
  packet[63] = (uint8_t)(crc >> 8);
}

static inline bool FlightLog_IsRequestValid(const uint8_t *packet)
{
  if (packet[0] != PACKET_TYPE_LOG_REQUEST || packet[2] != FLIGHT_LOG_VERSION ||
      packet[1] < FLIGHT_LOG_BEGIN || packet[1] > FLIGHT_LOG_END ||
      FlightLog_Read32(packet + 4) == 0u ||
      FlightLog_Crc16(packet, 62u) != (uint16_t)(packet[62] | (packet[63] << 8)))
    return false;
  if ((packet[1] == FLIGHT_LOG_BEGIN ? packet[3] > FLIGHT_LOG_PROFILE_FULL : packet[3] != 0u) ||
      (packet[1] != FLIGHT_LOG_READ && FlightLog_Read32(packet + 8) != 0u))
    return false;
  for (unsigned i = 12; i < 62; i++) if (packet[i] != 0u) return false;
  return true;
}

static inline bool FlightLog_IsResponseValid(const uint8_t *packet)
{
  return packet[0] == PACKET_TYPE_LOG_RESPONSE &&
         packet[2] == FLIGHT_LOG_VERSION && packet[1] <= FLIGHT_LOG_RANGE_ERROR &&
         packet[3] <= FLIGHT_LOG_PAYLOAD_SIZE &&
         FlightLog_Crc16(packet, 62u) == (uint16_t)(packet[62] | (packet[63] << 8));
}

#endif
