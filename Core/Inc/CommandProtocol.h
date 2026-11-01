#ifndef COMMAND_PROTOCOL_H
#define COMMAND_PROTOCOL_H

#include "FlightLogProtocol.h"

#define COMMAND_PACKET_SIZE 64u

/* Keep identical in FlightController and RemoteController. Control and
 * heartbeat builders zero every reserved byte. Check the complete frame,
 * rather than treating the first two bytes of UART noise as a command. */
static inline bool CommandPacket_IsValidShape(const uint8_t *packet,
                                             unsigned length)
{
  if (packet == NULL || length != COMMAND_PACKET_SIZE) return false;

  unsigned reserved_start;
  switch (packet[0])
  {
  case 1u: /* Configuration: semantic limits are checked by the receiver. */
    /* New senders use zero. Accept a bounded legacy hover value, which the
     * aircraft ignores while learning its own throttle. Both receivers agree. */
    if (packet[43] != 0u && (packet[43] < packet[1] ||
        packet[43] > packet[3] || packet[43] > 100u)) return false;
    /* 44/45 are recorder-only metadata; 46 disables the SRF05 failsafe. */
    if (packet[35] != 0u || packet[40] != 0u || packet[44] != 0u ||
        packet[45] != 0u || packet[46] > 1u) return false;
    reserved_start = 47u;
    break;
  case 2u: /* DISARM=0, ARM=1, TAKEOFF=2, LAND=3, throttle +/-1%=4/5. */
    if (packet[1] > 5u ||
        (packet[1] == 0u ? packet[2] > CONTROL_DISARM_REASON_LINK_TIMEOUT
                         : packet[2] != 0u)) return false;
    reserved_start = 3u;
    break;
  case 3u: /* Heartbeat. */
    reserved_start = 1u;
    break;
  case PACKET_TYPE_LOG_REQUEST:
    return FlightLog_IsRequestValid(packet);
  default:
    return false;
  }

  for (unsigned i = reserved_start; i < COMMAND_PACKET_SIZE; i++)
    if (packet[i] != 0u) return false;
  return true;
}

#endif
