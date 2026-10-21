#include "FlightLogProtocol.h"
#include <assert.h>
#include <string.h>

int main(void)
{
  uint8_t packet[64];
  for (unsigned operation = FLIGHT_LOG_BEGIN; operation <= FLIGHT_LOG_END; operation++)
    for (unsigned profile = 0; profile < 3; profile++)
    {
      memset(packet, 0, sizeof(packet));
      packet[0] = PACKET_TYPE_LOG_REQUEST;
      packet[1] = operation;
      packet[2] = FLIGHT_LOG_VERSION;
      packet[3] = profile;
      FlightLog_Write32(packet + 4, 123);
      FlightLog_Seal(packet);
      bool allowed = operation == FLIGHT_LOG_BEGIN ? profile <= FLIGHT_LOG_PROFILE_FULL : profile == 0;
      assert(FlightLog_IsRequestValid(packet) == allowed);
      packet[62] ^= 1;
      assert(!FlightLog_IsRequestValid(packet));
    }
  return 0;
}
