#include "FlightLogProtocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
  uint8_t packet[64];
  assert(!FlightLog_IsRequestValid(NULL));
  assert(!FlightLog_IsResponseValid(NULL));
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

  /* Same offsets as the desktop BuildRequest: a nonzero transfer ID is
   * independent of the request's chunk index. BEGIN/END use chunk index zero;
   * READ must accept chunk zero and subsequent chunks. The bridge previously
   * required the BEGIN response sentinel as the ID and rejected later reads.
   */
  const uint32_t ids[] = {1u, 123u, 0x12345678u, UINT32_MAX};
  const uint32_t chunks[] = {0u, 1u, 48u, 1359u};
  for (unsigned id = 0; id < sizeof(ids) / sizeof(ids[0]); ++id)
    for (unsigned chunk = 0; chunk < sizeof(chunks) / sizeof(chunks[0]); ++chunk)
      for (unsigned operation = FLIGHT_LOG_BEGIN; operation <= FLIGHT_LOG_END; ++operation)
      {
        memset(packet, 0, sizeof(packet));
        packet[0] = PACKET_TYPE_LOG_REQUEST;
        packet[1] = operation;
        packet[2] = FLIGHT_LOG_VERSION;
        FlightLog_Write32(packet + 4, ids[id]);
        FlightLog_Write32(packet + 8, chunks[chunk]);
        FlightLog_Seal(packet);
        assert(FlightLog_IsRequestValid(packet) ==
               (operation == FLIGHT_LOG_READ || chunks[chunk] == 0u));
        FlightLog_Write32(packet + 4, 0u);
        FlightLog_Seal(packet);
        assert(!FlightLog_IsRequestValid(packet));
        FlightLog_Write32(packet + 4, ids[id]);
        packet[12] = 1u;
        FlightLog_Seal(packet);
        assert(!FlightLog_IsRequestValid(packet));
      }
  puts("shared flight-log protocol tests passed");
  return 0;
}
