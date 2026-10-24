#include "CommandProtocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
  uint8_t packet[64] = {0};
  assert(!CommandPacket_IsValidShape(NULL, 64));
  for (unsigned command = 0; command <= 2; command++)
  {
    memset(packet, 0, sizeof(packet));
    packet[0] = 2; packet[1] = (uint8_t)command;
    assert(CommandPacket_IsValidShape(packet, 64));
    for (unsigned size = 0; size < 64; size++)
      assert(!CommandPacket_IsValidShape(packet, size));
    assert(!CommandPacket_IsValidShape(packet, 65));
    for (unsigned i = 3; i < 64; i++)
    {
      packet[i] = 1;
      assert(!CommandPacket_IsValidShape(packet, 64));
      packet[i] = 0;
    }
    packet[2] = 1;
    assert(CommandPacket_IsValidShape(packet, 64) == (command == 0));
    packet[2] = 2;
    assert(!CommandPacket_IsValidShape(packet, 64));
  }

  memset(packet, 0, sizeof(packet)); packet[0] = 3;
  assert(CommandPacket_IsValidShape(packet, 64));
  for (unsigned i = 1; i < 64; i++)
  {
    packet[i] = 1;
    assert(!CommandPacket_IsValidShape(packet, 64));
    packet[i] = 0;
  }

  memset(packet, 0, sizeof(packet)); packet[0] = 1;
  packet[43] = 80; /* Existing hover-throttle field is permitted. */
  assert(CommandPacket_IsValidShape(packet, 64));
  const unsigned reserved[] = {35, 40, 44, 63};
  for (unsigned i = 0; i < sizeof(reserved)/sizeof(reserved[0]); i++)
  {
    packet[reserved[i]] = 1;
    assert(!CommandPacket_IsValidShape(packet, 64));
    packet[reserved[i]] = 0;
  }

  memset(packet, 0, sizeof(packet));
  packet[0] = PACKET_TYPE_LOG_REQUEST; packet[1] = FLIGHT_LOG_BEGIN;
  packet[2] = FLIGHT_LOG_VERSION; packet[4] = 1;
  FlightLog_Seal(packet);
  assert(CommandPacket_IsValidShape(packet, 64));
  packet[10] = 1;
  assert(!CommandPacket_IsValidShape(packet, 64));

  /* UART noise captured on the attached board, with a command-like prefix.
   * None of it may become ARM, DISARM, TAKEOFF or a heartbeat. */
  const uint8_t noise[64] = {
    0x30,0x61,0xa0,0x10,0x30,0x30,0x61,0x20,0x30,0x00,0x61,0x50,
    0x41,0x30,0x30,0x61,0x50,0x30,0x30,0x71,0x40,0x30,0x30,0x00,
    0x30,0x10,0x30,0x30,0x61,0x20,0x30,0x00,0xf0,0x11,0x11,0x40,
    0x30,0x00,0x82,0x00,0x00,0xce,0x0b,0x53,0x34,0x83,0x0a,0x75,
    0x82,0x00,0x21};
  for (unsigned command = 0; command <= 2; command++)
  {
    memcpy(packet, noise, sizeof(packet)); packet[0] = 2;
    packet[1] = (uint8_t)command; packet[2] = 0;
    assert(!CommandPacket_IsValidShape(packet, 64));
  }
  packet[0] = 3;
  assert(!CommandPacket_IsValidShape(packet, 64));
  puts("shared command packet tests passed");
}
