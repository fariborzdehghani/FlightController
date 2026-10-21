# Radio diagnostics, flight logs and playback

Update all three projects together. No CubeMX settings or regeneration are
required for this feature; the existing UART, SPI, motor timers and LED are used.

## Build and use

The current firmware builds are `build/Debug/FlightController.elf`
and `../RemoteController/build/Debug/RemoteController.elf`. Flash each
to its corresponding board using your usual STM32 programmer. The desktop build
is in `D:\Electronic Projects\Flight Controller\07-Desktop Apps\DroneLogger\DroneLogger\bin\CompactLog\Debug\net8.0-windows`.
Run `DroneLogger.exe` from that folder, keeping its companion files beside it.

1. In Radio mode, connect the desktop **Transmitter Port** to the radio bridge.
   The aircraft does not need a serial cable and the Logger Port can stay disconnected.
2. Wait for aircraft startup/calibration to finish and send your configuration.
3. **Space** anywhere in the main application window starts takeoff. It sends
   ARM first if needed. Press Space again to send DISARM immediately. Holding
   Space does not repeat the toggle. The TAKE OFF button also becomes STOP
   during a requested takeoff. The separate DISARM button remains available.
4. While disarmed, open **Flight Logs**, keep **Compact PID log** selected, and
   press **Download diagnostics**. Select **Full precision log** when you need
   yaw or altitude PID channels, or the original floating-point precision.
   This works before the first takeoff and before SET CONFIG as soon as startup
   completes. After a test, press **DISARM** yourself before downloading.
   Download does not send a motor command or automatically disarm an aircraft.
   ARM/takeoff and configuration changes are disabled during download.
5. While transferring, the aircraft LED repeats two fast flashes: 80 ms on,
   80 ms off, 80 ms on, then 760 ms off. Its usual state pattern returns when
   transfer ends. An abandoned transfer expires after eight seconds.
6. On a successful, checksum-verified download, the app fills all existing
   graphs and saves a folder in **Documents\DroneLogger\FlightLogs**. Use
   **Logs folder** to open it. Full logs can take minutes over the current LoRa
   settings; progress updates as chunks arrive. **Cancel download** retains
   the onboard log, so you can retry.
7. **Flight Logs → Overview** shows recorded signal charts, altitude and a 3D
   view. **Play flight** replays them along with the existing views using recorded
   timestamps. Use the timeline to seek, **Restart**, or the speed selector
   (0.25×, 0.5×, 1×, 2×). Pause/resume is available.
   **Messages** lists recent firmware information/errors, including rejected
   takeoff commands. The loop-timing card shows the actual average main-loop dt
   in milliseconds over every iteration since startup, captured at download
   BEGIN, plus the iteration count in its tooltip/settings. This is not the
   average of the sparse retained samples. Older v1 logs show average unavailable.
   **Samples** lists raw values with field groups and a faults/output-limits
   filter; selecting a row pauses playback at that sample. **Raw data** shows
   JSON, original log bytes in hex, or CSV, with a copy button. **Recorded settings**
   shows the PID gains and configuration that were used during the flight.
   **Export CSV** saves a separate sample file. **Open flight log** loads a saved recording for
   later replay. Playback sends no commands to the aircraft. Space still starts
   a real flight test and exits playback.

   **PID review** puts roll or pitch on four linked time charts: target/measured
   angle/error, gyro rate, P/I/D and applied total, and all motor commands.
   Pan or zoom one chart to move the common time range; use the playback slider
   to inspect the same instant in all four. Gains and targets come from the
   recording. Older logs show gyro rates as unavailable.

Tab and Shift+Tab select all text when entering a textbox. The existing arrow
shortcuts now have one key handler, avoiding duplicate throttle changes.

Verify Space/STOP and a short radio download on the bench with propellers
removed before relying on the updated system during a flight test.

## What is retained

From boot, a separate ring retains the latest 16 information/error messages,
each with boot timestamp, code, severity, and up to 96 UTF-8 bytes of text.
Before any accepted takeoff, each new download captures one current disarmed
sensor/state sample and the current applied configuration (or marks it absent).
This diagnoses rejected commands without requiring a flight. Subsequent accepted
takeoff replaces that snapshot with the normal flight recording. Downloads after
a flight keep its samples/configuration and append the recent diagnostic messages.
Messages are frozen during a download; messages emitted during transfer are omitted.
Loop timing continues accumulating, but the downloaded average/count remain frozen
for that transfer. A fresh download captures the updated average/count.

The recorder starts on an **accepted TAKEOFF** and samples at most once per
10 ms. It keeps the latest 512 records: about 5.12 seconds at 100 Hz, or
7.68 seconds with the observed 15 ms control-loop period. Sampling follows
actual control updates rather than a new timer interrupt.
Longer tests overwrite the oldest records, and the download reports the
number overwritten. A forced first and final event preserve short attempts,
even those shorter than one sample interval. Firmware faults, manual DISARM,
the aircraft heartbeat failsafe, and the bridge operator-link timeout freeze
the recording. A bridge timeout is marked as link loss rather than a pilot stop.

Samples contain roll/pitch/yaw, yaw rate, vertical velocity and validity,
distance-sensor altitude and validity, base throttle, all four motor outputs,
roll/pitch/yaw-rate/altitude PID terms, roll/pitch totals, loop timestep, altitude
target, altitude-control engagement, mixer limiting, and roll/pitch gyro rates
with their validity flag. Roll follows MPU6050 Gy and pitch follows Gx, matching
the existing attitude axis mapping. The final event includes
the stop state and application error, with motor commands set to zero. Its PID
terms are the last calculated terms, not a calculation made after stopping.

The configuration snapshot records the quantized gains and limits actually
accepted by firmware, the effective hover throttle, and enabled sensors. Graph
targets come from this snapshot rather than whatever happens to be typed in
the current desktop settings. Invalid altitude/velocity measurements appear
as gaps in graphs and N/A in labels; their raw values remain in the saved data.

The sample ring uses **63,488 bytes**; metadata and bookkeeping are small fixed
buffers. There is no allocation, file I/O, flash writing, or radio log streaming
while flying. Radio mode also avoids formatting live serial telemetry.
The current Debug build uses **72,616 bytes of 128 KB main RAM**, including
the recorder and the linker's reserved heap/stack. RAM is not cleared/copied
as a whole on every test, and download reads the ring in chronological order
without creating another onboard copy.

**Download before power-off/reset or the next accepted takeoff.** This is volatile
RAM: power loss, brownout, a watchdog reset, or a hard fault followed by reset
can destroy the recording. It cannot diagnose a reset that erased its own RAM.
Changing configuration after stopping does not change the settings saved with
the previous log. Arming alone does not erase it; the next accepted takeoff does.

## Saved files and analysis

Each download gets a unique folder named by desktop receipt time and aircraft
session id:

- `flight.json`: settings, effective hover throttle, stop description/error,
  validity flags, units and every sample. This is the best single file to share
  for analysis.
- `samples.csv`: sample values and timing for spreadsheet/plotting tools.
- `diagnostics.txt`: average loop dt/count and recent timestamped messages.
  The same fields are also included in `flight.json` and the binary archive.
- `flight.flightlog`: original binary recording with an archive CRC32; use
  **Open flight log** to verify and replay it.

Timestamps are milliseconds since MCU boot. `elapsed_ms` starts at the first
retained sample; if the ring wrapped, that is later than the takeoff start.
`StartTickMs` and `EndTickMs` preserve the whole test boundaries. Angles are
degrees, yaw rate is degrees/second, vertical velocity is m/s, altitude is cm,
motor commands are percent, and loop timestep is milliseconds.

Playback visualizes measured attitude and height; the firmware does not record
horizontal position, so it cannot reconstruct a complete flight path. A sampled
log cannot resolve oscillations above its sampling limit. Use the logs to
establish whether the stop was caused by tilt, missing altitude, link loss,
timing, or takeoff timeout before changing PID gains.

## Shared wire format (version 1)

All commands and replies are exactly 64 bytes. Existing configuration (1),
control (2), and heartbeat (3) layouts remain compatible. DISARM byte 2 is zero
for an operator stop and optionally 1 for bridge operator-link timeout.

| Bytes | Log request (type 4) | Log response (type 5) |
| --- | --- | --- |
| 0 | 4 | 5 |
| 1 | BEGIN=1, READ=2, END=3 | OK=0, empty=1, busy=2, session mismatch=3, range error=4 |
| 2 | Version=1 | Version=1 |
| 3 | BEGIN profile: 0 compact (default), 1 full precision; READ/END zero | Payload length, 0..48 |
| 4..7 | Nonzero desktop transfer id | Echoed transfer id |
| 8..11 | READ chunk index; otherwise zero | READ index; BEGIN=0xFFFFFFFF; END=0xFFFFFFFE |
| 12..59 | Zero | Payload |
| 60..61 | Zero | Zero |
| 62..63 | CRC-16/CCITT-FALSE of bytes 0..61 | Same CRC |

BEGIN payload contains blob length, blob CRC32 (IEEE), and aircraft session id
as three little-endian uint32 values. READ returns up to 48 bytes at chunk
index × 48. The desktop requests only one outstanding chunk and retries up to
six times, with a one-second response timeout. Matching transfer id and sequence
reject stale replies, and frame CRC16 plus whole-blob CRC32 detect corruption.
END is idempotent. Requests are validated by the bridge and the aircraft;
flight log requests do not refresh the aircraft's flight heartbeat.

The bridge forwards validated responses as `FlightLog=` followed by 128 hex
digits and newline over its existing 115200-baud serial port. The direct serial
path uses the same envelope. Aircraft radio TX is polled without waiting in
the flight loop, allowing the LED, UART and watchdog to keep running. Motors
are always stopped before log-transfer or final checksum work.

Default blob format is now **4** (compact PID log); full-precision downloads use
format **3**. The 64-byte packet transport remains version **1** on all three
projects. The desktop can still open format-1, format-2 and format-3 recordings.
Blob layout: 64-byte header, 64-byte frozen configuration snapshot,
chronological 50-byte compact or 124-byte full records, then chronological 104-byte messages, all
little-endian. Stop reason 4 means a disarmed diagnostic snapshot; only this
kind may contain configuration byte 0 equal to zero (not yet configured).
In the log snapshot only,
configuration byte 44 stores effective hover throttle × 2 and byte 45 stores
enabled sensor bits (MPU6050=1, QMC5883=2, SRF05=4, BMP280=8). These are not new
configuration-command settings.

Header offsets: magic `FLOG` at 0; uint16 version at 4, flags at 6 (bit 0 overwritten,
bit 1 automatic full-precision fallback),
uint32 session at 8, uint16 count at 12, record size at 14, interval at 16;
byte stop reason at 18, HAL status at 19, uint16 application error at 20;
uint32 start tick at 24, stop tick at 28, overwritten count at 32;
uint16 configuration size at 36 and header size at 38. Format 2 adds float32
average loop dt in milliseconds at 40, uint32 iteration count at 44,
uint16 message count at 48 and message size (104) at 50. Unused bytes are zero.
Each message has uint32 boot tick at 0, uint16 diagnostic code at 4,
byte severity at 6 (0 information, 1 error), byte text length at 7,
then up to 96 UTF-8 text bytes at 8. Counts are bounded to 16 messages.

Each record has uint32 timestamp at 0; state byte at 4; flags at 5 (Vz valid=1,
altitude valid=2, altitude control=4, mixer limited=8, gyro rates valid=16);
uint16 error at 6. In format 3, twenty-nine float32 values follow at offset 8: roll, pitch, yaw, Gz, Vz,
altitude, base throttle, M1..M4, roll P/I/D/total, pitch P/I/D/total,
Gz P/I/D, altitude P/I/D, loop dt in ms, altitude target in cm, roll gyro rate,
pitch gyro rate in deg/s. Formats 1 and 2 retain their 116-byte / 27-value layout,
256-record capacity and 50 ms minimum interval. Format 3 uses 512 records and
a 10 ms minimum interval, as does format 4.

Format 4 replaces the float payload with 21 signed little-endian int16 values:

| Order | Values | Scale (stored = round(value × scale)) | Resolution |
| --- | --- | --- | --- |
| 0..1 | Roll, pitch | 100 | 0.01 deg |
| 2..3 | Roll/pitch gyro rates | 100 | 0.01 deg/s |
| 4..7 | Roll P/I/D/total | 1000 | 0.001 percentage point |
| 8..11 | Pitch P/I/D/total | 1000 | 0.001 percentage point |
| 12..16 | M1, M2, M3, M4, base throttle | 100 | 0.01 percentage point |
| 17..18 | Altitude, altitude target | 10 | 0.1 cm |
| 19 | Vertical velocity | 1000 | 0.001 m/s |
| 20 | Loop timestep | 100 | 0.01 ms |

Full floats remain in the onboard ring. Encoding and representability checks
run only during a disarmed BEGIN, so sampling rate and flight-loop work are
unchanged. If any retained compact field is nonfinite or its scaled value is
outside -32767..32767, the entire transfer automatically uses format 3 with
header flag bit 1 set. Nothing is clipped. Explicit full downloads clear that
flag. A retried BEGIN for the same active transfer keeps its original profile,
bytes and checksum even if byte 3 changes.

The desktop defaults to **Compact PID log**; **Full precision log** can download
the same onboard flight again with yaw, yaw-rate PID and altitude PID channels.
Those eight channels are absent from compact downloads and decode as unavailable
(NaN), never zero. The compact 3D view uses a fixed heading and labels it as such.
All messages, stop information, flags and configuration bytes are preserved.
New BEGIN profile validation is synchronized in the bridge and aircraft headers.
An older aircraft still accepts the default zero profile and returns its older
full format; requesting the optional full profile requires updated firmware.

Samples shrink by 59.7% (124 to 50 bytes). The 21:38 flight with 246 samples and
16 diagnostic messages would shrink from 32,296 to 14,092 radio payload bytes,
requiring 294 instead of 673 READ replies. Actual elapsed download time depends
on radio retries; radio modulation, response pacing and chunk size are unchanged.

The `.flightlog` archive wraps that blob with four bytes `FDL1` and a
little-endian CRC32 of the blob. The decoder checks length, version, layout,
sample count and timestamp order before replaying any file.

## Verification

The C host tests (`flight_recorder_test.c`, `diagnostics_test.c` and
`compact_flight_log_test.c`, compiled with `-lm`, `Core/Src/FlightRecorder.c`
and `-I Core/Inc` in Linux/WSL) cover ring rollover,
sample throttling, frozen data, forced events, configuration preservation,
requests/checksums, armed rejection, invalid ranges, transfer expiration,
idempotent END and MCU tick rollover. Diagnostics tests additionally check
downloads before configuration/takeoff, bounded messages, true all-loop average,
frozen retries, and preservation of the previous flight.
Compact tests cover integer precision/sign, full-fidelity redownload, range and
nonfinite fallback, ring wraparound, frozen profile retries and whole-blob CRC.
The shared request-validator test is compiled against both firmware headers.

Desktop tests:

```powershell
dotnet test tests\DroneLogger.PacketTests\DroneLogger.PacketTests.csproj
```

`Fixtures/flight-log-v1.bin` remains a compatibility fixture from the original
recorder, and `Fixtures/diagnostics-v2.bin` preserves format-2 compatibility.
`Fixtures/flight-log-v3.bin` is generated by the production C recorder via
`flight_recorder_test <output-path>`.
`Fixtures/flight-log-v4.bin` is generated by the production compact encoder via
`compact_flight_log_test <output-path>`.
Tests decode every field of that fixture, exercise dropped/corrupt/duplicate
packets, cancellation and rejection, save/reopen, tick wraparound, full graph
loading with frozen targets, invalid-sensor displays, graph resets, and rapid
stop while the desktop ARM write is still outstanding.

Firmware builds and desktop tests do not replace testing the radio timing,
physical keyboard, LED and motor stop behavior on the actual boards.
