# Radio diagnostics, flight logs and playback

Update all three projects together. No CubeMX settings or regeneration are
required for this feature; the existing UART, SPI, motor timers and LED are used.

## Build and use

The current firmware builds are `build/Debug/FlightController.elf`
and `../RemoteController/build/Debug/RemoteController.elf`. Flash each
to its corresponding board using your usual STM32 programmer. The desktop build
is in `D:\Electronic Projects\Flight Controller\07-Desktop Apps\DroneLogger\DroneLogger\bin\RadioDownload\net8.0-windows`.
Run `DroneLogger.exe` from that folder, keeping its companion files beside it.

Both radio firmwares now default to **250 kHz bandwidth** for faster log
downloads. Flash **both boards** before using the radio link. The matching
`Core/Inc/RadioConfig.h` files contain `RADIO_BANDWIDTH_HZ`, which accepts
`250000u` (default) or `125000u` (the previous setting). Keep the two files in
sync; compiler `-D` overrides must also match. Bandwidth applies to the whole
radio link, including control packets. The faster setting trades receiver
sensitivity/range for shorter packet airtime; actual download speed still
depends on turnaround, serial processing and retries. No CubeMX regeneration
or desktop packet-format change is needed.

1. In Radio mode, connect the desktop **Transmitter Port** to the radio bridge.
   The aircraft does not need a serial cable and the Logger Port can stay disconnected.
2. Wait for aircraft startup/calibration to finish and send your configuration.
3. **Space** anywhere in the main application window starts takeoff. It sends
   ARM first if needed. Press Space again to send DISARM immediately. Holding
   Space does not repeat the toggle. The TAKE OFF button also becomes STOP
   during a requested takeoff. The separate DISARM button remains available.
4. While disarmed, open **Flight Logs**, keep **Compact PID log** selected, and
   press **Download diagnostics**. Select **Full precision + sensor traces** when
   diagnosing roll/pitch estimation, or when you need yaw/altitude PID channels
   and original floating-point precision.
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
   The progress area shows elapsed time, estimated remaining time, and estimated
   total time, without a `~` prefix on the displayed durations. Estimates use accepted payload bytes and average payload-transfer
   speed, including retry delays; the total also includes the initial BEGIN wait.
   They remain "Estimating" before data arrives, then update every 250 ms even
   while waiting for a response. After all bytes arrive, "Finishing" covers
   verification, release and saving. Successful downloads retain the actual total
   elapsed time; cancellation/failure clears the estimates and freezes elapsed
   time when the operation ends. Opening a saved recording hides old timings.
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

Continuous rapid LED flashing (100 ms on/off, without a long pause) means
FAULT. Five 200 ms flashes followed by a one-second pause mean startup is ready.
After a startup fault, valid diagnostic-download requests are allowed over the
initialized radio link while ARM/configuration remain locked out. The download
captures the fault state, application error code and recent startup messages;
it does not recover or clear the fault. UART command reception may not have
started if initialization failed, so use the radio path in that case.

MPU6050 initialization wakes and settles the module before configuring its
sample rate, bandwidth and ranges. ST-Link debugging on this module showed
that writes while asleep were acknowledged but ignored: divider, filter and
accelerometer range stayed at zero, so the new +/-8 g scale was being applied
to +/-2 g output and startup calibration timed out. An awake range write took
effect immediately. The corrected order retains the same settling delays.
Older log headers recorded the requested cutoff/range, without register
readback. Those headers alone do not prove the MPU actually applied a setting;
the pre-fix board read back divider=0 and filter=0 as well as range=0.
Initialization reads back all four configuration registers and checks their
divider/filter/range/self-test bits before continuing. A mismatch or readback
error stops startup and records a specific message. Calibration timeouts record
the observed acceleration-norm range, peak gyro rate, rejected-sample counts
and longest stationary window. The thresholds remain unchanged. These details
distinguish a range/output mismatch from motion or another startup failure.

On 2026-10-22 the corrected firmware was flashed and verified through ST-Link.
Two software-reset startups completed calibration and sensor warm-up, with
hardware ready, commands enabled, no application/HAL error and motors stopped.
Live register reads were SMPLRT_DIV=0x07, CONFIG=0x04, GYRO_CONFIG=0x00,
ACCEL_CONFIG=0x10: 125 Hz sampling, 20 Hz gyro cutoff, +/-250 deg/s gyro range
and +/-8 g accelerometer range. Bench captures and GDB transcripts are retained
under `build/stlink-debug-20261022/`. A power-cycle and flight were not part of
that verification.

The subsequent [live sensor bench test](SENSOR_BENCH_20261022.md) checked slow
roll/pitch calculations and found evidence of accelerometer offset, especially
on Z. It also reproduced garbage commands on floating PA10 / USART1_RX. The user
regenerated CubeMX code with PA10 Pull-up; live register and UART checks verified
the change. Both firmware receivers now validate complete 64-byte command shapes,
DISARM preserves INIT before configuration, and ARM requires valid configuration.
The rebuilt flight firmware was flashed and verified; the final still capture
remained disarmed with zero motor output and no errors. No accelerometer
calibration coefficients were applied; see the bench report for measurements,
capture limitations and the recommended next calibration.

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
even those shorter than one sample interval. Manual DISARM, hard faults, and ordinary control-link stops freeze the
recording. Altitude loss after confirmed liftoff and takeoff timeouts now keep
recording through a gradual throttle-down, freezing only once motors stop.
A link timeout during that landing does not interrupt the rundown; an explicit
operator DISARM remains immediate. A bridge timeout outside landing is still
marked as link loss rather than a pilot stop.

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

The sample ring uses **106,496 bytes (104 KiB)**; metadata and bookkeeping are small fixed
buffers. There is no allocation, file I/O, flash writing, or radio log streaming
while flying. Radio mode also avoids formatting live serial telemetry.
The current Debug build uses **116,024 bytes of 128 KiB main RAM**, including
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
Every request carries its nonzero transfer id at bytes 4..7. READ chunk indexes
at bytes 8..11 start at zero; BEGIN and END requests leave those bytes zero.
The special BEGIN/END sequence values are used only in responses. Keep the
aircraft and bridge validators identical so valid requests are not discarded.
END is idempotent. Requests are validated by the bridge and the aircraft;
flight log requests do not refresh the aircraft's flight heartbeat.

The bridge forwards validated responses as `FlightLog=` followed by 128 hex
digits and newline over its existing 115200-baud serial port. The direct serial
path uses the same envelope. Aircraft radio TX is polled without waiting in
the flight loop, allowing the LED, UART and watchdog to keep running. Motors
are always stopped before log-transfer or final checksum work.

Default blob format is now **4** (compact PID log); full-precision downloads use
format **7**. The 64-byte packet transport remains version **1** on all three
projects. The desktop can still open format-1 through format-6 recordings.
Blob layout: 64-byte header, 64-byte frozen configuration snapshot,
chronological 50-byte compact or 208-byte full records, then chronological 104-byte messages, all
little-endian. Stop reason 4 means a disarmed diagnostic snapshot; only this
kind may contain configuration byte 0 equal to zero (not yet configured).
In the log snapshot only,
configuration byte 44 stores effective hover throttle × 2 and byte 45 stores
enabled sensor bits (MPU6050=1, QMC5883=2, SRF05=4, BMP280=8). These are not new
configuration-command settings.

Header offsets: magic `FLOG` at 0; uint16 version at 4, flags at 6 (bit 0 overwritten,
bit 1 automatic full-precision fallback, bit 2 gyro-based roll/pitch damping),
uint32 session at 8, uint16 count at 12, record size at 14, interval at 16;
byte stop reason at 18, HAL status at 19, uint16 application error at 20;
uint32 start tick at 24, stop tick at 28, overwritten count at 32;
uint16 configuration size at 36 and header size at 38. Format 2 adds float32
average loop dt in milliseconds at 40, uint32 iteration count at 44,
uint16 message count at 48 and message size (104) at 50. Updated firmware also
records trace schema 1 at byte 52, accelerometer-rejection enabled at 53,
gyro cutoff in Hz at 54, accelerometer full-scale range at 55 (8 g in current
firmware; 2 g in earlier recordings), uint16 gyro range
(250 deg/s) at 56, uint16 trace size (68) at 58, and uint32 estimator source
signature at 60. Compact downloads retain these settings but omit the traces.
Other unused bytes are zero.

The MPU6050 now measures +/-8 g with 4096 raw counts per g to give acceleration
peaks more headroom before clipping. Startup calibration, SI-unit conversion
and the IMU trace use this same scale. This does not widen the filter's
0.8-1.2 g acceptance window or change its 15-degree innovation gate, gyro
bandwidth or PID gains. A wider sensor range does not remove vibration or make
accelerometer-derived tilt trustworthy during linear acceleration. Compare
full-trace recordings before making further estimator changes. The desktop
decoder accepts both the earlier +/-2 g and current +/-8 g metadata.

Each message has uint32 boot tick at 0, uint16 diagnostic code at 4,
byte severity at 6 (0 information, 1 error), byte text length at 7,
then up to 96 UTF-8 text bytes at 8. Counts are bounded to 16 messages.

Each record has uint32 timestamp at 0; state byte at 4; flags at 5 (Vz valid=1,
altitude valid=2, altitude control=4, mixer limited=8, gyro rates valid=16,
gradual landing=32);
uint16 error at 6. In format 3, twenty-nine float32 values follow at offset 8: roll, pitch, yaw, Gz, Vz,
altitude, base throttle, M1..M4, roll P/I/D/total, pitch P/I/D/total,
Gz P/I/D, altitude P/I/D, loop dt in ms, altitude target in cm, roll gyro rate,
pitch gyro rate in deg/s. Formats 1 and 2 retain their 116-byte / 27-value layout,
256-record capacity and 50 ms minimum interval. Format 3 uses 512 records and
a 10 ms minimum interval, as do formats 4 through 7. Format 5 preserves format 3's
124-byte prefix exactly, followed by the 68-byte IMU trace described below.
Format 6 preserves that 192-byte prefix and appends the 16-byte SRF05 trace.
Format 7 keeps the same 208-byte record, replacing redundant age with raw echo
distance and using the spare trace byte for the specific filter result.

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
outside -32767..32767, the entire transfer automatically uses format 7 with
header flag bit 1 set. Nothing is clipped. Explicit full downloads clear that
flag. A retried BEGIN for the same active transfer keeps its original profile,
bytes and checksum even if byte 3 changes.

The desktop defaults to **Compact PID log**; **Full precision + sensor traces** can
download the same onboard flight again with the IMU and SRF05 traces, yaw,
yaw-rate PID and altitude PID channels.
Those eight channels are absent from compact downloads and decode as unavailable
(NaN), never zero. The compact 3D view uses a fixed heading and labels it as such.
All messages, stop information, flags and configuration bytes are preserved.
New BEGIN profile validation is synchronized in the bridge and aircraft headers.
An older aircraft still accepts the default zero profile and returns its older
full format; requesting the optional full profile requires updated firmware.

Samples shrink by 59.7% (124 to 50 bytes). The 21:38 flight with 246 samples and
16 diagnostic messages would shrink from 32,296 to 14,092 radio payload bytes,
requiring 294 instead of 673 READ replies. Actual elapsed download time depends
on radio retries. Both radios now use the configurable bandwidth described
above; response pacing and chunk size are unchanged.

The `.flightlog` archive wraps that blob with four bytes `FDL1` and a
little-endian CRC32 of the blob. The decoder checks length, version, layout,
sample count and timestamp order before replaying any file.

## IMU estimator trace

The trace observes the current filter; it does not retune PID gains or change
the Kalman calculation, rejection thresholds, recovery rate, or MPU bandwidth.
Flash the updated flight-controller ELF and run the updated desktop application.
No CubeMX change is required. The radio bridge forwards the same opaque 48-byte
payloads, so this trace feature does not require a bridge firmware change.

After a test, DISARM and download **Full precision + sensor traces**. In Flight Logs:

- **Overview -> Roll estimator / Pitch estimator** compares gyro prediction,
  final angle and the accelerometer-derived angle.
- **Angle corrections** shows the signed correction applied on each IMU update.
- **Acceleration**, **Gyro bias**, and **Loop timing** show acceleration in g,
  residual Kalman bias, and effective IMU dt alongside control-loop dt.
- **Samples -> IMU estimator trace** shows the individual values and decoded
  rejection/recovery status. **Recorded settings** includes cutoff, rejection
  setting, ranges and estimator source signature. JSON and CSV retain the trace.

Every successful `MPU6050_ReadAll` captures one trace from that exact update.
Acquisition caches it alongside the resulting angles/rates, and flight control
passes that same snapshot to the recorder. There is no extra sensor read and no
new serial/radio streaming while flying. The recorder still retains at most
one regular sample per 10 ms; at the observed 15 ms loop period it retains each
control update. Forced start/stop events can repeat an IMU sequence number;
use that number to avoid treating the same sensor sample as a new measurement.
A failed MPU read invalidates the acquisition trace; old or compact recordings
show trace unavailable rather than fabricated zeros.

The 68-byte extension starts at offset 124 of a format-5, format-6 or format-7 record:

| Trace offset | Type | Meaning |
| --- | --- | --- |
| 0, 4 | uint32 | Successful IMU read sequence, sensor-read boot timestamp in ms |
| 8 | float32 | Effective IMU timestep in ms, after existing 1/100 ms clamps |
| 12, 16, 20 | float32 | MPU acceleration X, Y, Z in g |
| 24..43 | 5 float32 | Roll previous angle, gyro prediction, accelerometer angle, bias used, applied correction |
| 44..63 | 5 float32 | Same five values for pitch |
| 64 | uint8 | Trace valid (0 or 1) |
| 65, 66 | uint8 | Roll and pitch estimator flags |
| 67 | uint8 | Timing/clipping flags |

For each axis, `prediction = previous + (gyro - bias) * imu_dt_seconds`.
The recorded bias is the residual Kalman bias used for that prediction; the
logged gyro already has startup calibration subtracted. `correction` is the
actual added angle correction, not an offline difference of rounded angles.
`final = prediction + correction`, allowing for +/-180-degree normalization
where wrapping is enabled. Recovery resets the bias before its prediction;
that actual zero bias and the state-reset flag are recorded. Legacy angle
resets are marked separately when rejection is compiled out.

Estimator flag bits are correction applied=1, magnitude rejected=2, innovation
rejected=4, recovery waiting=8, recovery applied=16, filter state reset=32,
angle wrapped=64, legacy angle reset=128. These are independent flags, not an
exclusive state enumeration. Timing/clipping bits are zero dt clamped=1,
dt over 100 ms clamped=2, raw gyro at a signed-int16 rail=4, raw acceleration at
a signed-int16 rail=8. A rail flag is evidence of reaching a digital limit;
its absence cannot rule out physical sensor distortion or missed vibration.

CMake generates an eight-hex-digit signature from `MPU6050.c`, `MPU6050.h`,
`AttitudeTrace.h` and `SensorConfig.h`, including local uncommitted edits.
It identifies estimator source content, not the entire firmware binary.
Actual compiled cutoff/rejection settings are recorded separately. A zero
signature means unavailable, as in the standalone host-test fixture.

For this drone's installation, usable SRF05 reports start at approximately
8 cm above ground. Before altitude reporting starts in an attempt, interpret
its absence as the drone not having reached 8 cm; do not diagnose a sensor
failure from that absence alone. Losing reports after reporting starts is a
separate case. Never use invalid cached altitude/Vz as physical measurements.

## Gyro-based roll/pitch damping and SRF05 freshness

Roll and pitch now calculate `D = -Kd * gyro_rate_dps`. The rates are the same
axis-mapped, startup-calibrated, MPU-filtered values recorded in each sample.
An accelerometer correction to the fused angle therefore cannot create a false
rotation in D. D applies on the first control update too. P still uses target
minus fused angle; gains, integral behavior, yaw PID and altitude PID are unchanged.
No additional sensor read or software filter is introduced. Header flag bit 2
identifies this behavior, including in compact logs. Older recordings without
the flag show damping mode unknown; the estimator source signature alone does
not identify controller changes.

The sensor validity window is unchanged: altitude becomes unavailable after
more than 300 ms without an accepted echo. The flight response now distinguishes
unconfirmed takeoff from airborne flight, as described below. A near-ground
baseline alone does not enable the airborne altitude-loss response.

Download **Full precision + sensor traces** and select **SRF05 freshness** in
Overview. The chart shows the age of the last accepted measurement, independently
of changes in predicted altitude. Selected-sample details show age and the last
completed attempt result. The sensor-trace sample columns, JSON and CSV retain
both timestamps and age. A repeated last-attempt status is not a new echo.
Format 7 also shows the last raw echo distance and its specific filter result in
selected-sample details, sensor-trace columns, JSON and CSV. These belong to the
last successfully read echo timestamp; a later timeout/read error retains that
echo and its filter result. The raw value is the driver distance before tilt
compensation/filtering, not a validated altitude. Driver range/deadline failures
do not supply a new distance; the trace retains the previous successful echo.

The 16-byte SRF05 extension starts at record offset 192. Formats 6 and 7 share
the same size; the version selects the meaning of offsets 8 and 15:

| Trace offset | Type | Meaning |
| --- | --- | --- |
| 0 | uint32 | Last successfully read echo boot timestamp (ms), including filter-rejected echoes |
| 4 | uint32 | Last accepted measurement boot timestamp (ms) |
| 8 | uint32 (v6) / float32 (v7) | v6: accepted-measurement age in ms; v7: last raw echo distance in cm |
| 12 | uint8 | Trace valid (0 or 1) |
| 13 | uint8 | Presence flags: last echo=1, last accepted measurement=2 |
| 14 | uint8 | Last attempt: waiting=0, accepted=1, altitude-filter rejection=2, tilt rejection=3, echo timeout=4, read error=5, start error=6 |
| 15 | uint8 | v6: reserved zero; v7: filter result for the last echo (table below) |

Presence flags distinguish a real timestamp of zero from a measurement that has
never occurred. Age is unavailable until a measurement has been accepted. For
v7 the desktop calculates unsigned `record_timestamp - last_accepted_timestamp`,
including tick wrap and final fault samples; v6 still validates its recorded age.
Older logs do not invent raw distance/filter details. Compact logs omit all traces.

| Filter result | Meaning |
| --- | --- |
| 0 | Invalid filter input (or SRF05 disabled) |
| 1 | Accepted |
| 2 | Rejected due to tilt |
| 3 | Waiting for the first three echoes to establish the median |
| 4 | Duplicate or out-of-order timestamp |
| 5 | Raw echo differs too far from the three-echo median/motion allowance |
| 6 | Echo differs too far from the predicted altitude (innovation gate) |
| 7 | Invalid innovation or covariance |

Format 7 adds **zero bytes of sample RAM**: 512 x 208 = 106,496 bytes, unchanged
from v6. A compile-time assertion enforces the 104 KiB sample budget. The ring
overwrites its oldest record when full; it never grows or allocates a second
flight buffer. The maximum full download remains 108,288 bytes including header,
configuration and all 16 messages; it streams from the frozen ring in 48-byte
payloads. No allocation occurs during capture or download.

The verified Debug linker map uses **116,024 / 131,072 bytes** of main RAM
(88.52%), including the linker's 512-byte heap
and 1,024-byte minimum stack reservation. There are **15,048 bytes** beyond those
linked allocations; the separate 64 KiB CCM region is unused. This is linked
memory accounting, not a runtime worst-case stack-depth measurement. The landing
phase adds 24 bytes of static control state; the sample buffer and maximum
transfer size stay unchanged. Filter thresholds, gains, the 300 ms validity
window and CubeMX settings are unchanged.

## Takeoff grace and gradual altitude-loss landing

The 2026-10-23 12:40 recording stopped after 510 ms with error 9, before any
liftoff confirmation. Brief valid readings near 8 cm created a baseline, and
the previous code treated loss of that baseline as an in-flight fault. The
baseline is now only a reference for the existing further 5 cm / 150 ms rise
confirmation. It no longer makes a pre-liftoff echo gap stop the motors.

The bounded liftoff budget is now at least **4,000 ms** from accepted TAKEOFF,
or longer if the configured ramp needs it, up to the existing 10,000 ms maximum.
Absent/intermittent altitude before confirmed liftoff keeps the bounded ramp
running. Missing altitude resets partial confirmation so it cannot leave the
ramp stuck. Explicit DISARM, invalid attitude/timing, excessive tilt and hardware
failures still take priority over this wait. A liftoff/target timeout starts a
gradual rundown, since an unconfirmed aircraft might nevertheless be airborne.

After liftoff is confirmed, altitude becoming unavailable starts a latched
landing phase. This is an internal phase of TAKEOFF/FLYING, recorded with flag
bit 5 (value 32); it does not change packet layouts, archive sizes or state enum
values. Updated desktop playback displays LANDING and includes a Gradual landing
sample column/filter. The final stopped event clears this active-phase flag.

Landing starts from the **mean of the four actual applied motor commands**,
including any altitude PID contribution/mixer limiting. Collective decreases by
**10 percentage points per second** (`LANDING_THROTTLE_RAMP_DOWN_PER_SECOND`),
using elapsed HAL ticks with unsigned wrap. For example, 70% runs down toward
zero in seven seconds. The maximum motor-output ceiling decreases proportionally
from the previous actual maximum; the configured idle/minimum cannot keep the
motors running indefinitely. Roll/pitch P and gyro D continue stabilizing, and
existing attitude/yaw integrals are held rather than removed abruptly or allowed
to wind up. Altitude PID is disabled; invalid height/Vz are never used for control.

Returning altitude readings cannot restart the climb. A heartbeat timeout or a
bridge-generated link-timeout DISARM also cannot interrupt an already active
landing. An explicit operator DISARM still stops immediately, as do an unusable
IMU, excessive configured tilt, invalid loop timing or motor-output failure.
After the ramp reaches zero, the original altitude-loss/takeoff-timeout error is
reported through the normal fault path, and the recording is frozen. Recovery
then follows the existing disarm/configuration process.

This is a bounded throttle fallback, not measured vertical-speed or touchdown
control: it cannot guarantee a soft landing without usable height feedback. The
rate needs restrained, low-height validation on this aircraft before free flight.
A long descent can overwrite early samples in the existing 512-record ring;
start/end ticks and diagnostic messages still describe the full attempt.

## Verification

The IMU trace tests reconstruct both angle predictions from the recorded gyro,
bias and effective sensor dt, and final angles from prediction plus correction.
They run through the existing normal-motion, rejection, recovery and angle-wrap
scenarios at all four cutoffs, with rejection enabled and disabled. Additional
checks cover sequence/timestamp alignment, timestep clamps, raw sensor rails,
read failures, forwarding the cached trace, ring rollover, compact/full fallback,
and native C-to-desktop format-5/6/7 fixtures. Acquisition/logger integration tests
cover rejected echoes, a single timeout, the 300/301 ms boundary, fault snapshots,
recovery, tick zero and wraparound. Controller tests cover gyro damping sign,
first-update damping and immunity to fused-angle and target steps.
Desktop tests cover old recordings,
missing traces, malformed metadata/values, CSV/JSON/archive round trips, plot and
table selection without motor commands, the older 100,096-byte format-5 blob
(2,086 READ responses), and a maximum 108,288-byte format-6/7 blob
(512 samples plus 16 messages, 2,256 READ responses).

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
