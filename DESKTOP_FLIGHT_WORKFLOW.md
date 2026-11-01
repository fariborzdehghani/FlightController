# Desktop camera, serial recording and altitude options

The header throttle buttons now support live manual throttle while armed or flying,
with Up/Down shortcuts. See [live throttle](LIVE_THROTTLE.md) for takeover behavior
and the synchronized desktop and radio changes.

Hover throttle is now automatic; see [automatic hover](AUTOMATIC_HOVER.md).
The desktop configuration page omits hover-throttle entry and its estimate label. Maximum
throttle bounds the liftoff search. Old hover values are ignored by new commands
and firmware; existing logs remain readable.

All three desktop 3D views (Main, Canvas, and Flight Logs) support left-drag
to pan, right-drag to orbit, and the mouse wheel to zoom. Their cameras are
saved independently on application close in
`%LOCALAPPDATA%\DroneLogger\view-state.json`. Window bounds, graph-panel layout,
configuration fields, selected tabs and the last reviewed file/time are restored
on the next launch. Restoring the interface does not connect or arm the aircraft.

With the aircraft's Logger Port connected, ARM starts desktop serial capture,
including when Space sends ARM before TAKEOFF. Repeated ARM does not split an
active recording. DISARM ends and saves it automatically under
`Documents\DroneLogger\FlightLogs`. Closing the app or losing the serial
connection saves an interrupted recording. After a normal DISARM the recording
is loaded in Flight Logs for charts, sample inspection, playback and CSV export.
Use Open flight log to reopen any saved `.flightlog` file.

These desktop captures retain the incoming live stream for the whole armed
session; they do not depend on the onboard sample ring or downloading it. They
use an `SFL1` header, CRC32 and JSON payload, and coexist with existing `FDL1`
onboard downloads. Serial capture includes received messages, source boot ticks
when supplied, altitude validity and desktop arrival time relative to ARM.
Updated firmware automatically sends full-precision serial samples with both
IMU and SRF05 traces. These preserve the same 32-bit floating-point values and
trace fields as a full onboard download, at the existing live rate of at most
10 samples/second (one snapshot per 100 ms; typically about 106 ms between
records). This rate is lower than the onboard recorder's 10 ms minimum interval;
the stream does not include every intervening IMU read or SRF05 attempt.

Each `LiveLog=` line contains a Base64-encoded v7 header, the actual applied
configuration, one 208-byte full-precision record, and a CRC16. The 338-byte
packet becomes 452 Base64 characters, or 461 bytes including marker/newline.
At most 4,610 bytes/second uses about 40% of 115200 baud with 8N1 framing.
The existing interrupt-driven transmit queue remains nonblocking, and the
onboard ring/download contents are independent of live-frame encoding.

Serial recordings include raw SRF05 distance, echo/acceptance timestamps,
filter rejection reason and attempt result, plus IMU predictions, corrections,
acceleration, bias and diagnostic flags. Flight Logs recognizes these captures
as **full precision + sensor traces** and preserves the device's estimator
settings in the archive. Invalid height/velocity remain invalid. Damaged frames
are discarded and an error is retained in the desktop recording's messages.

The desktop still reads old `Data=` JSON senders and earlier serial files;
those records cannot gain traces that were never captured. A session with no
received telemetry is saved with its ARM/end messages but has no playback.
Flash updated aircraft firmware and restart the updated desktop to use full
serial traces. No new command fields, radio-bridge update or CubeMX changes
are required for this serial-stream addition.

In the header, **SRF05 automatic landing** defaults to checked. Change it
while disarmed and press **SET CONFIG** to apply it to the aircraft. Disabling
it bypasses liftoff timeout, takeoff-target timeout and automatic altitude-loss
landing. The initial ramp remains bounded by the existing takeoff throttle
ceiling. If altitude is lost after altitude control starts, the controller
holds the applied collective and stops using altitude PID; attitude control
continues. LAND, DISARM, maximum-angle protection, motor limits and heartbeat
protection remain active. This mode cannot guarantee an altitude hold.

Configuration byte 46 is now a boolean opt-out: `0` keeps automatic landing
enabled (including legacy zero-filled packets); `1` disables it. Bytes 44/45
remain zero in commands and carry metadata only in recorded configurations.
Bytes 47–63 remain reserved and zero. Both firmware validators and the desktop
builder/decoder share this unchanged 64-byte packet layout. Update the aircraft,
radio bridge and desktop together to use the switch.

BMP280 initialization/read failures no longer stop the aircraft. Failed reads
mark barometer altitude unavailable and report once per outage; finite reads
resume normal reporting. Failed initialization leaves barometer reads disabled
until initialization succeeds after a restart. SRF05 altitude and the IMU remain
independent of the barometer. Missing SRF05 reports before the aircraft reaches
its approximately 8 cm usable-height threshold are expected near the ground.

Validation covers desktop packet/recording/review/camera tests, firmware host
regressions with address/undefined-behavior sanitizers, and both firmware builds.
No CubeMX changes are required. Hardware flashing and physical flight checks
are separate from these software checks.
