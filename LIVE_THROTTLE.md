# Live throttle buttons and keyboard shortcuts

The header's **THROTTLE + (Up)** and **THROTTLE - (Down)** change throttle by
one percentage point per click or key press. Up/Down work on every tab, including
with a text field focused while armed. Held-key repeats are ignored, and a pending
adjustment cannot build a queue of additional adjustments.

While disarmed, these controls retain their existing behavior: change the arm
throttle setting and send configuration. Text fields, combo boxes and sliders
keep normal arrow-key behavior while disarmed.

While armed, taking off or flying, they send dedicated live commands without
changing saved configuration. The first adjustment switches to manual collective
throttle, starting from the average applied motor output, including altitude PID
correction and mixer limits. Automatic takeoff ramp, altitude hold and hover
learning stop. Each subsequent adjustment changes the held collective within the
configured minimum/maximum. Attitude stabilization and maximum-angle protection
remain active, including when manual control starts from armed idle.

The takeoff button becomes **STOP (Space)**. Space/DISARM stops the motors; L/LAND
starts the existing landing procedure. Throttle adjustments are rejected during
landing and while disarmed. The desktop can still edit arm throttle while disarmed.
DISARM and connection loss clear the desktop's manual state; a new firmware ARM
starts at configured arm throttle. DISARM before starting automatic takeoff again.

The heartbeat failsafe and existing SRF05 automatic-landing setting remain active.
In manual mode an initial absence of altitude reports near the ground does not
start an automatic takeoff timeout. Losing usable altitude after reporting has
started still initiates landing when the SRF05 failsafe is enabled. Manual throttle
does not hold an altitude.

## Protocol and companion changes

Packets remain 64 bytes. Control type 2 gains command 4 (throttle up) and command 5
(throttle down); all bytes 2-63 remain zero. Configuration is unchanged. Both shape
validators and the radio bridge's control-command validator accept the new commands.
Older firmware rejects them, so update the desktop and both firmware projects.

The desktop and radio updates have been applied to the original repositories.
The desktop toolbar wraps its flight buttons at smaller window widths to keep
the throttle controls accessible. Build the three projects from their own
repositories; no companion patch or apply script is needed. Obsolete patch
packages, staging copies, backups and test folders have been removed.

## Validation

The checks below were run before the test folders were removed at the owner's
request. They describe prior validation; this checkout no longer includes those
test suites.

The firmware host regressions cover manual takeover during armed idle and takeoff,
applied-collective continuity, successive adjustments, configured limits, active
attitude correction, no automatic ramp/height PID after takeover, landing interlocks,
altitude loss after reporting starts, failsafe opt-out, and restart reset. The shared
command tests run against both updated protocol headers. Desktop tests cover packet
shape, buttons/keyboard with editor event routing, armed and takeoff states, pending
command coalescing, landing and Space/DISARM.

Verified on 2026-11-01:

- Desktop and test assemblies build with zero warnings/errors.
- All 156 desktop cases pass through the normal VSTest runner, including packet
  builders, live buttons/shortcuts, landing/start races, serial capture and log review.
- All 24 firmware takeoff/landing/hover/manual-throttle host groups pass with MSVC.
- Command shape/noise tests pass against both updated protocol headers.

Both firmware projects build successfully with the STM32 GCC toolchain using
`cmake --build --preset Debug`. No hardware was flashed or flight-tested.
No CubeMX changes are required. Restart the desktop app and flash the updated
firmware to both boards before using live throttle over the radio bridge.
