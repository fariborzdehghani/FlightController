# Automatic takeoff and hover throttle

Hover throttle is learned by the aircraft. The desktop configuration page has
no hover-throttle field or estimate label. The estimate remains in flight logs. Set arm
throttle, maximum throttle, takeoff ramp and target altitude as usual, then ARM
and TAKEOFF. The maximum throttle also bounds the initial liftoff search.

## Flight behavior

- The initial ramp starts at arm throttle. No usable SRF05 height below this
  aircraft's approximately 8 cm reporting threshold is expected. The existing
  time limits and maximum throttle still bound this phase.
- Liftoff requires a measured 5 cm rise from the first usable baseline (or
  height already at the goal), with three accepted echoes spanning 150 ms.
  Height loss after initial reporting pauses the ramp for guarded recovery.
- At confirmation, the average applied motor command seeds the hover estimate.
  The altitude target moves toward the selected height at 20 cm/s. Altitude
  PID supplies climb and braking corrections; there is no further blind ramp
  toward a guessed hover value. Actual motion depends on control tuning.
- Once the final target is held, learning requires one second of distinct,
  fresh accepted echoes, valid vertical speed within 0.10 m/s, roll and pitch
  within 5 degrees, low angular rates and unsaturated PID/mixer output.
  A small steady height error is allowed so it can be trimmed even with zero
  altitude Ki. This is a provisional estimate, not proof of equilibrium.
- The baseline slowly follows average applied collective with a 5 second
  time constant and a maximum change of 1 throttle percentage point/s. Where
  altitude I is available, the change is subtracted from I to avoid counting
  the same correction twice. With zero I or a clamped I term, any remaining
  collective change is still limited by that learning rate.
- Learning pauses during motion, tilt, missing/invalid measurements,
  saturation, recovery or landing. A stopped flight retains its last estimate
  for logging; a new start/takeoff learns again. Nothing is written to flash.

The existing SRF05 automatic-landing option retains its meaning. Disabling it
can leave the aircraft searching up to maximum throttle without height feedback;
this option cannot provide automatic altitude hold when height is unavailable.
LAND, DISARM, heartbeat and maximum-angle behavior remain in place.

This feature controls height and attitude. Horizontal position hold requires
additional motion/position sensing and control. Reliable height data and suitable
PID tuning remain necessary; software tests do not establish physical stability.

## Commands and recorded data

All three projects keep the 64-byte command layout. Desktop configuration byte
43 is always zero, including when loading a saved legacy hover value. Both
firmware validators accept zero and bounded old values between arm and maximum
throttle; the aircraft ignores the old value. No bytes or offsets shift.

Log header flag bit 3 (`0x0008`) identifies automatic hover learning. The existing
recorded configuration byte 44 carries the current/last estimate in half-percent
units. Live frames refresh it each sample; onboard recordings refresh it while
recording and freeze it on stop. The full-precision `base_throttle` trace shows
the baseline during altitude control. It remains ramp throttle before liftoff
and landing collective during landing. Record sizes and stream rate are unchanged.
Old archives retain their legacy configured/effective-throttle interpretation.

## Verification

Before removal of the test folders at the owner's request, host checks exercised
controller, sensor, recorder and shared protocol regressions with
AddressSanitizer/UndefinedBehaviorSanitizer.
The 21 controller groups include positive/negative I transfer, zero-Ki learning,
invalid speed, duplicate/stale echoes, tilt, saturation, landing and reset.

A limited closed-loop vertical model covers two unknown hover requirements,
zero/nonzero Ki, tick rollover and a change in required thrust. It includes
nonlinear thrust, gravity, drag and ground contact, with 60 ms observations.
It does not model real sensor errors, full aircraft dynamics or hardware timing.
Desktop checks covered automatic packets, old settings/logs, absence of hover controls and the
native firmware's new live frame through serial recording/archive round trips.

Build and deploy matching aircraft, radio bridge and desktop versions. No CubeMX
configuration change is needed. Controlled hardware validation remains necessary
before treating automatic takeoff or hover as flight-proven.
