# Flight-control, MPU6050, SRF05 and flight-recorder regression tests

Run from the repository root on Linux, or in Ubuntu WSL on Windows:

```sh
sh tests/host/run.sh
```

```powershell
wsl -d Ubuntu -- sh tests/host/run.sh
```

Requires GCC with AddressSanitizer and UndefinedBehaviorSanitizer. The runner
compiles the production FlightControl, PID, CoreHelpers, AppError, MPU6050, SRF05,
VelocityEstimator and FlightRecorder sources. The freshness integration test
also compiles SensorAcquisition and Logger. Sensor input, clock, timer/GPIO
hardware and motor hardware are stubbed. Outputs go
under the ignored `build/host-tests` directory.

The eleven takeoff/landing regression groups cover bounded motor output with and without altitude,
failure to lift, delayed altitude acquisition, transient height changes,
continuous altitude-controller engagement, target slew and derivative damping,
settling before hover, the 300 cm target budget, sensor loss, disarm/reset,
back-to-back arm/takeoff, tick wraparound, and forwarding roll/pitch gyro rates
to the recorder input. Gyro damping checks cover the first update, correct sign,
fused-angle correction and target steps, timestep independence, and invalid rates.
Landing checks reproduce the brief near-ground baseline loss, verify the
4,000 ms minimum wait with and without initial altitude, delayed acquisition,
partial-confirmation reset, continuity from applied collective and existing
integral, the 10 percentage-point/s rundown, retained attitude P/D, held I,
no automatic climb on sensor recovery, zero output below configured idle,
bounded completion, tick wrap, immediate DISARM and hard-fault overrides.
The controller tests stub sensors and motors; they do not simulate touchdown
or prove a soft physical landing. Core's bridge/heartbeat guards are compiled
in the firmware build; these host tests do not execute the full command loop.
Recorder tests also cover format 7's unchanged 208-byte records,
10 ms minimum interval, 512-record ring rollover, and disarmed diagnostic messages.

The ten SRF05 regression groups cover on-time and late completed echoes, missing
edges, deadline boundaries, overruns on both edges and all four channels,
overrun recovery, preservation of unrelated flags, counter rollover with pending
interrupts, tick wrap, trigger spacing, cancellation, range rejection, and start
failure, invalid configuration, full-width 32-bit timer rollover, and capture
replacement after HAL clears CCxIF. A sweep exercises 3,312 combinations of
timer phase, pulse width, interrupt delay, and pending/serviced update ordering.
They check shared-channel preservation against a model of the HAL stop
behavior; they do not run the MCU's actual interrupt hardware or digital filter.

The six altitude-filter groups exercise the production estimator with isolated
and repeated false echoes (8, 50, 150, 300, and 395 cm against a 100 cm reference),
NaN/infinite input, bursts of 1 through 100 samples, dropouts and recovery,
spikes during initial tracking, duplicate/old timestamps, and tick rollover.
A simulated 20 cm/s climb includes periodic false echoes and checks that the
altitude error stays below 8 cm. These scenarios use a level, stationary IMU
input to isolate SRF05 behavior; they are not a calibrated flight model.

These are deterministic controller tests, not a physical aircraft simulation.
They do not exercise the complete radio/command loop or replace flight tuning.

## Complete command-frame validation

`command_protocol_test.c` runs against both firmware projects' identical
`CommandProtocol.h` files. It covers exact 64-byte length, valid control opcodes
and DISARM reasons, every reserved control/heartbeat byte, reserved configuration
fields, CRC-protected log requests, and captured UART noise given command-like
prefixes. Existing desktop builders already zero these reserved fields; the
wire format is unchanged. The attached board additionally verified rejection
of a malformed ARM-prefix frame and preservation of INIT on DISARM before
configuration. See `SENSOR_BENCH_20261022.md` for the PA10 pull-up diagnosis and
post-flash checks. These unit tests do not exercise `Core.c` state transitions
or establish electrical immunity to UART noise.

## MPU6050 accelerometer rejection and bandwidth

The accelerometer is configured for +/-8 g (`AFS_SEL=2`, register value `0x10`)
with 4096 counts per g. `SensorConfig.h` keeps the fixed register value, scale
and recorded range together. This adds measurement headroom without changing
the filter's acceptance limits or PID gains. MPU tests verify calibration at
1 g, all-axis conversion to g and m/s^2, unclipped peaks beyond +/-2 g, and
clipping flags at the +/-8 g signed-int16 rails. The recorder test verifies
that the log header advertises 8 g; the desktop retains support for old 2 g logs.
Startup tests reproduce the attached module ignoring configuration writes
while asleep: reset leaves sleep enabled, and wake-up must precede divider,
filter and range writes. Tests also simulate acknowledged writes that did not
take effect, a failed configuration readback, and calibration timeouts caused separately
by acceleration magnitude and gyro motion. The driver must stay unready and
report the actual failure; it does not guess a scale or widen acceptance.

`Core/Inc/SensorConfig.h` also contains two build-time filter settings. Rebuild
and flash after changing them; both accept compiler `-D` overrides:

- `MPU6050_ACCEL_REJECTION_ENABLED=1u` enables rejection by default. Set it to
  `0u` to restore the original correction and angle-wrap behavior.
- `MPU6050_DLPF_HZ=20u` sets the default gyro cutoff. Supported values are
  `5`, `10`, `20`, and `42` Hz, corresponding to accelerometer cutoffs of
  `5`, `10`, `21`, and `44` Hz. Lower values smooth more but add delay.
  The sensor sample rate remains 125 Hz for every supported value.

The rejection path accepts accelerometer magnitude from 0.8 to 1.2 g and,
independently for each angle, an accelerometer direction within 15 degrees of
the gyro prediction during normal tracking. These initial limits are defined in
`MPU6050.c` and need checking against measured vibration. Rejected corrections
still propagate the angle and covariance; they do not learn gyro bias. The
residual Kalman bias is bounded to +/-3 degrees/second in the rejection path.
Angles are compared across the shortest +/-180-degree distance, so a wrap cannot
bypass rejection. Plausible normal corrections resume on the next sample.

A larger angle error no longer locks the filter out permanently. Recovery needs
one continuous second with magnitude within 0.15 g of gravity, all three gyro
rates within +/-3 degrees/second, and the accelerometer angle within 2 degrees
of its fixed reference. Each angle then moves toward the gravity reading at a
maximum correction rate of 30 degrees/second. The untrusted learned bias is
cleared at recovery entry; angle/bias covariance is reset at entry and completion.
Recovery continues through the 15-degree boundary until it reaches the gravity
angle. Invalid magnitude, excessive gyro rate or unstable direction cancels it.

These checks target brief disturbances. Noise or sustained acceleration inside
the limits can pass, and gyro drift can accumulate during prolonged rejection.
Low gyro rates and stable acceleration are recovery evidence, not proof that a
vehicle has no linear acceleration; a sustained false gravity direction can
still be trusted. Recovery waits while its conditions are not satisfied. This
small change does not replace a full 3D attitude estimator or mechanical
vibration reduction. The 20 Hz default
reduces delay; use 10 Hz for stronger vibration filtering if needed. No CubeMX
changes are needed. Full downloads now use archive format 7 for the diagnostic
trace; the 64-byte command and radio transport remain version 1.

The trace captures the gyro prediction, applied correction, accelerometer XYZ,
accelerometer-derived angles, bias used by each prediction, IMU dt, sample ID,
and rejection/recovery/wrap/timing/clipping flags. MPU tests reconstruct the
predicted and final angles on every simulated update, including recovery and
wrap scenarios, without changing the existing expected filter outputs. Download
**Full precision + sensor traces** to retain it; compact format 4 omits trace data.
See `POST_FLIGHT_LOGGING.md` for byte offsets and interpretation.

The SRF05 freshness test exercises production acquisition, velocity estimation,
logger snapshots and recorder serialization together. It distinguishes successful
raw echoes from accepted measurements, tolerates one missed response, verifies
the existing 300/301 ms validity boundary, and preserves the timestamp and age in
the final fault sample. It also covers recovery, stop/reset, tick zero and wrap.
Format 7 additionally checks raw echo distance and separate median-warmup,
timestamp, median-motion, innovation and tilt results. Timeouts retain the
last echo distance/result and the final fault sample preserves them. The native
recorder generates the desktop v7 fixture without overwriting the v6 fixture.
Compile-time record-size/budget guards and recorder tests retain 512 x 208 bytes
(106,496 bytes); v7 adds no sample RAM or download bytes. This instrumentation
does not extend the altitude-loss timeout.

The MPU tests run the production I2C driver and filter with simulated register
data, with rejection both enabled and disabled at all four bandwidths. They
cover startup calibration, unchanged normal tracking, freefall and excessive
acceleration, brief false directions at 1 g, gyro prediction through rejected
bursts, recovery from 24/36-degree lockout and errors near -146/-178 degrees,
tilted and +/-180-degree references, interrupted recovery windows, third-axis
motion, bias bounding, valid +/-180 crossings, invalid crossings that used to
force an angle reset, tick rollover, and read-error propagation. They verify software
behavior, not the physical MPU's noise filtering or aircraft stability.

## SRF05 input validation

Completed echoes are checked against their trigger-to-completion deadline, even
if foreground polling was delayed. A timely echo can still be read later; the
acquisition layer uses its timestamp to decide whether it is stale. Capture
overrun (`CCxOF`) invalidates the measurement, and the next trigger clears the old
flags for that channel. Error returns leave output arguments unchanged.

Capture timestamps account for both pending updates and updates serviced before
a pending capture callback. The input channel is briefly disabled while reading
its capture register and changing polarity. The shared PWM counter is never
reset. The 10 us trigger/setup is protected from interrupts; subsequent triggers
wait 51 HAL ticks to guarantee the minimum 50 ms quiet time despite tick rounding.
These timing requirements come from the
[manufacturer's SRF05 specification](https://www.robot-electronics.co.uk/htm/srf05tech.htm).

The driver requires an up-counting timer with fixed ARR/PSC, direct input capture
without input prescaling, and exclusive ownership of its capture channel and
update interrupt. Interrupt service must be faster than the shortest accepted
echo (464 us at 8 cm); update events must not be lost across whole timer periods.
The blocking API also requires an advancing HAL tick and an enabled timer IRQ.

TIM3 CH3 uses input-filter setting 8 in both `main.c` and `FlightController.ioc`.
This requires six samples at fDTS/8, as specified in
[ST's RM0090 reference manual](https://www.st.com.cn/resource/en/reference_manual/dm00031020-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf).
Its electrical-noise performance still needs verification on the board.

## Confidence against false altitude jumps

Repeated rejected echoes no longer reset the estimator to the rejected distance.
After more than 300 ms without an accepted sample, altitude and vertical velocity
become invalid while the last tracking reference is retained. Plausible readings
can restore tracking; an explicit vertical reset during stop/start allows a new
reference. This deliberately favors rejecting an unexpected surface change over
silently treating it as aircraft motion. The existing flight-control response to
sustained altitude loss after acquisition is an emergency motor stop.

During initial tracking, the existing 8 cm innovation threshold remains fixed
until eight inliers have established tracking; large initial covariance cannot
admit large jumps. Established tracking uses the existing covariance-based gate.
Duplicate/out-of-order samples cannot build confidence. A long raw-sample gap
also clears the three-sample history without discarding the tracking reference.

Passing these tests provides evidence for the specified software fault cases,
not a probability that a physical sensor will never be wrong. Small errors inside
the acceptance gates, a consistently wrong initial echo, gradual drift, and
reflections that resemble real motion can still pass. No single SRF05 can identify
all such cases. Physical confidence requires controlled measurements at known
heights and over representative surfaces, including vibration/motor interference,
tilt, missing echoes, and transitions out of range. Compare raw accepted pulse
width/distance, filtered altitude, validity, and actual height in recorded traces;
check trigger/echo timing and unchanged motor PWM with a scope or logic analyzer.
This review has not performed those board measurements.

## Takeoff settings

The packet remains 64 bytes with the same fields. Use measured `hoverThrottle`
when available; the old zero/midpoint fallback is an estimate, not calibration.
The initial motor ceiling is `min(maxThrottle, hoverThrottle + 5)`.
Liftoff needs a 5 cm rise sustained for 150 ms. Initial operation without usable
altitude shares the liftoff deadline: ramp duration to the ceiling plus 1 second,
capped at 10 seconds. Slow ramps can time out before reaching the ceiling.

After liftoff, the altitude target moves at 20 cm/s. Actual climb speed depends
on tuning. Hover is declared after 500 ms within 5 cm of the final target, with
vertical speed at most 0.15 m/s when the velocity estimate is valid. These policy
constants live in `Core/Inc/Core.h`.

The target remains height above the surface seen by the SRF05, including sensor
mounting height, rather than a distance to climb relative to the launch point.
The existing brief-dropout tolerance stays in sensor acquisition; sustained
altitude loss after acquisition is an emergency motor stop, not a landing.
