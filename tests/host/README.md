# Flight-control, SRF05 and flight-recorder regression tests

Run from the repository root on Linux, or in Ubuntu WSL on Windows:

```sh
sh tests/host/run.sh
```

```powershell
wsl -d Ubuntu -- sh tests/host/run.sh
```

Requires GCC with AddressSanitizer and UndefinedBehaviorSanitizer. The runner
compiles the production FlightControl, PID, CoreHelpers, AppError, SRF05,
VelocityEstimator and FlightRecorder sources. Sensor input, clock, timer/GPIO
hardware, motor hardware, and logging are stubbed. Outputs go
under the ignored `build/host-tests` directory.

The eight takeoff regression groups cover bounded motor output with and without altitude,
failure to lift, delayed altitude acquisition, transient height changes,
continuous altitude-controller engagement, target slew and derivative damping,
settling before hover, the 300 cm target budget, sensor loss, disarm/reset,
back-to-back arm/takeoff, tick wraparound, and forwarding roll/pitch gyro rates
to the recorder input. Recorder tests also cover format 3's 124-byte records,
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

The five altitude-filter groups exercise the production estimator with isolated
and repeated false echoes (8, 50, 150, 300, and 395 cm against a 100 cm reference),
NaN/infinite input, bursts of 1 through 100 samples, dropouts and recovery,
spikes during initial tracking, duplicate/old timestamps, and tick rollover.
A simulated 20 cm/s climb includes periodic false echoes and checks that the
altitude error stays below 8 cm. These scenarios use a level, stationary IMU
input to isolate SRF05 behavior; they are not a calibrated flight model.

These are deterministic controller tests, not a physical aircraft simulation.
They do not exercise the complete radio/command loop or replace flight tuning.

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
