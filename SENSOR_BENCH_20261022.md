# ST-Link sensor bench test — 2026-10-22

The user disconnected motor power and moved the aircraft by hand. Sensor data
was read from the running STM32 through ST-Link/OpenOCD without halting the CPU.
The tests covered upright/still, slow roll, slow pitch, and six approximate
orientations. They establish sensor and calculation behavior without motor
vibration; they do not establish flight stability or absolute angle accuracy.

## Findings and next step

- The corrected MPU startup completed successfully with the intended divider,
  filter and +/-8 g range. Matched raw samples divided by 4096 exactly reproduced
  the cached accelerometer values in every phase.
- During the slow roll/pitch tests, each captured update used 15 ms. Reconstructing
  the gyro prediction and the final angle from its applied correction agreed
  within 0.000004 degrees, consistent with rounding the trace to floats. This
  checks internal calculation consistency, not an independent angle reference.
- Upright gravity magnitude was about 0.883 g. Opposite Z orientations suggest
  approximately -0.132 g of Z offset. The existing stationary startup procedure
  estimates gyro bias and initializes attitude; it does not calibrate the three
  accelerometer offsets and sensitivities.
- The next measurement improvement should be a controlled six-position
  accelerometer calibration, followed by known-angle checks. Retain the +/-8 g
  range and 0.8–1.2 g acceptance window while collecting it. No accelerometer
  calibration coefficients or PID/filter tuning were applied in this test.
- An unexpected ARMED state interrupted the orientation recording. A floating
  USART1 RX input and permissive command handling were investigated and fixed
  as described below. Final checks left the aircraft unconfigured/disarmed,
  ready, with commands enabled, zero motor output and no application/HAL error.

## Still and slow-motion measurements

| Phase | Retained updates | Roll, degrees | Pitch, degrees | Other observations |
| --- | ---: | --- | --- | --- |
| Upright/still, 12 s | 679 | Mean -2.303; standard deviation 0.0135 | Mean -2.319; standard deviation 0.0117 | Acceleration norm mean 0.88294 g; no clipping or rejected corrections |
| Slow roll, 30 s | 1,671 | Range -35.992 to +36.254 | Range -9.660 to +2.665 | Six updates rejected an accelerometer correction; no clipping |
| Slow pitch, 30 s | 1,699 | Range -7.262 to +6.195 | Range -30.780 to +34.796 | One update rejected an accelerometer correction; no clipping |
| Still after UART fix, 10 s | 559 | Mean -3.384; standard deviation 0.0127 | Mean -2.900; standard deviation 0.0150 | Norm mean 0.88302 g; no clipping, rejected corrections or state changes |

The different still angles are not a measured drift: the aircraft was handled
and repositioned without a level reference fixture between these recordings.
For the final still recording, gyro means were roll -0.0031, pitch -0.0036 and
Z +0.0006 degrees/s; gyro standard deviations were 0.024–0.028 degrees/s.

For consecutive captured updates in the respective motion tests, angle-change
rate and gyro rate correlations were 0.979 for roll and 0.980 for pitch. These
signals share the same estimator, so this supports internal consistency rather
than proving gain, timing, alignment or angle accuracy against ground truth.
The logged accelerometer correction and learned gyro bias explain why the final
angle change is not simply the recorded gyro rate multiplied by dt.

## Approximate six-orientation assessment

Use only the first 55 seconds of `04-orientations.jsonl`, before the interrupted
state check. Select low-motion samples with all gyro rates below 5 degrees/s and
the dominant acceleration component above 85% of the measured vector magnitude.
The following are medians in sensor coordinates, not a certified calibration:

| Sensor axis | Positive-face median, g | Negative-face median, g | Approximate offset, g | Approximate sensitivity |
| --- | ---: | ---: | ---: | ---: |
| X | +1.02563 | -0.96240 | +0.03162 | 0.99402 |
| Y | +0.98193 | -1.01880 | -0.01843 | 1.00037 |
| Z | +0.88037 | -1.14502 | -0.13232 | 1.01270 |

For ideal opposing orientations, `offset = (positive + negative) / 2` and
`sensitivity = (positive - negative) / 2`, giving
`corrected_g = (measured_g - offset) / sensitivity`. Hand-held faces were not
exactly aligned, and cross-axis readings demonstrate that limitation. Treat
these numbers as evidence to repeat a controlled calibration, not coefficients
to hardcode. A six-parameter offset/sensitivity calibration is described in
[NXP AN4399](https://www.nxp.com/docs/en/application-note/AN4399.pdf).

Five captured updates hit a raw gyro rail during a fast transition around 25 s;
one matched raw sample showed X = -32768. No acceleration rail was captured.
The slow roll/pitch tests did not clip. Large rotations also cross the current
Euler-angle estimator's wrap/recovery boundaries, so this test does not validate
full three-dimensional attitude tracking or justify changing the gyro range for
normal flight on its own.

SRF05 altitude and vertical-speed validity remained false in the still/slow
roll/pitch recordings. No known-height test was performed. This installation
starts usable SRF05 reporting at approximately 8 cm; invalid cached altitude/Vz
must not be treated as physical height or motion. Heading and absolute barometric
altitude were not calibrated or independently verified.

## Unexpected ARM: UART input and command fixes

The orientation recording stopped when the state guard observed ARMED. The user
confirmed they only moved the aircraft and sent no ARM, TAKEOFF or Space command.
Motor commands were still zero and configuration remained all zero. The UART
buffer was receiving changing garbage while PA10 had no pull resistor enabled.
Temporarily applying its internal pull-up stopped those changes; removing it
reproduced them. The exact bytes that triggered ARM were not retained, but the
observed noise and previous parser allowed this sequence: a DISARM could change
INIT to CONFIGURED, a command-like ARM prefix could pass without valid
configuration, and command-like noise could refresh the link timeout.

The user changed PA10 / USART1_RX to Pull-up in CubeMX and regenerated the code.
The regenerated source and the running GPIO pull register were both verified.
No generated initialization or `.ioc` file was edited by the agent.

The firmware now also:

- Checks the complete 64-byte command shape, including reserved zero bytes,
  opcode and DISARM reason, before accepting it or refreshing the timeout.
- Preserves INIT when DISARM arrives before configuration.
- Requires valid configuration before ARM.
- Uses the same command-shape validator in the flight and radio-controller
  projects. Existing desktop packets and the 64-byte wire layout are unchanged.

CubeMX regeneration disabled the watchdog HAL declaration. The user-owned root
`CMakeLists.txt` now enables that HAL module for the application and drivers,
preserving the existing watchdog startup after calibration.

The rebuilt flight firmware was flashed and verified on the attached board.
Live checks confirmed PA10 pull-up mode 1 and a stable UART receive buffer.
Replaying a malformed ARM-prefix frame left INIT and motor outputs unchanged;
a valid DISARM before configuration also preserved INIT. No valid ARM or TAKEOFF
was sent by the debugger. The final 10-second still capture and a later status
check both retained zero outputs and no error. The radio-controller firmware
built successfully but was not flashed during this session.

Native host regressions passed, including command-shape tests against both
firmware headers. All 83 desktop packet/log tests passed. Full command and
heartbeat frames still have no CRC; complete-frame validation rejects the
observed garbage pattern but is not a general integrity guarantee.

## Evidence and capture limits

Artifacts are retained locally under `build/stlink-debug-20261022/` (ignored by
Git), including the capture and analysis scripts, flash/RAM snapshots and:

- `live/01-flat-still.jsonl` through `live/05-flat-after-fix.jsonl`.
- Per-phase `.summary.json` files, except the interrupted orientation recording.
- `live/bench-analysis.json`, `post-fix-command-checks.json` and
  `final-status.json`.
- `live/interrupted-orientation-sram.bin`.

The sampler accepts two identical consecutive reads of cached flight data and
uses only new IMU sequence IDs. Separate raw values are used only when their
sequence matches. PC read timing missed about 14–17% of IMU updates; sequence
gaps are recorded, and missing samples are not interpolated. Consequently,
observed peak and rejection/clipping counts can undercount unsampled events.
All retained updates in the still, slow-motion and first-55-second orientation
datasets used 15 ms. This was a motor-power-disconnected bench test, without a
power-cycle, precision angle fixture or subsequent free flight.
