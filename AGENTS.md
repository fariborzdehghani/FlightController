# Flight-controller project context

This firmware is one part of a three-project system:

- Flight controller firmware (this repository):
  `D:\Electronic Projects\Flight Controller\06-Program\FlightController`
- Windows desktop controller/logger:
  `D:\Electronic Projects\Flight Controller\07-Desktop Apps\DroneLogger\DroneLogger`
- STM32 radio bridge/controller firmware:
  `D:\Electronic Projects\Flight Controller\06-Program\RemoteController`

The three projects share the same fixed 64-byte command protocol. When a
configuration or control-packet field changes, inspect and keep all three
projects synchronized, including desktop packet tests and radio validation.

## Project goals and code style

- This is a hobby and learning project for building a simple flight controller.
- Keep implementations simple, readable, and easy to understand when modifying
  existing code or adding features to the firmware or companion applications.
- Avoid unnecessary abstractions, architectural complexity, and dependencies.
  Prefer straightforward solutions, while preserving correctness and safety.

## CubeMX configuration changes

- Do not edit `.ioc` files directly, including through scripts or tools.
- When a task requires a CubeMX configuration change, tell the user exactly
  which settings to change in the CubeMX UI and ask them to make the change
  and regenerate the code themselves.
- Wait for the user to confirm that the required configuration change and
  regeneration are complete before relying on the new configuration. Continue
  independent work while waiting.
- Keep custom firmware logic in separate modules or appropriate `USER CODE`
  blocks so it survives regeneration. Do not bypass this rule by changing
  generated initialization code outside `USER CODE` blocks as a substitute
  for the required CubeMX configuration change.

## Flight-log interpretation: SRF05 near-ground threshold

- The user has established that, on this drone, the SRF05 only starts reporting
  usable altitude once the drone reaches approximately **8 cm above the ground**.
- During a takeoff attempt, if altitude reporting has not started, interpret
  this as the drone not yet having reached that 8 cm threshold. Missing initial
  reports, invalid-altitude flags, or an SRF05-unavailable message alone must
  not be diagnosed as a sensor failure in this situation.
- Distinguish this expected absence of initial reports from losing usable
  altitude after reporting has already started; investigate that later loss
  separately.
- Invalid altitude and vertical-speed values can be stale cached numbers.
  Do not interpret them as physical height or motion while their validity
  flags are false.
