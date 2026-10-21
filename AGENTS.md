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
