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

