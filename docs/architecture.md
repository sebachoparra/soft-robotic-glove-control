# System Architecture

## Overview

This project migrates a validated standalone RP2040 ("Pico") soft-glove
pneumatic controller into a ROS 2 Jazzy system, while preserving the
original controller's numerical behavior. See the project's governance
policy for the authoritative statement of migration scope and
constraints; this document summarizes the resulting code layout.

## Module relationships

```
reference/rp2040_validated/   Frozen, standalone RP2040 firmware (golden
                               reference, tag rp2040-golden-v1). Runs the
                               full control stack (UKF4, gain scheduling,
                               PI/LADRC position control, pressure PI,
                               FILL/HOLD/VENT) on-device with no ROS 2
                               involvement. NEVER modified.

pico_firmware/                 Current, ROS 2-integrated Pico firmware.
                               Handles low-level sensor acquisition (ADC,
                               pressure, flex), actuator/valve GPIO,
                               precise pulse timing, the serial wire
                               protocol to the host, and a local
                               communication watchdog / safety latch. Per
                               the project's migration policy, the Pico must
                               not become the high-level controller.

ros2_ws/src/                   ROS 2 Jazzy packages implementing the
                               high-level control stack that used to run
                               entirely on the RP2040 in
                               reference/rp2040_validated/:
                                 - pico_bridge      serial <-> ROS 2 bridge
                                 - glove_estimator   UKF state estimation
                                 - glove_controller  position/pressure
                                                     control, mode logic
                                 - glove_interfaces  shared msg/srv defs
                                 - soft_glove_core   shared/core utilities
                               See docs/ros2_architecture.md for topic-
                               level detail.

tools/pressure/, tools/position/
                               Host-side Python scripts for calibration
                               and sweep-capture experiments (see
                               docs/calibration.md, docs/pressure_control.md).

tests/equivalence/             Host C tests comparing the ROS 2 / current
                               firmware behavior against
                               reference/rp2040_validated/ for numerical
                               equivalence. Not modified as part of this
                               documentation pass.

data/pressure_tests/           Raw experiment CSVs (gitignored bulk data,
                               see data/README.md).
```

## Data flow (high level)

1. **Sensing**: `pico_firmware/` samples pressure/flex ADC channels and,
   via the ROS 2 estimator input path, IMU (BNO055) data is carried to the
   host for UKF processing. See `docs/pico_firmware.md` and
   `docs/sensing_and_ukf.md`.
2. **Bridge**: `pico_bridge` (ros2_ws) parses the Pico's serial frames
   into ROS 2 messages (`glove_interfaces/msg/SensorFrame`,
   `PicoStatus`, `EstimatorInputFrame`) and turns ROS 2
   `ActuatorCommand`/`ControlCommand` messages into serial frames back to
   the Pico. See `docs/ros2_architecture.md`.
3. **Estimation**: `glove_estimator` runs the UKF4 state estimator,
   publishing `glove_interfaces/msg/EstimatorState`.
4. **Control**: `glove_controller` runs the position (PI or LADRC/LESO,
   gain-scheduled) and pressure (PI, FILL/HOLD/VENT) control logic,
   publishing `ActuatorCommand` and `ControllerStatus`.
5. **Actuation & safety**: `pico_firmware` executes the actuator commands
   with precise pulse timing and enforces a local watchdog/safety latch
   independent of the ROS 2 host, so a communication loss or host fault
   cannot leave actuators in an unsafe state. See
   `docs/pico_firmware.md` and `docs/troubleshooting.md`.

## Golden reference vs. migrated system

`reference/rp2040_validated/` is a complete, self-contained implementation
of the same control algorithms (UKF4, gain scheduling, PI/LADRC position
control, pressure PI, FILL/HOLD/VENT, reversal lockout, calibration,
safety limits) running entirely on the RP2040. It is kept **only** as a
frozen behavioral reference to validate the ROS 2 migration against — it
is not part of the deployed ROS 2 system and must never be changed to
"match" ROS 2 behavior; equivalence must be achieved by matching the ROS 2
/ current-firmware implementation to it, not the other way around.
