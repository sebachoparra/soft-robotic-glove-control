# Soft Glove — ROS 2 Jazzy + RP2040 Pneumatic Soft-Robotics Glove

## Overview

This project controls a pneumatically-actuated soft-robotics glove. It is
migrating a validated, standalone RP2040 ("Pico") controller
(`reference/rp2040_validated/`, tag `rp2040-golden-v1`) to a ROS 2 Jazzy
system while preserving the original controller's numerical behavior. See
the project's governance policy for the authoritative migration scope,
constraints, and the list of control elements that must not be retuned or
redesigned (UKF4, gain scheduling, PI position controller, LADRC/LESO,
pressure PI, FILL/HOLD/VENT, reversal lockout, calibration, safety
limits).

## Architecture summary

The system splits into a high-level ROS 2 controller and a low-level Pico
firmware, per the project's migration policy:

- **ROS 2 / host side** (`ros2_ws/src/`): UKF state estimation, gain
  scheduling, the external PI/LADRC position controller, the internal
  pressure PI controller, supervisory mode/state logic, and the
  telemetry/experiment interface.
- **RP2040 Pico** (`pico_firmware/`): sensor acquisition (ADC, IMU), GPIO
  actuation, precise pulse timing, a local communication watchdog, and
  local emergency safety — it does not implement high-level control.

Full detail: [docs/architecture.md](docs/architecture.md) and
[docs/ros2_architecture.md](docs/ros2_architecture.md).

## Repository layout

| Path | Contents |
|---|---|
| `ros2_ws/src/` | ROS 2 Jazzy packages: `glove_controller`, `glove_estimator`, `glove_interfaces`, `pico_bridge`, `soft_glove_core`. |
| `pico_firmware/` | Current, ROS 2-integrated Pico C firmware (+ host tests). |
| `reference/rp2040_validated/` | **Frozen golden reference** RP2040 firmware (tag `rp2040-golden-v1`). Never modify. |
| `tools/pressure/`, `tools/position/` | Python calibration and pressure/position sweep scripts. |
| `docs/` | Project documentation (see [docs/README.md](docs/README.md) for the index). |
| `data/pressure_tests/` | Raw experiment CSVs (gitignored, regenerable — see [data/README.md](data/README.md)). |
| `build/`, `install/`, `log/` | colcon-generated output at the project root (gitignored). |

## Build instructions

### ROS 2 workspace (`ros2_ws/`)

```bash
source /opt/ros/jazzy/setup.bash
cd soft_glove_stage6
colcon build --base-paths ros2_ws/src
source install/setup.bash
```

(colcon build output lands at the project root — `./build`, `./install`,
`./log` — not inside `ros2_ws/`; see `.gitignore`.)

### Pico firmware (`pico_firmware/`)

Built with the Raspberry Pi Pico SDK against the C sources in
`pico_firmware/src` / `pico_firmware/include`. `pico_firmware/tests/`
contains host-side (non-hardware) unit tests built with plain CMake/CTest;
see `pico_firmware/tests/CMakeLists.txt`.

### Golden reference (`reference/rp2040_validated/`)

Built separately, using its own build scripts:
`reference/rp2040_validated/BUILD_RP2040.ps1` / `BUILD_RP2040.bat` (Pico
SDK build) or `reference/rp2040_validated/CMakeLists.txt` directly. This
firmware is frozen and must not be modified — only built/flashed as-is if
needed to reproduce reference behavior.

## Running the ROS 2 system

See [docs/ros2_architecture.md](docs/ros2_architecture.md) for the node,
topic, and service layout, and
[docs/PRESSURE_BRINGUP.md](docs/PRESSURE_BRINGUP.md) for a concrete
bring-up walkthrough (control command interface, calibration gating,
dry-run sequence).

## Calibration

See [docs/calibration.md](docs/calibration.md) for the pressure-zero and
FLEX calibration workflow, and `tools/pressure/run_calibration.sh` for the
scripted bring-up sequence.

## Safety

See [docs/troubleshooting.md](docs/troubleshooting.md) and
[docs/PRESSURE_BRINGUP.md](docs/PRESSURE_BRINGUP.md) for watchdog,
communication-timeout, and overpressure-latch behavior. Do not perform
hardware actuation experiments without understanding these safety
mechanisms first.

## Golden reference policy

`reference/rp2040_validated/` (tag `rp2040-golden-v1`) is preserved as a
frozen, validated behavioral reference. It must **never** be modified to
match ROS 2 behavior — equivalence is validated the other way around, by
testing the ROS 2 / current-firmware implementation against it (see
`docs/MIGRATION_PLAN.md`).

## Hardware / PCB design

See [hardware/pcb/](hardware/pcb/) for the PCB design files (schematics,
board layout, Gerbers, BOM) as they're added.

## Videos / Demos

- _(add YouTube links here as they become available)_
