# Documentation Index

| Doc | Description |
|---|---|
| [architecture.md](architecture.md) | Overall system architecture, module relationships, data flow. |
| [ros2_architecture.md](ros2_architecture.md) | Per-package breakdown of the ROS 2 stack: nodes, topics, services, messages. |
| [pico_firmware.md](pico_firmware.md) | Current (ROS 2-integrated) Pico firmware structure, serial protocol, watchdog/failsafe behavior. |
| [sensing_and_ukf.md](sensing_and_ukf.md) | UKF4 state estimation, IMU usage, relationship between the golden-reference UKF and the ROS 2 `glove_estimator` port. |
| [pressure_control.md](pressure_control.md) | Internal pressure PI controller, FILL/HOLD/VENT state logic. |
| [position_control_pi.md](position_control_pi.md) | PI position controller and gain scheduling. |
| [position_control_adrc.md](position_control_adrc.md) | LADRC/LESO position controller with scheduled b0. |
| [calibration.md](calibration.md) | Pressure zero and FLEX calibration workflow, related tools/pressure scripts and services. |
| [troubleshooting.md](troubleshooting.md) | Common failure modes (watchdog trips, safety latches, comm loss) and how they present/recover. |
| [MIGRATION_PLAN.md](MIGRATION_PLAN.md) | Detailed RP2040 -> ROS 2 migration plan, with file/line citations against the golden reference. |
| [PRESSURE_BRINGUP.md](PRESSURE_BRINGUP.md) | Stage 6 pressure-only bring-up notes: interface, calibration wire protocol, dry-run sequence. |

See also `reference/rp2040_validated/README.md` and
`reference/rp2040_validated/GAIN_SCHEDULING.md` for documentation that ships
alongside the frozen golden reference firmware itself.
