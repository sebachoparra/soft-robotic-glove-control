import csv
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

from glove_interfaces.msg import (
    ActuatorCommand,
    ControlCommand,
    SensorFrame,
    ControllerStatus,
    EstimatorState,
)

SETPOINTS = [20, 40, 60, 80, 60, 40, 20]

PI_UKF_MODE = 4

STABLE_BAND_PCT = 3.0
STABLE_TIME_S = 10.0
MAX_STEP_TIME_S = 60.0

RAW_PRESSURE_LIMIT_KPA = 180.0

VENT_MAX_TIME_S = 45.0
VENT_ZERO_BAND_KPA = 1.0
VENT_STABLE_TIME_S = 3.0

CSV_PATH = Path("/tmp/pi_ukf_sweep_20_80.csv")


class TestNode(Node):
    def __init__(self):
        super().__init__("pi_ukf_sweep_20_80")

        self.sensor = None
        self.status = None
        self.estimator = None

        self.sensor_rx_time = None
        self.estimator_rx_time = None
        self.pico_us = None

        self.act_seq = int(time.time() * 1000) & 0xFFFFFFFF

        self.act_pub = self.create_publisher(
            ActuatorCommand,
            "/glove_control/actuator_command",
            10,
        )

        self.ctrl_pub = self.create_publisher(
            ControlCommand,
            "/glove_control/control_command",
            10,
        )

        self.create_subscription(
            SensorFrame,
            "/pico_bridge/sensor_frame",
            self.sensor_cb,
            10,
        )

        self.create_subscription(
            ControllerStatus,
            "/glove_control/controller_status",
            self.status_cb,
            10,
        )

        self.create_subscription(
            EstimatorState,
            "/ukf/estimator_state",
            self.estimator_cb,
            10,
        )

    def sensor_cb(self, msg):
        self.sensor = msg
        self.pico_us = msg.pico_timestamp_us
        self.sensor_rx_time = time.monotonic()

    def status_cb(self, msg):
        self.status = msg

    def estimator_cb(self, msg):
        self.estimator = msg
        self.estimator_rx_time = time.monotonic()

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for_data(self, timeout=5.0):
        end = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

            if (
                self.sensor is not None
                and self.status is not None
                and self.estimator is not None
            ):
                return True

        return False

    def send_actuator(self, command, pulse_ms=0):
        if self.pico_us is None:
            return False

        self.act_seq = (self.act_seq + 1) & 0xFFFFFFFF

        msg = ActuatorCommand()
        msg.command = command
        msg.pulse_ms = pulse_ms
        msg.seq = self.act_seq
        msg.deadline_us = (self.pico_us + 150000) & 0xFFFFFFFF

        self.act_pub.publish(msg)
        return True

    def set_mode(self, mode):
        msg = ControlCommand()
        msg.kind = 1
        msg.mode = mode
        self.ctrl_pub.publish(msg)

    def set_q(self, q_ref):
        msg = ControlCommand()
        msg.kind = 0
        msg.q_ref_pct = float(q_ref)
        self.ctrl_pub.publish(msg)


def vent_to_safe(node, reason, abort=False):
    print()

    if abort:
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        print("ABORT:", reason)
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    else:
        print("========================================")
        print(reason)
        print("========================================")

    node.set_mode(0)
    node.spin_for(0.03)

    start = time.monotonic()
    next_vent = 0.0
    zero_since = None

    print("VENT sostenido...")

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.005)
        now = time.monotonic()

        if now >= next_vent:
            node.send_actuator(2, 0)
            next_vent = now + 0.05

        if node.sensor is not None:
            pf = node.sensor.pressure_filtered_kpa

            if abs(pf) <= VENT_ZERO_BAND_KPA:
                if zero_since is None:
                    zero_since = now
                elif now - zero_since >= VENT_STABLE_TIME_S:
                    print(f"VENT complete: Pf={pf:.3f} kPa")
                    break
            else:
                zero_since = None

        if now - start >= VENT_MAX_TIME_S:
            print("VENT maximum time reached.")
            break

    node.send_actuator(5, 0)
    node.spin_for(0.10)


rclpy.init()
node = TestNode()

csv_file = open(CSV_PATH, "w", newline="")
writer = csv.writer(csv_file)

writer.writerow([
    "elapsed_s",
    "step",
    "q_ref_pct",
    "s_hat_pct",
    "position_error_pct",
    "pressure_ref_kpa",
    "pressure_raw_kpa",
    "pressure_filtered_kpa",
    "active_kp",
    "active_ki",
    "eta_hat",
    "b_f_hat",
    "theta_deg",
    "nis",
    "pneumatic_state",
    "pulse_active",
    "pump_on",
    "safety_latched",
    "watchdog_state",
    "link_up",
    "estimator_stale",
])

experiment_start = time.monotonic()

try:
    print("Waiting for Pico + controller + UKF...")

    if not node.wait_for_data():
        raise RuntimeError("Missing Pico, controller or UKF telemetry")

    print(
        f"Initial: "
        f"P={node.sensor.pressure_kpa:.3f} kPa  "
        f"Pf={node.sensor.pressure_filtered_kpa:.3f} kPa  "
        f"s={node.estimator.s_hat:.2f}%  "
        f"NIS={node.estimator.nis:.3f}  "
        f"pump={int(node.sensor.pump_on)}  "
        f"latch={int(node.sensor.safety_latched)}  "
        f"wd={node.sensor.watchdog_state}"
    )

    if abs(node.sensor.pressure_filtered_kpa) > 2.0:
        raise RuntimeError("Initial pressure is not near zero")

    if not node.estimator.valid:
        raise RuntimeError("UKF is not valid")

    if not node.estimator.reference_valid:
        raise RuntimeError("UKF reference is not valid")

    if node.status.sensor_stale:
        raise RuntimeError("Sensor telemetry is stale")

    if node.status.estimator_stale:
        raise RuntimeError("Estimator telemetry is stale")

    if not node.status.link_up:
        raise RuntimeError("Pico link is down")

    print("\nFresh ALL_OFF acknowledgement...")
    node.send_actuator(5, 0)
    node.spin_for(0.05)

    print("Entering PI_UKF mode...")
    node.set_mode(PI_UKF_MODE)
    node.spin_for(0.30)

    if node.status.controller_mode != PI_UKF_MODE:
        raise RuntimeError("PI_UKF mode was not entered")

    if not node.status.control_enabled:
        raise RuntimeError("Controller did not enable")

    if node.sensor.safety_latched:
        raise RuntimeError("Pico safety latch remained active")

    if node.sensor.watchdog_state != 0:
        raise RuntimeError("Pico watchdog active at mode entry")

    print()
    print("========================================")
    print("STARTING PI + UKF POSITION SWEEP")
    print(SETPOINTS)
    print("========================================")

    for step_index, q_ref in enumerate(SETPOINTS, start=1):

        print()
        print("----------------------------------------")
        print(
            f"STEP {step_index}/{len(SETPOINTS)}: "
            f"q_ref={q_ref}%"
        )
        print("----------------------------------------")

        node.set_q(q_ref)
        node.spin_for(0.70)

        step_start = time.monotonic()
        stable_since = None
        last_print = 0.0

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            now = time.monotonic()

            if (
                node.sensor is None
                or node.status is None
                or node.estimator is None
            ):
                continue

            if (
                node.sensor_rx_time is None
                or now - node.sensor_rx_time > 0.25
            ):
                raise RuntimeError("Sensor telemetry stale >250 ms")

            if (
                node.estimator_rx_time is None
                or now - node.estimator_rx_time > 0.25
            ):
                raise RuntimeError("UKF telemetry stale >250 ms")

            if not node.status.link_up:
                raise RuntimeError("Pico link lost")

            if node.status.abort_active:
                raise RuntimeError("Controller abort active")

            if node.status.overpressure_latched:
                raise RuntimeError("Controller overpressure latch active")

            if node.status.sensor_stale:
                raise RuntimeError("Controller reports sensor stale")

            if node.status.estimator_stale:
                raise RuntimeError("Controller reports estimator stale")

            if node.sensor.safety_latched:
                raise RuntimeError("Pico safety latch activated")

            if node.sensor.watchdog_state != 0:
                raise RuntimeError("Pico watchdog activated")

            if not node.sensor.pump_on:
                raise RuntimeError(
                    "Pump unexpectedly OFF during PI_UKF control"
                )

            if not node.estimator.valid:
                raise RuntimeError("UKF became invalid")

            if not node.estimator.reference_valid:
                raise RuntimeError("UKF reference became invalid")

            if node.estimator.nis > 1000.0:
                raise RuntimeError(
                    f"UKF NIS diverged: {node.estimator.nis:.1f}"
                )

            p = node.sensor.pressure_kpa
            pf = node.sensor.pressure_filtered_kpa
            s = node.estimator.s_hat
            error = q_ref - s

            if p >= RAW_PRESSURE_LIMIT_KPA:
                raise RuntimeError(
                    f"RAW pressure reached {p:.2f} kPa "
                    f"(limit {RAW_PRESSURE_LIMIT_KPA:.1f})"
                )

            writer.writerow([
                now - experiment_start,
                step_index,
                q_ref,
                s,
                error,
                node.status.pressure_ref_kpa,
                p,
                pf,
                node.status.active_kp,
                node.status.active_ki,
                node.estimator.eta_hat,
                node.estimator.b_f_hat,
                node.estimator.theta_deg,
                node.estimator.nis,
                node.sensor.pneumatic_state,
                int(node.sensor.pulse_active),
                int(node.sensor.pump_on),
                int(node.sensor.safety_latched),
                node.sensor.watchdog_state,
                int(node.status.link_up),
                int(node.status.estimator_stale),
            ])
            csv_file.flush()

            if now - last_print >= 0.50:
                print(
                    f"t={now-step_start:5.1f}s  "
                    f"qref={q_ref:5.1f}%  "
                    f"s={s:7.2f}%  "
                    f"e={error:7.2f}%  "
                    f"Pref={node.status.pressure_ref_kpa:7.2f}  "
                    f"Pf={pf:7.2f}  "
                    f"Kp={node.status.active_kp:6.3f}  "
                    f"Ki={node.status.active_ki:6.3f}  "
                    f"eta={node.estimator.eta_hat:5.3f}  "
                    f"NIS={node.estimator.nis:8.2f}"
                )
                last_print = now

            if abs(error) <= STABLE_BAND_PCT:
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= STABLE_TIME_S:
                    print(
                        f"STABLE: s={s:.2f}%, "
                        f"|e|={abs(error):.2f}% "
                        f"for {now-stable_since:.1f} s"
                    )
                    break
            else:
                stable_since = None

            if now - step_start >= MAX_STEP_TIME_S:
                raise RuntimeError(
                    f"q_ref={q_ref}% failed to stabilize within "
                    f"{MAX_STEP_TIME_S:.0f} s. "
                    f"Final s={s:.2f}%"
                )

    vent_to_safe(node, "SWEEP COMPLETED", abort=False)

except KeyboardInterrupt:
    vent_to_safe(node, "Operator Ctrl+C", abort=True)

except Exception as exc:
    vent_to_safe(node, str(exc), abort=True)

finally:
    csv_file.close()
    print(f"CSV: {CSV_PATH}")
    node.destroy_node()
    rclpy.shutdown()
