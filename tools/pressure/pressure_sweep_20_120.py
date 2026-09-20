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
)

SETPOINTS = [20, 40, 60, 80, 100, 120, 100, 80, 60, 40, 20]

STABLE_BAND_KPA = 3.0
STABLE_TIME_S = 5.0
MAX_STEP_TIME_S = 30.0

GLOBAL_RAW_LIMIT_KPA = 135.0
UPWARD_OVERSHOOT_KPA = 15.0

VENT_MAX_TIME_S = 45.0
VENT_ZERO_BAND_KPA = 1.0
VENT_STABLE_TIME_S = 3.0

CSV_PATH = Path("/tmp/pressure_sweep_20_120.csv")


class SweepNode(Node):
    def __init__(self):
        super().__init__("pressure_sweep_20_120")

        self.sensor = None
        self.status = None
        self.sensor_rx_time = None
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

    def sensor_cb(self, msg):
        self.sensor = msg
        self.pico_us = msg.pico_timestamp_us
        self.sensor_rx_time = time.monotonic()

    def status_cb(self, msg):
        self.status = msg

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

    def set_pressure(self, pressure):
        msg = ControlCommand()
        msg.kind = 5
        msg.pressure_ref_kpa = float(pressure)
        self.ctrl_pub.publish(msg)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_for_data(self, timeout=5.0):
        end = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.sensor is not None and self.status is not None:
                return True

        return False


def fail_and_vent(node, reason):
    print()
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    print("ABORT:", reason)
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    # Stop the controller. This briefly produces ALL_OFF.
    node.set_mode(0)
    node.spin_for(0.03)

    # Immediately take control of the valves and continuously VENT.
    start = time.monotonic()
    next_vent = 0.0
    zero_since = None

    print("Emergency/end VENT...")

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
node = SweepNode()

csv_file = open(CSV_PATH, "w", newline="")
writer = csv.writer(csv_file)

writer.writerow([
    "elapsed_s",
    "step",
    "pref_kpa",
    "pressure_raw_kpa",
    "pressure_filtered_kpa",
    "pneumatic_state",
    "pulse_active",
    "pulse_ms_remaining",
    "pump_on",
    "safety_latched",
    "watchdog_state",
])

experiment_start = time.monotonic()

try:
    print("Waiting for Pico + controller...")

    if not node.wait_for_data():
        raise RuntimeError("Missing Pico or controller telemetry")

    print(
        f"Initial: P={node.sensor.pressure_kpa:.3f} "
        f"Pf={node.sensor.pressure_filtered_kpa:.3f} "
        f"pump={int(node.sensor.pump_on)} "
        f"latch={int(node.sensor.safety_latched)} "
        f"wd={node.sensor.watchdog_state}"
    )

    if abs(node.sensor.pressure_filtered_kpa) > 2.0:
        raise RuntimeError(
            "Initial pressure is not near zero. "
            "Run VENT before the sweep."
        )

    print("\nFresh ALL_OFF acknowledgement...")
    node.send_actuator(5, 0)
    node.spin_for(0.05)

    print("Entering CTRL_PRESSURE...")
    node.set_mode(3)
    node.spin_for(0.30)

    if node.status.controller_mode != 3:
        raise RuntimeError("CTRL_PRESSURE mode was not entered")

    if not node.status.control_enabled:
        raise RuntimeError("Controller did not enable")

    if node.sensor.safety_latched:
        raise RuntimeError("Pico safety latch remained active")

    if node.sensor.watchdog_state != 0:
        raise RuntimeError("Pico watchdog active at mode entry")

    if not node.sensor.pump_on:
        raise RuntimeError("Pump did not turn ON")

    previous_sp = node.sensor.pressure_filtered_kpa

    print("\n========================================")
    print("STARTING PRESSURE SWEEP")
    print(SETPOINTS)
    print("========================================")

    for step_index, sp in enumerate(SETPOINTS, start=1):

        upward = sp > previous_sp

        print()
        print("----------------------------------------")
        print(f"STEP {step_index}/{len(SETPOINTS)}: SET_P={sp} kPa")
        print("----------------------------------------")

        node.set_pressure(float(sp))

        # Allow status transport.
        node.spin_for(0.10)

        step_start = time.monotonic()
        stable_since = None
        last_print = 0.0

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            now = time.monotonic()

            if node.sensor is None or node.status is None:
                continue

            if (
                node.sensor_rx_time is None
                or now - node.sensor_rx_time > 0.25
            ):
                raise RuntimeError("Sensor telemetry stale >250 ms")

            p = node.sensor.pressure_kpa
            pf = node.sensor.pressure_filtered_kpa

            writer.writerow([
                now - experiment_start,
                step_index,
                sp,
                p,
                pf,
                node.sensor.pneumatic_state,
                int(node.sensor.pulse_active),
                node.sensor.pulse_ms_remaining,
                int(node.sensor.pump_on),
                int(node.sensor.safety_latched),
                node.sensor.watchdog_state,
            ])
            csv_file.flush()

            if now - last_print >= 0.50:
                print(
                    f"t={now-step_start:5.1f}s  "
                    f"P={p:7.2f}  "
                    f"Pf={pf:7.2f}  "
                    f"Pref={node.status.pressure_ref_kpa:6.1f}  "
                    f"state={node.sensor.pneumatic_state:2d}  "
                    f"pulse={int(node.sensor.pulse_active)}  "
                    f"pump={int(node.sensor.pump_on)}"
                )
                last_print = now

            # Global safety ceiling for this first campaign.
            if p >= GLOBAL_RAW_LIMIT_KPA:
                raise RuntimeError(
                    f"RAW pressure reached {p:.2f} kPa "
                    f"(limit {GLOBAL_RAW_LIMIT_KPA})"
                )

            # Extra guard during upward steps.
            if upward and p > sp + UPWARD_OVERSHOOT_KPA:
                raise RuntimeError(
                    f"Upward-step overshoot: "
                    f"P={p:.2f}, target={sp}"
                )

            if node.sensor.safety_latched:
                raise RuntimeError("Pico safety latch activated")

            if node.sensor.watchdog_state != 0:
                raise RuntimeError("Pico watchdog activated")

            if not node.sensor.pump_on:
                raise RuntimeError(
                    "Pump unexpectedly OFF during pressure control"
                )

            error = abs(pf - sp)

            if error <= STABLE_BAND_KPA:
                if stable_since is None:
                    stable_since = now

                stable_duration = now - stable_since

                if stable_duration >= STABLE_TIME_S:
                    print(
                        f"STABLE: Pf={pf:.2f} kPa, "
                        f"|e|={error:.2f} kPa "
                        f"for {stable_duration:.1f} s"
                    )
                    break
            else:
                stable_since = None

            if now - step_start >= MAX_STEP_TIME_S:
                raise RuntimeError(
                    f"SET_P={sp} failed to stabilize within "
                    f"{MAX_STEP_TIME_S:.0f} s. "
                    f"Final Pf={pf:.2f} kPa"
                )

        previous_sp = float(sp)

    print()
    print("========================================")
    print("SWEEP COMPLETED")
    print("========================================")

    fail_and_vent(node, "Normal completion")

except KeyboardInterrupt:
    fail_and_vent(node, "Operator Ctrl+C")

except Exception as exc:
    fail_and_vent(node, str(exc))

finally:
    csv_file.close()

    print()
    print("CSV:", CSV_PATH)

    if node.sensor is not None:
        print(
            "FINAL:",
            f"P={node.sensor.pressure_kpa:.3f}",
            f"Pf={node.sensor.pressure_filtered_kpa:.3f}",
            f"state={node.sensor.pneumatic_state}",
            f"pump={int(node.sensor.pump_on)}",
            f"latch={int(node.sensor.safety_latched)}",
            f"wd={node.sensor.watchdog_state}",
        )

    node.destroy_node()
    rclpy.shutdown()
