#!/usr/bin/env python3

import csv
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node

from glove_interfaces.msg import (
    SensorFrame,
    EstimatorInputFrame,
    EstimatorState,
    ControllerStatus,
    PicoStatus,
    ActuatorCommand,
    ControlCommand,
)


DATA_DIR = Path(
    "/home/sebas/soft_glove_stage6/data/pressure_tests"
)
DATA_DIR.mkdir(parents=True, exist_ok=True)

STAMP = datetime.now().strftime("%Y%m%d_%H%M%S")
CSV_PATH = DATA_DIR / f"pressure_sweep_full_sensing_{STAMP}.csv"


class FullSensingLogger(Node):

    def __init__(self):
        super().__init__("full_sensing_logger")

        self.est_input = None
        self.est_state = None
        self.controller = None
        self.pico_status = None
        self.actuator = None

        self.step_index = 0
        self.current_setpoint = None
        self.previous_setpoint = None
        self.direction = "NONE"

        self.row_count = 0
        self.last_report = time.monotonic()

        self.file = CSV_PATH.open(
            "w",
            newline="",
            buffering=1,
        )

        self.writer = csv.writer(self.file)

        self.writer.writerow([
            # Logger
            "host_time_ns",
            "host_monotonic_ns",
            "step_index",
            "direction",
            "commanded_setpoint_kpa",

            # SensorFrame
            "sensor_ros_sec",
            "sensor_ros_nanosec",
            "sensor_pico_timestamp_us",
            "sensor_seq",
            "pressure_raw",
            "pressure_vout",
            "pressure_kpa",
            "pressure_filtered_kpa",
            "flex_raw",
            "flex_filtered_raw",
            "sensor_pneumatic_state",
            "pulse_active",
            "pulse_ms_remaining",
            "pump_on",
            "safety_latched",
            "pulse_alarm_fail_count",
            "watchdog_state",

            # EstimatorInputFrame
            "est_input_ros_sec",
            "est_input_ros_nanosec",
            "est_input_pico_timestamp_us",
            "est_input_seq",
            "est_input_flex_filtered_raw",
            "bno1_ok_input",
            "bno1_q0",
            "bno1_q1",
            "bno1_q2",
            "bno1_q3",
            "bno2_ok_input",
            "bno2_q0",
            "bno2_q1",
            "bno2_q2",
            "bno2_q3",

            # UKF EstimatorState
            "ukf_ros_sec",
            "ukf_ros_nanosec",
            "s_hat",
            "v_hat",
            "eta_hat",
            "b_f_hat",
            "theta_deg",
            "innovation_flex",
            "innovation_theta",
            "nis",
            "sigma_s",
            "sigma_v",
            "sigma_eta",
            "sigma_b_f",
            "rho_eta_b_f",
            "dt_used",
            "ukf_valid",
            "reference_valid",
            "theta_valid",
            "bno1_ok_ukf",
            "bno2_ok_ukf",
            "ukf_source_seq",
            "ukf_source_timestamp_us",
            "estimator_stamp_sec",
            "estimator_stamp_nanosec",

            # ControllerStatus
            "controller_ros_sec",
            "controller_ros_nanosec",
            "controller_mode",
            "control_enabled",
            "position_ref_pct",
            "pressure_ref_kpa",
            "active_kp",
            "active_ki",
            "active_b0",
            "eso_z1_pct",
            "eso_z2_pct_s",
            "ff_gamma",
            "controller_pneumatic_state",
            "last_pulse_ms",
            "rate_limited",
            "saturated",
            "ran_sample",
            "ran_ukf",
            "ran_eso",
            "ran_outer",
            "ran_pressure",
            "ran_log",
            "estimator_source_seq",
            "estimator_acquisition_age_us",
            "estimator_pipeline_latency_us",
            "estimator_seq_repeat",
            "abort_active",
            "overpressure_latched",
            "link_up_controller",
            "estimator_stale",
            "sensor_stale",

            # ActuatorCommand
            "actuator_command",
            "actuator_pulse_ms",
            "actuator_seq",
            "actuator_deadline_us",

            # PicoStatus
            "pico_link_up",
            "pico_rx_frames",
            "pico_tx_frames",
            "pico_crc_errors",
            "pico_framing_errors",
            "pico_seq_gaps",
            "pico_last_rx_pico_us",
            "pico_reconnect_count",
        ])

        self.create_subscription(
            SensorFrame,
            "/pico_bridge/sensor_frame",
            self.sensor_cb,
            50,
        )

        self.create_subscription(
            EstimatorInputFrame,
            "/pico_bridge/estimator_input",
            self.est_input_cb,
            50,
        )

        self.create_subscription(
            EstimatorState,
            "/ukf/estimator_state",
            self.est_state_cb,
            50,
        )

        self.create_subscription(
            ControllerStatus,
            "/glove_control/controller_status",
            self.controller_cb,
            50,
        )

        self.create_subscription(
            PicoStatus,
            "/pico_bridge/pico_status",
            self.pico_status_cb,
            20,
        )

        self.create_subscription(
            ActuatorCommand,
            "/glove_control/actuator_command",
            self.actuator_cb,
            100,
        )

        self.create_subscription(
            ControlCommand,
            "/glove_control/control_command",
            self.control_command_cb,
            50,
        )

        print("============================================")
        print("FULL SENSING LOGGER")
        print("============================================")
        print(f"CSV: {CSV_PATH}")
        print("Waiting for data...")
        print("Press Ctrl+C after the sweep is completely finished.")
        print()

    @staticmethod
    def val(obj, name, default=""):
        if obj is None:
            return default
        return getattr(obj, name, default)

    @staticmethod
    def stamp(obj):
        if obj is None:
            return "", ""

        try:
            return (
                obj.header.stamp.sec,
                obj.header.stamp.nanosec,
            )
        except Exception:
            return "", ""

    @staticmethod
    def quat_values(q):
        if q is None:
            return ["", "", "", ""]

        try:
            return [q[0], q[1], q[2], q[3]]
        except Exception:
            return ["", "", "", ""]

    def est_input_cb(self, msg):
        self.est_input = msg

    def est_state_cb(self, msg):
        self.est_state = msg

    def controller_cb(self, msg):
        self.controller = msg

    def pico_status_cb(self, msg):
        self.pico_status = msg

    def actuator_cb(self, msg):
        self.actuator = msg

    def control_command_cb(self, msg):
        # SET_P = 5
        if msg.kind != 5:
            return

        new_sp = float(msg.pressure_ref_kpa)

        self.previous_setpoint = self.current_setpoint
        self.current_setpoint = new_sp
        self.step_index += 1

        if self.previous_setpoint is None:
            self.direction = "START"
        elif new_sp > self.previous_setpoint:
            self.direction = "UP"
        elif new_sp < self.previous_setpoint:
            self.direction = "DOWN"
        else:
            self.direction = "HOLD"

        print(
            f"STEP {self.step_index}: "
            f"SET_P={new_sp:.1f} kPa "
            f"direction={self.direction}"
        )

    def sensor_cb(self, s):
        ei = self.est_input
        ukf = self.est_state
        ctrl = self.controller
        act = self.actuator
        ps = self.pico_status

        sensor_sec, sensor_nsec = self.stamp(s)
        ei_sec, ei_nsec = self.stamp(ei)
        ukf_sec, ukf_nsec = self.stamp(ukf)
        ctrl_sec, ctrl_nsec = self.stamp(ctrl)

        bno1 = self.quat_values(
            self.val(ei, "bno1_quat", None)
        )
        bno2 = self.quat_values(
            self.val(ei, "bno2_quat", None)
        )

        row = [
            time.time_ns(),
            time.monotonic_ns(),
            self.step_index,
            self.direction,
            (
                self.current_setpoint
                if self.current_setpoint is not None
                else ""
            ),

            # SensorFrame
            sensor_sec,
            sensor_nsec,
            s.pico_timestamp_us,
            s.seq,
            s.pressure_raw,
            s.pressure_vout,
            s.pressure_kpa,
            s.pressure_filtered_kpa,
            s.flex_raw,
            s.flex_filtered_raw,
            s.pneumatic_state,
            int(s.pulse_active),
            s.pulse_ms_remaining,
            int(s.pump_on),
            int(s.safety_latched),
            s.pulse_alarm_fail_count,
            s.watchdog_state,

            # EstimatorInputFrame
            ei_sec,
            ei_nsec,
            self.val(ei, "pico_timestamp_us"),
            self.val(ei, "seq"),
            self.val(ei, "flex_filtered_raw"),
            int(self.val(ei, "bno1_ok", False)),
            *bno1,
            int(self.val(ei, "bno2_ok", False)),
            *bno2,

            # UKF
            ukf_sec,
            ukf_nsec,
            self.val(ukf, "s_hat"),
            self.val(ukf, "v_hat"),
            self.val(ukf, "eta_hat"),
            self.val(ukf, "b_f_hat"),
            self.val(ukf, "theta_deg"),
            self.val(ukf, "innovation_flex"),
            self.val(ukf, "innovation_theta"),
            self.val(ukf, "nis"),
            self.val(ukf, "sigma_s"),
            self.val(ukf, "sigma_v"),
            self.val(ukf, "sigma_eta"),
            self.val(ukf, "sigma_b_f"),
            self.val(ukf, "rho_eta_b_f"),
            self.val(ukf, "dt_used"),
            int(self.val(ukf, "valid", False)),
            int(self.val(ukf, "reference_valid", False)),
            int(self.val(ukf, "theta_valid", False)),
            int(self.val(ukf, "bno1_ok", False)),
            int(self.val(ukf, "bno2_ok", False)),
            self.val(ukf, "source_seq"),
            self.val(ukf, "source_timestamp_us"),
            (
                self.val(ukf, "estimator_stamp").sec
                if ukf is not None
                else ""
            ),
            (
                self.val(ukf, "estimator_stamp").nanosec
                if ukf is not None
                else ""
            ),

            # ControllerStatus
            ctrl_sec,
            ctrl_nsec,
            self.val(ctrl, "controller_mode"),
            int(self.val(ctrl, "control_enabled", False)),
            self.val(ctrl, "position_ref_pct"),
            self.val(ctrl, "pressure_ref_kpa"),
            self.val(ctrl, "active_kp"),
            self.val(ctrl, "active_ki"),
            self.val(ctrl, "active_b0"),
            self.val(ctrl, "eso_z1_pct"),
            self.val(ctrl, "eso_z2_pct_s"),
            self.val(ctrl, "ff_gamma"),
            self.val(ctrl, "pneumatic_state"),
            self.val(ctrl, "last_pulse_ms"),
            int(self.val(ctrl, "rate_limited", False)),
            int(self.val(ctrl, "saturated", False)),
            int(self.val(ctrl, "ran_sample", False)),
            int(self.val(ctrl, "ran_ukf", False)),
            int(self.val(ctrl, "ran_eso", False)),
            int(self.val(ctrl, "ran_outer", False)),
            int(self.val(ctrl, "ran_pressure", False)),
            int(self.val(ctrl, "ran_log", False)),
            self.val(ctrl, "estimator_source_seq"),
            self.val(ctrl, "estimator_acquisition_age_us"),
            self.val(ctrl, "estimator_pipeline_latency_us"),
            int(self.val(ctrl, "estimator_seq_repeat", False)),
            int(self.val(ctrl, "abort_active", False)),
            int(self.val(ctrl, "overpressure_latched", False)),
            int(self.val(ctrl, "link_up", False)),
            int(self.val(ctrl, "estimator_stale", False)),
            int(self.val(ctrl, "sensor_stale", False)),

            # ActuatorCommand
            self.val(act, "command"),
            self.val(act, "pulse_ms"),
            self.val(act, "seq"),
            self.val(act, "deadline_us"),

            # PicoStatus
            int(self.val(ps, "link_up", False)),
            self.val(ps, "rx_frames"),
            self.val(ps, "tx_frames"),
            self.val(ps, "crc_errors"),
            self.val(ps, "framing_errors"),
            self.val(ps, "seq_gaps"),
            self.val(ps, "last_rx_pico_us"),
            self.val(ps, "reconnect_count"),
        ]

        self.writer.writerow(row)
        self.row_count += 1

        now = time.monotonic()

        if now - self.last_report >= 2.0:
            self.last_report = now

            s_hat = self.val(ukf, "s_hat", float("nan"))
            theta = self.val(
                ukf,
                "theta_deg",
                float("nan"),
            )

            print(
                f"rows={self.row_count:7d}  "
                f"P={s.pressure_filtered_kpa:7.2f}  "
                f"FLEX={s.flex_filtered_raw:8.2f}  "
                f"s_UKF={s_hat:7.2f}  "
                f"theta={theta:7.2f}  "
                f"BNO1={int(self.val(ei, 'bno1_ok', False))}  "
                f"BNO2={int(self.val(ei, 'bno2_ok', False))}  "
                f"step={self.step_index}"
            )

    def close(self):
        try:
            self.file.flush()
            self.file.close()
        except Exception:
            pass

        print()
        print("============================================")
        print("LOGGER CLOSED")
        print(f"Rows: {self.row_count}")
        print(f"CSV: {CSV_PATH}")
        print("============================================")


def main():
    rclpy.init()
    node = FullSensingLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
