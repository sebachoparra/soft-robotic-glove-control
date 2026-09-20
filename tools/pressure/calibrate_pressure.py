import time
import rclpy

from rclpy.node import Node
from glove_interfaces.msg import ActuatorCommand, SensorFrame
from glove_interfaces.srv import CalibrateSensors


class Calibrator(Node):
    def __init__(self):
        super().__init__('pressure_calibration_helper')

        self.pico_us = None
        self.seq = int(time.time() * 1000) & 0xFFFFFFFF

        self.pub = self.create_publisher(
            ActuatorCommand,
            '/glove_control/actuator_command',
            10
        )

        self.create_subscription(
            SensorFrame,
            '/pico_bridge/sensor_frame',
            self.sensor_cb,
            10
        )

        self.client = self.create_client(
            CalibrateSensors,
            '/pico_bridge/calibrate_sensors'
        )

    def sensor_cb(self, msg):
        self.pico_us = msg.pico_timestamp_us

    def send(self, command, pulse_ms=0):
        if self.pico_us is None:
            return False

        self.seq = (self.seq + 1) & 0xFFFFFFFF

        msg = ActuatorCommand()
        msg.command = command
        msg.pulse_ms = pulse_ms
        msg.seq = self.seq
        msg.deadline_us = (self.pico_us + 150000) & 0xFFFFFFFF

        self.pub.publish(msg)
        return True


rclpy.init()
node = Calibrator()

print("Waiting for Pico sensor frames...")

deadline = time.monotonic() + 5.0

while rclpy.ok() and node.pico_us is None and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)

if node.pico_us is None:
    print("ERROR: no SensorFrame received")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(1)

node.send(5)

for _ in range(5):
    rclpy.spin_once(node, timeout_sec=0.01)

print("ALL_OFF acknowledgement sent")
print("Maintaining VENT during calibration preparation...")

prep_end = time.monotonic() + 10.0
next_send = 0.0

while rclpy.ok() and time.monotonic() < prep_end:
    rclpy.spin_once(node, timeout_sec=0.005)

    now = time.monotonic()

    if now >= next_send:
        node.send(2, 0)
        next_send = now + 0.05

if not node.client.wait_for_service(timeout_sec=2.0):
    print("ERROR: calibration service unavailable")
    node.send(5)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(2)

print("Calling calibration service...")

future = node.client.call_async(CalibrateSensors.Request())

service_timeout = time.monotonic() + 3.0
next_send = 0.0

while rclpy.ok() and not future.done() and time.monotonic() < service_timeout:
    rclpy.spin_once(node, timeout_sec=0.005)

    now = time.monotonic()

    if now >= next_send:
        node.send(2, 0)
        next_send = now + 0.05

if future.done():
    response = future.result()

    print("CALIBRATION RESULT")
    print("success =", response.success)
    print("pressure_zero_vout =", response.pressure_zero_vout)
    print("flex_zero_raw =", response.flex_zero_raw)
    print("message =", response.message)
else:
    print("ERROR: calibration service timeout")

node.send(5)

for _ in range(10):
    rclpy.spin_once(node, timeout_sec=0.01)

print("Final ALL_OFF sent")

node.destroy_node()
rclpy.shutdown()
