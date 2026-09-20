import time
import rclpy

from rclpy.node import Node
from glove_interfaces.msg import ActuatorCommand, SensorFrame


class VentToZero(Node):
    def __init__(self):
        super().__init__('vent_to_zero')

        self.sensor = None
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

    def sensor_cb(self, msg):
        self.sensor = msg
        self.pico_us = msg.pico_timestamp_us

    def send(self, command):
        if self.pico_us is None:
            return

        self.seq = (self.seq + 1) & 0xFFFFFFFF

        msg = ActuatorCommand()
        msg.command = command
        msg.pulse_ms = 0
        msg.seq = self.seq
        msg.deadline_us = (self.pico_us + 150000) & 0xFFFFFFFF

        self.pub.publish(msg)


rclpy.init()
node = VentToZero()

print("Waiting for Pico...")

deadline = time.monotonic() + 5.0

while rclpy.ok() and node.sensor is None and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.05)

if node.sensor is None:
    print("ERROR: no sensor telemetry")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(1)

# Fresh ALL_OFF acknowledgement
node.send(5)

for _ in range(5):
    rclpy.spin_once(node, timeout_sec=0.01)

print("VENT active for 30 s")

start = time.monotonic()
next_send = 0.0
next_print = 0.0

while rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.005)

    now = time.monotonic()

    if now >= next_send:
        node.send(2)
        next_send = now + 0.05

    if node.sensor is not None and now >= next_print:
        print(
            f"t={now-start:5.1f}s  "
            f"P={node.sensor.pressure_kpa:7.3f} kPa  "
            f"Pf={node.sensor.pressure_filtered_kpa:7.3f} kPa  "
            f"state={node.sensor.pneumatic_state}  "
            f"pump={int(node.sensor.pump_on)}  "
            f"latch={int(node.sensor.safety_latched)}"
        )
        next_print = now + 0.25

    if now - start >= 30.0:
        print("30 s vent complete.")
        break

node.send(5)

for _ in range(10):
    rclpy.spin_once(node, timeout_sec=0.01)

print("ALL_OFF")

node.destroy_node()
rclpy.shutdown()
