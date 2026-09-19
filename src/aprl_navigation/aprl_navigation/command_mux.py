"""Choose fresh operator or permitted Nav2 commands using a wall-clock watchdog."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class VelocityGate:
    def __init__(self, timeout=0.35, permit_timeout=0.4, max_linear=0.3, max_angular=0.6):
        self.timeout, self.permit_timeout = timeout, permit_timeout
        self.max_linear, self.max_angular = max_linear, max_angular
        self.manual = self.navigation = (0.0, 0.0)
        self.manual_time = self.navigation_time = self.permit_time = -math.inf
        self.inhibited = True

    def receive(self, source, linear, angular, now):
        if not math.isfinite(linear) or not math.isfinite(angular):
            linear = angular = 0.0
        value = (
            max(-self.max_linear, min(self.max_linear, linear)),
            max(-self.max_angular, min(self.max_angular, angular)),
        )
        if source == "manual":
            self.manual, self.manual_time = value, now
        else:
            self.navigation, self.navigation_time = value, now

    def inhibit(self, value, now):
        self.inhibited, self.permit_time = value, now

    def output(self, now):
        if now - self.manual_time <= self.timeout:
            return self.manual
        if (
            not self.inhibited
            and now - self.permit_time <= self.permit_timeout
            and now - self.navigation_time <= self.timeout
        ):
            return self.navigation
        return (0.0, 0.0)


class CommandMux(Node):
    def __init__(self):
        super().__init__("aprl_command_mux")
        settings = {
            name: self.declare_parameter(name, default).value
            for name, default in {
                "command_timeout": 0.35,
                "permit_timeout": 0.4,
                "max_linear": 0.3,
                "max_angular": 0.6,
            }.items()
        }
        self.gate = VelocityGate(
            settings["command_timeout"], settings["permit_timeout"],
            settings["max_linear"], settings["max_angular"],
        )
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.create_subscription(Twist, "/cmd_vel_teleop", self.manual, 1)
        self.create_subscription(Twist, "/cmd_vel_nav", self.navigation, 1)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/navigation/inhibit", self.inhibit, qos)
        self.create_timer(0.05, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def manual(self, message):
        self.gate.receive("manual", message.linear.x, message.angular.z, time.monotonic())

    def navigation(self, message):
        self.gate.receive("navigation", message.linear.x, message.angular.z, time.monotonic())

    def inhibit(self, message):
        self.gate.inhibit(message.data, time.monotonic())
        self.publish()

    def publish(self):
        linear, angular = self.gate.output(time.monotonic())
        message = Twist()
        message.linear.x, message.angular.z = linear, angular
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = CommandMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
