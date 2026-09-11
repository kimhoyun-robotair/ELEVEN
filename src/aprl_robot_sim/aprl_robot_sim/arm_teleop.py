"""Jog NERO joints from a terminal; targets are limited by the supplied URDF."""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def main():
    if not sys.stdin.isatty():
        raise SystemExit('arm_teleop needs an interactive terminal')
    rclpy.init()
    node = Node('nero_keyboard')
    publisher = node.create_publisher(JointState, '/arm/joint_command', 10)
    joints = {}
    node.create_subscription(JointState, '/joint_states',
                             lambda msg: joints.update(zip(msg.name, msg.position)), qos_profile_sensor_data)
    selected = 'nero_joint1'
    original = termios.tcgetattr(sys.stdin)
    print('1–7: select joint | j/k: −/+ 0.05 rad | o/c: open/close | q: quit', flush=True)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.02)
            if not select.select([sys.stdin], [], [], .02)[0]:
                continue
            key = sys.stdin.read(1)
            if key in ('q', '\x03'):
                break
            if key in '1234567':
                selected = 'nero_joint' + key
                print(f'Joint {key}', flush=True)
            elif key in 'jkoc':
                if selected not in joints:
                    print('Waiting for a locomanipulator /joint_states stream', flush=True)
                    continue
                msg = JointState()
                msg.name = ['gripper'] if key in 'oc' else [selected]
                msg.position = [(.08 if key == 'o' else 0.0) if key in 'oc'
                                else joints[selected]+(.05 if key == 'k' else -.05)]
                publisher.publish(msg)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
