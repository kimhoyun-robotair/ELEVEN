"""A live ROS feedback route: drive, board, change floor, exit, drive."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String


ROOT = Path(__file__).resolve().parents[1]
# The 180-degree arm mount presses rearward, keeping the front LiDAR clear of the panel.
DOCK_YAW = -math.pi/2


def angle_error(target, current):
    return math.atan2(math.sin(target-current), math.cos(target-current))


class Route(Node):
    def __init__(self, route, speed, robot):
        super().__init__('amr_elevator_example')
        self.route, self.speed, self.robot = route, speed, robot
        self.arm = None
        self.press_count = 0
        self.pose = self.state = None
        self.phase, self.distance = 'waiting', {'before': 0.0, 'after': 0.0}
        self.samples, self.last_sample = [], -math.inf
        self.last_state_wall = self.last_pose_wall = 0.0
        self.velocity = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_buttons = self.create_publisher(String, '/arm/press_button', 10)
        self.create_subscription(String, '/arm/state', self.on_arm, 10)
        self.buttons = self.create_publisher(String, '/elevator/command', 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_pose, 10)
        self.create_subscription(String, '/elevator/state', self.on_state, 10)

    def on_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        pose = (p.x, p.y, p.z, yaw)
        if self.pose and self.phase in self.distance:
            self.distance[self.phase] += math.hypot(p.x-self.pose[0], p.y-self.pose[1])
        self.pose = pose
        self.last_pose_wall = time.monotonic()
        seconds = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if seconds-self.last_sample >= .5:
            self.samples.append({'time':seconds, 'phase':self.phase, 'pose':pose})
            self.last_sample = seconds

    def on_arm(self, msg):
        self.arm = json.loads(msg.data)

    def on_state(self, msg):
        self.state = json.loads(msg.data)
        self.last_state_wall = time.monotonic()

    def drive(self, v=0.0, w=0.0):
        msg = Twist()
        msg.linear.x, msg.angular.z = float(v), float(w)
        self.velocity.publish(msg)

    def press(self, action, floor=1):
        self.buttons.publish(String(data=json.dumps({'id':self.route['elevator'], 'action':action, 'floor':floor})))

    def wait(self, predicate, *, control=None, hold_door=False, timeout=180):
        deadline, next_command, next_open = time.monotonic()+timeout, 0.0, 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=.02)
            now = time.monotonic()
            if now > deadline:
                raise RuntimeError(f'Timed out in {self.phase}: pose={self.pose}')
            if self.phase != 'waiting' and now-min(self.last_pose_wall, self.last_state_wall) > 5:
                raise RuntimeError('Simulation streams stopped; stopping the robot')
            if predicate():
                self.drive()
                return
            if now >= next_command:
                self.drive(*(control() if control else (0,0)))
                next_command = now+.04
            if hold_door and now >= next_open:
                self.press('open')
                next_open = now+.3
        raise RuntimeError('ROS shutdown during route')

    def go(self, target, hold_door=False, tolerance=.08, reverse=False):
        print(f"{self.phase}: {'reverse' if reverse else 'drive'} to {target}", flush=True)
        def control():
            dx, dy = target[0]-self.pose[0], target[1]-self.pose[1]
            error = angle_error(math.atan2(dy, dx) + (math.pi if reverse else 0), self.pose[3])
            v = min(self.speed, .9*math.hypot(dx,dy)) if abs(error) < .4 else 0.0
            return -v if reverse else v, max(-.65, min(.65, 1.8*error))
        self.wait(lambda: math.dist(target, self.pose[:2]) < tolerance,
                  control=control, hold_door=hold_door)

    def face(self, yaw, hold_door=False):
        def control():
            error = angle_error(yaw, self.pose[3])
            angular = max(-.6, min(.6, 1.5*error))
            # Four-wheel skid steering stalls at small proportional pivot commands.
            if self.robot == 'scout' and abs(error) >= .035:
                angular = math.copysign(max(.25, abs(angular)), error)
            return 0, angular
        self.wait(lambda: abs(angle_error(yaw, self.pose[3])) < .035,
                  control=control,
                  hold_door=hold_door)

    def straight_to(self, y, hold_door=False):
        def control():
            error = angle_error(DOCK_YAW, self.pose[3])
            speed = max(-self.speed, min(self.speed, .8*(self.pose[1]-y))) if abs(error) < .12 else 0.0
            return speed, max(-.45, min(.45, 1.8*error))
        self.wait(lambda: abs(y-self.pose[1]) < .025, control=control, hold_door=hold_door)

    def physical_press(self, action, floor):
        self.press_count += 1
        request_id = f'{action}-{self.press_count}'
        self.arm_buttons.publish(String(data=json.dumps({'id': self.route['elevator'],
            'action': action, 'floor': floor, 'request_id': request_id})))
        def finished():
            if self.arm and self.arm['request_id'] == request_id:
                if self.arm['phase'] == 'failed':
                    raise RuntimeError(self.arm['error'])
                return self.arm['phase'] == 'complete'
            return False
        self.wait(finished, timeout=120)
        if len(self.arm['contacts']) != self.press_count:
            raise RuntimeError('The requested physical button contact was not observed')

    def cabin(self):
        return self.state['elevators'][self.route['elevator']]

    def run(self):
        self.wait(lambda: self.state is not None and self.pose is not None and self.state['ready'], timeout=240)
        if self.state.get('robot') != self.robot or self.state['scene'] != self.route['scene'] or math.dist(self.pose[:2], self.route['spawn'][:2]) > .5:
            raise RuntimeError('Start this route in a fresh matching scene at its default spawn')
        self.phase = 'before'
        for point in self.route['before']:
            self.go(point)
        if self.distance['before'] < 6:
            raise RuntimeError('Measured pre-elevator travel is under 6 m')
        self.phase = 'call'
        self.go(self.route['lobby'])
        if self.robot == 'locomanipulator':
            self.phase = 'hall_button'
            self.go(self.route['hall_dock'], tolerance=.025)
            self.face(DOCK_YAW)
            self.physical_press('hall', self.route['from_floor'])
            self.go(self.route['lobby'], hold_door=True)
            self.press('open')
        else:
            self.press('hall', self.route['from_floor'])
        self.wait(lambda: self.cabin()['floor'] == self.route['from_floor']-1 and self.cabin()['doorOpen'] > .99)
        self.phase = 'board'
        self.go(self.route['cabin'], hold_door=True)
        if self.robot == 'locomanipulator':
            self.phase = 'cabin_button'
            self.face(DOCK_YAW, hold_door=True)
            self.go([self.route['floor_dock'][0], self.route['floor_dock'][1]-.4],
                    hold_door=True, tolerance=.025, reverse=True)
            self.face(DOCK_YAW, hold_door=True)
            self.straight_to(self.route['floor_dock'][1], hold_door=True)
        else:
            self.phase = 'face_exit'
            self.face(-math.pi/2, hold_door=True)
        self.press('close')
        self.phase = 'closed_cabin'
        self.wait(lambda: self.cabin()['doorOpen'] < .01)
        closed_at = self.state['sim_time']
        self.wait(lambda: self.state['sim_time']-closed_at > 3)
        self.phase = 'ride'
        if self.robot == 'locomanipulator':
            self.physical_press('floor', self.route['to_floor'])
        else:
            self.press('floor', self.route['to_floor'])
        self.wait(lambda: self.cabin()['floor'] == self.route['to_floor']-1 and self.cabin()['doorOpen'] > .99)
        height = self.cabin()['position']
        base_height = .08 if self.robot == 'scout' else .15
        if abs(self.pose[2] - height - base_height) > .1:
            raise RuntimeError('Cabin arrived but the physical robot did not ride with it')
        arrived = self.state['sim_time']
        self.wait(lambda: self.state['sim_time'] - arrived >= 4, hold_door=True)
        if self.robot == 'locomanipulator':
            self.phase = 'face_exit'
            self.straight_to(self.route['cabin'][1], hold_door=True)
            self.go(self.route['cabin'], hold_door=True)
            self.face(-math.pi/2, hold_door=True)
        self.phase = 'exit'
        self.go(self.route['exit'], hold_door=True)
        self.phase = 'after'
        for point in self.route['after']:
            self.go(point)
        self.phase = 'complete'
        self.drive()
        if self.distance['after'] < 6:
            raise RuntimeError('Measured post-elevator travel is under 6 m')
        return {'passed':True, 'scene':self.route['scene'], 'robot':self.robot,
                'button_contacts':self.arm['contacts'] if self.arm else [], 'distance_m':self.distance,
                'arrival_floor':self.route['to_floor'], 'final_pose':self.pose, 'samples':self.samples}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', choices=('office','house','research'), default='office')
    parser.add_argument('--robot', choices=('amr','locomanipulator','scout'), default='amr')
    parser.add_argument('--file', type=Path)
    parser.add_argument('--speed', type=float, default=.45)
    parser.add_argument('--report', type=Path, default=Path('.runtime/logs/route.json'))
    args = parser.parse_args()
    if not math.isfinite(args.speed) or not .05 <= args.speed <= .6:
        parser.error('--speed must be 0.05..0.6 m/s')
    path = args.file or ROOT/'examples'/f'{args.scene}.json'
    if not path.is_file():
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('aprl_robot_sim'))/'examples'/f'{args.scene}.json'
    rclpy.init()
    node = Route(json.loads(path.read_text()), args.speed, args.robot)
    report = {'passed':False}
    code = 0
    try:
        report = node.run()
        print(f"PASS: {args.scene}: {report['distance_m']}, floor {report['arrival_floor']}", flush=True)
    except (RuntimeError, KeyboardInterrupt, ExternalShutdownException) as exc:
        report.update(error=str(exc), phase=node.phase, pose=node.pose, samples=node.samples)
        print(f'FAIL in {node.phase}: {exc}', flush=True)
        code = 1
    finally:
        if rclpy.ok():
            for _ in range(5):
                node.drive()
                rclpy.spin_once(node, timeout_sec=.03)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report,indent=2)+'\n')
    return code


if __name__ == '__main__':
    raise SystemExit(main())
