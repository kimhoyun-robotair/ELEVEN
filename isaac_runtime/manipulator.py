"""NERO joint servos and contact-confirmed elevator button presses."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema, PhysicsSchemaTools
import omni.physx
from isaacsim.core.utils.types import ArticulationAction
from sensor_msgs.msg import JointState
from std_msgs.msg import String

ROOT = '/World/Robot'


def matrix(origin):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')).as_matrix()
    result[:3, 3] = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
    return result


def prepare_buttons(stage):
    bounds = UsdGeom.BBoxCache(0, ['default', 'render'])
    for prim in list(stage.Traverse()):
        if not prim.HasAttribute('testbed:button'):
            continue
        lamp = next(child for child in prim.GetChildren() if child.GetName().endswith('_Lamp'))
        box = bounds.ComputeWorldBound(lamp).ComputeAlignedBox()
        center = box.GetMidpoint()
        center[1] = box.GetMin()[1]
        local = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).GetInverse().Transform(center)
        switch = UsdGeom.Cube.Define(stage, str(prim.GetPath()) + '/contact')
        switch.CreateSizeAttr(1)
        switch.AddTranslateOp().Set(local)
        switch.AddScaleOp().Set(Gf.Vec3f(.038, .004, .038))
        switch.CreateVisibilityAttr('invisible')
        UsdPhysics.CollisionAPI.Apply(switch.GetPrim())
        UsdPhysics.RigidBodyAPI.Apply(switch.GetPrim()).CreateKinematicEnabledAttr(True)
        collision = PhysxSchema.PhysxCollisionAPI.Apply(switch.GetPrim())
        collision.CreateContactOffsetAttr(.001)
        collision.CreateRestOffsetAttr(0)


class Manipulator:
    def __init__(self, stage, robot, node, cfg, elevators):
        self.stage, self.robot, self.node, self.cfg, self.elevators = stage, robot, node, cfg, elevators
        self.names = [f'nero_joint{i}' for i in range(1, 8)]
        self.indices = np.array([robot.get_dof_index(n) for n in self.names])
        self.fingers = np.array([robot.get_dof_index(f'nero_gripper_joint{i}') for i in (1, 2)])
        self.home = np.array(cfg['home_rad'], dtype=float)
        self.target, self.commanded = self.home.copy(), self.home.copy()
        self.aperture = 0.0
        robot.set_joint_positions(self.home, joint_indices=self.indices)
        robot.set_joint_positions(np.zeros(2), joint_indices=self.fingers)
        urdf = ET.parse(Path(__file__).resolve().parents[1] / 'src/nero_description/urdf/nero_with_gripper_description.urdf')
        joints = [urdf.find(f"joint[@name='joint{i}']") for i in range(1, 8)]
        self.origins = [matrix(j.find('origin')) for j in joints]
        self.lower = np.array([float(j.find('limit').get('lower')) for j in joints])
        self.upper = np.array([float(j.find('limit').get('upper')) for j in joints])
        self.tool = matrix(urdf.find("joint[@name='gripper_flange_joint']/origin")) @ matrix(urdf.find("joint[@name='gripper_base_joint']/origin"))
        self.tool[:3, 3] += self.tool[:3, :3] @ np.array([0, 0, .225])
        self.tip_path = ROOT + '/nero/nero_gripper_base/button_tip'
        self.phase, self.error, self.button, self.request_id = 'idle', '', None, ''
        self.contact = None
        self.events = []
        self.elapsed = 0.0
        self.publisher = node.create_publisher(String, '/arm/state', 10)
        node.create_subscription(JointState, '/arm/joint_command', self.joint_command, 10)
        node.create_subscription(String, '/arm/press_button', self.press, 10)
        self.contacts = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(self.on_contact)

    @property
    def busy(self):
        return self.phase in ('approach', 'press', 'retract', 'home')

    def fk(self, q):
        transform = np.eye(4)
        for origin, angle in zip(self.origins, q):
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_rotvec([0, 0, angle]).as_matrix()
            transform = transform @ origin @ rotation
        return transform @ self.tool

    def world(self, path):
        return UsdGeom.Xformable(self.stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    def solve(self, world_point, seed):
        mount = self.world(ROOT + '/nero/nero_base_link').GetInverse()
        target = np.array(mount.Transform(Gf.Vec3d(*world_point)))
        direction = np.array(mount.TransformDir(Gf.Vec3d(0, 1, 0)))
        def residual(q):
            tool = self.fk(q)
            return np.r_[tool[:3, 3]-target, .25*(tool[:3, 2]-direction), .001*(q-seed)]
        candidates = (seed, self.home, np.zeros(7))
        for initial in candidates:
            result = least_squares(residual, np.clip(initial, self.lower+1e-6, self.upper-1e-6),
                                   bounds=(self.lower, self.upper), max_nfev=160)
            tool = self.fk(result.x)
            if np.linalg.norm(tool[:3, 3]-target) <= .004 and np.dot(tool[:3, 2], direction) >= .985:
                return result.x
        raise ValueError('Button is outside the current arm workspace; move the base closer and face the panel')


    def joint_command(self, msg):
        try:
            if self.busy:
                raise ValueError('Button motion is active')
            if not msg.name or len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
                raise ValueError('Supply unique joint names and matching positions')
            target, aperture = self.target.copy(), self.aperture
            for name, value in zip(msg.name, msg.position):
                if not np.isfinite(value):
                    raise ValueError('Nonfinite joint target')
                if name == 'gripper':
                    if not 0 <= value <= .1:
                        raise ValueError('Gripper aperture must be 0..0.1 m')
                    aperture = value
                elif name in self.names:
                    index = self.names.index(name)
                    if not self.lower[index] <= value <= self.upper[index]:
                        raise ValueError(f'{name} exceeds the URDF limit')
                    target[index] = value
                else:
                    raise ValueError(f'Unknown arm joint: {name}')
            self.target, self.aperture = target, aperture
            self.phase, self.error = 'idle', ''
        except ValueError as exc:
            self.node.get_logger().warning(str(exc))

    def press(self, msg):
        if self.busy:
            return
        try:
            request = json.loads(msg.data)
            self.request_id = str(request.get('request_id', ''))
            if self.elevators is None:
                raise ValueError('No elevator in inspection mode')
            floor = request.get('floor', 1)
            if type(floor) is not int:
                raise ValueError('Floor must be an integer')
            self.button = next((b for b in self.elevators.scene.buttons.values()
                                if b.elevator_id == request['id'] and b.action == request['action']
                                and b.floor == floor-1 and b.direction == request.get('direction', 'up')), None)
            if self.button is None:
                raise ValueError('Button does not exist')
            switch = self.world(self.button.path + '/contact').ExtractTranslation()
            self.pre_point = np.array(switch) + [0, -.075, 0]
            self.press_point = np.array(switch) + [0, -.007, 0]
            measured = np.array(self.robot.get_joint_positions())[self.indices]
            self.pre = self.solve(self.pre_point, measured)
            self.pushed = self.solve(self.press_point, self.pre)
            self.target, self.phase, self.elapsed = self.pre, 'approach', 0.0
            self.contact, self.error = None, ''
            self.aperture = 0.0
        except (ValueError, KeyError, TypeError) as exc:
            self.phase, self.error = 'failed', str(exc)
            self.node.get_logger().warning(self.error)

    def on_contact(self, headers, data):
        if self.phase != 'press' or self.contact is not None:
            return
        for header in headers:
            pair = {str(PhysicsSchemaTools.intToSdfPath(header.collider0)),
                    str(PhysicsSchemaTools.intToSdfPath(header.collider1))}
            if {self.tip_path, self.button.path + '/contact'} != pair or not header.num_contact_data:
                continue
            point = data[header.contact_data_offset]
            if np.linalg.norm(point.impulse) < .001:
                continue
            self.contact = {'button': self.button.path, 'point_m': list(point.position),
                            'impulse_n_s': list(point.impulse), 'separation_m': float(point.separation)}

    def step(self, dt, seconds):
        self.elapsed += dt
        if self.busy and self.elapsed > 35:
            self.phase, self.error, self.target = 'failed', 'Button contact/motion timed out', self.home.copy()
        actual = np.asarray(self.robot.get_joint_positions())[self.indices]
        settled = np.max(np.abs(actual-self.target)) < .018
        if self.phase == 'approach' and settled:
            self.target, self.phase = self.pushed, 'press'
        elif self.phase == 'press' and self.contact:
            if self.elevators.scene.dispatch_press(self.button.path):
                self.contact['sim_time'] = seconds
                self.events.append(self.contact)
                self.target, self.phase = self.pre, 'retract'
            else:
                self.phase, self.error, self.target = 'failed', 'Elevator rejected the physical press', self.pre
        elif self.phase == 'retract' and settled:
            self.target, self.phase = self.home.copy(), 'home'
        elif self.phase == 'home' and settled:
            self.phase = 'complete'
        delta = self.cfg['joint_speed_rps'] * dt
        self.commanded += np.clip(self.target-self.commanded, -delta, delta)
        self.robot.apply_action(ArticulationAction(joint_positions=self.commanded, joint_indices=self.indices))
        self.robot.apply_action(ArticulationAction(joint_positions=np.array([.5, -.5])*self.aperture, joint_indices=self.fingers))

    def publish(self):
        self.publisher.publish(String(data=json.dumps({'phase': self.phase, 'error': self.error,
            'request_id': self.request_id, 'contacts': self.events,
            'tip_position_m': list(self.world(self.tip_path).ExtractTranslation())})))

    def close(self):
        self.contacts = None
