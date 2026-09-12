"""Four viewport modes and an optional 360-degree robot inspection camera."""
import math

import omni.ui as ui
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, Usd, UsdGeom


class RobotViews:
    def __init__(self, stage, tall=False, eye_path='/World/Robot/base_link/front_camera_link/camera'):
        self.tall = tall
        self.eye_path = eye_path
        self.stage, self.mode = stage, 'follow'
        self.path = '/World/RobotView'
        camera = UsdGeom.Camera.Define(stage, self.path)
        camera.CreateFocalLengthAttr(22.0)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.03, 300))
        self.xform = UsdGeom.Xformable(camera)
        self.xform.MakeMatrixXform()
        self.window = ui.Window('Robot view', width=360, height=170, position_x=1030, position_y=45)
        with self.window.frame:
            with ui.VStack(spacing=6):
                with ui.HStack(spacing=4):
                    ui.Button('Robot eye', clicked_fn=lambda: self.select('robot'))
                    ui.Button('Free view', clicked_fn=lambda: self.select('free'))
                with ui.HStack(spacing=4):
                    ui.Button('Go to robot', clicked_fn=lambda: self.select('focus'))
                    ui.Button('Follow robot', clicked_fn=lambda: self.select('follow'))
                self.label = ui.Label('Follow robot', height=25)
                ui.Button('360° inspection', clicked_fn=lambda: self.select('orbit'))
        self.update(0)
        self.select('follow')

    def select(self, mode):
        viewport = get_active_viewport()
        if not viewport:
            return
        # Copy the current pose before freeing the camera, preserving the user's view.
        if mode == 'free':
            current = self.stage.GetPrimAtPath(str(viewport.camera_path))
            if current:
                self.xform.MakeMatrixXform().Set(UsdGeom.Xformable(current).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        self.mode = mode
        if mode == 'robot':
            viewport.camera_path = self.eye_path
        else:
            viewport.camera_path = self.path
            self.update(0)
        self.label.text = {'robot':'Robot eye', 'free':'Free view', 'focus':'Go to robot',
                           'follow':'Follow robot', 'orbit':'360° inspection'}[mode]
        if mode == 'focus':
            self.mode = 'free'

    def update(self, seconds):
        if self.mode in ('robot', 'free'):
            return
        base = self.stage.GetPrimAtPath('/World/Robot/base_link')
        matrix = UsdGeom.Xformable(base).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        target = matrix.Transform(Gf.Vec3d(0, 0, .62 if self.tall else .10))
        if self.mode == 'orbit':
            angle = seconds * math.tau / 12
            eye = target + Gf.Vec3d((2.8 if self.tall else 1.55)*math.cos(angle), (2.8 if self.tall else 1.55)*math.sin(angle), 0.85)
        else:
            eye = matrix.Transform(Gf.Vec3d(-2.1, -1.8, 1.6))
        self.xform.MakeMatrixXform().Set(Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse())

    def close(self):
        self.window.destroy()
