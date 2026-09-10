"""Kit UI, viewport input and PhysX stepping for the multi-floor testbeds."""

from __future__ import annotations

import carb
import omni.kit.app
import omni.physx
import omni.timeline
import omni.ui as ui
import omni.usd
from pxr import Gf, Usd, UsdGeom

from isaac.usd_adapter import ElevatorScene


class TestbedRuntime:
    def __init__(self, *, ui_enabled=True, selection_presses=True):
        self.context = omni.usd.get_context()
        self.scene = ElevatorScene(self.context.get_stage())
        errors = self.scene.validate_physics()
        if errors:
            raise RuntimeError("Invalid moving collision bodies:\n" + "\n".join(errors))
        self.selection_presses = selection_presses
        self.ui_enabled = ui_enabled
        self.labels = {}
        self.window = None
        self._ui_elapsed = 0.0
        self._pending_reset = False
        self._warned_query = False
        self._query = omni.physx.get_physx_scene_query_interface()
        self._physics_subscription = omni.physx.get_physx_interface().subscribe_physics_on_step_events(
            self._on_physics, True, 0
        )
        self._stage_subscription = self.context.get_stage_event_stream().create_subscription_to_pop(
            self._on_stage_event, name="testbed_button_selection"
        )
        self._timeline_subscription = omni.timeline.get_timeline_interface().get_timeline_event_stream().create_subscription_to_pop(
            self._on_timeline, name="testbed_elevator_reset"
        )
        self._update_subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._on_update, name="testbed_control_panel"
        )
        if ui_enabled:
            self._build_panel()

    def _on_timeline(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            # Reset on the next Kit update, outside the physics/timeline callback.
            self._pending_reset = True

    def _on_stage_event(self, event):
        if event.type == int(omni.usd.StageEventType.SELECTION_CHANGED) and self.selection_presses:
            for path in self.context.get_selection().get_selected_prim_paths():
                self.scene.dispatch_press(path)

    def _door_occupied(self, rig):
        state = rig.controller.snapshot()
        if state["state"] == "moving":
            return False
        matrix = UsdGeom.Xformable(rig.root).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        center = matrix.Transform(Gf.Vec3d(0.0, -1.20, state["position"] + 1.10))
        rotation = matrix.ExtractRotationQuat()
        imaginary = rotation.GetImaginary()
        blocked = False

        def report(hit):
            nonlocal blocked
            path = str(hit.collision)
            if not path.startswith(rig.path + "/"):
                blocked = True
                return False
            return True

        try:
            self._query.overlap_box(
                carb.Float3(0.64, 0.25, 0.95), carb.Float3(*center),
                carb.Float4(*imaginary, rotation.GetReal()), report, False
            )
        except Exception as error:
            # A failed sensor must not authorize closing. Report once and hold.
            if not self._warned_query:
                carb.log_error(f"Testbed door obstruction query failed; holding doors open: {error}")
                self._warned_query = True
            return True
        return blocked

    def _on_physics(self, dt):
        for rig in self.scene.rigs.values():
            rig.update(float(dt), self._door_occupied(rig))

    def _on_update(self, event):
        if self._pending_reset:
            self.reset()
        self._ui_elapsed += float(event.payload.get("dt", 0.0))
        if self._ui_elapsed < 0.10:
            return
        self._ui_elapsed = 0.0
        for name, label in self.labels.items():
            state = self.scene.rigs[name].controller.snapshot()
            target = "—" if state["targetFloor"] is None else str(state["targetFloor"] + 1)
            queued = ", ".join(str(floor + 1) for floor in state["queue"]) or "none"
            label.text = (f"Floor {state['floor'] + 1} → {target} | {state['state']}\n"
                          f"Height {state['position']:.2f} m | Door {state['doorOpen']:.0%}\n"
                          f"Requested: {queued} | Door beam: {'occupied' if state['obstruction'] else 'clear'}")

    def press(self, elevator_id, action, floor=0, direction="up"):
        """Public API. Floors are zero-based; labels in the UI are one-based."""
        return self.scene.rigs[elevator_id].press(action, floor, direction)

    def dispatch_press(self, prim_path):
        """Use the collision/picking prim path reported by your robot or raycaster."""
        return self.scene.dispatch_press(prim_path)

    def reset(self):
        self.scene.reset()
        self._pending_reset = False

    def _build_panel(self):
        self.window = ui.Window("Multi-floor testbed", width=395, height=740)
        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=9, height=0):
                    ui.Label("Elevator controls", height=26, style={"font_size": 22})
                    ui.Label("Click a modeled button in the viewport, or use this panel.\n"
                             "Floor labels are 1-based. Only requested buttons illuminate.",
                             height=38, word_wrap=True)
                    selection_model = ui.SimpleBoolModel(self.selection_presses)
                    with ui.HStack(height=24):
                        ui.CheckBox(selection_model, width=24)
                        ui.Label("Viewport selection presses modeled buttons")
                    selection_model.add_value_changed_fn(
                        lambda model: setattr(self, "selection_presses", model.as_bool))
                    for elevator_id, rig in self.scene.rigs.items():
                        with ui.CollapsableFrame(rig.label, collapsed=False):
                            with ui.VStack(spacing=5, height=0):
                                self.labels[elevator_id] = ui.Label("Ready", height=58, word_wrap=True)
                                ui.Label("Cabin destinations", height=20)
                                with ui.HStack(height=30, spacing=4):
                                    for floor in range(len(rig.heights)):
                                        ui.Button(str(floor + 1), clicked_fn=lambda e=elevator_id, f=floor: self.press(e, "floor", f))
                                with ui.HStack(height=28, spacing=4):
                                    ui.Button("Open ◀▶", clicked_fn=lambda e=elevator_id: self.press(e, "open"))
                                    ui.Button("Close ▶◀", clicked_fn=lambda e=elevator_id: self.press(e, "close"))
                                    ui.Button("Alarm", clicked_fn=lambda e=elevator_id: self.press(e, "alarm"))
                                ui.Label("Landing calls", height=20)
                                for floor in range(len(rig.heights)):
                                    with ui.HStack(height=25, spacing=4):
                                        ui.Label(f"Floor {floor + 1}", width=70)
                                        if floor < len(rig.heights) - 1:
                                            ui.Button("↑ Call", clicked_fn=lambda e=elevator_id, f=floor: self.press(e, "hall", f, "up"))
                                        if floor > 0:
                                            ui.Button("↓ Call", clicked_fn=lambda e=elevator_id, f=floor: self.press(e, "hall", f, "down"))
                                        ui.Button("View lobby", clicked_fn=lambda e=elevator_id, f=floor: self.view_lobby(e, f))
                                with ui.HStack(height=25):
                                    obstruction_model = ui.SimpleBoolModel(False)
                                    ui.CheckBox(obstruction_model, width=24)
                                    ui.Label("Hold door beam occupied (test input)")
                                obstruction_model.add_value_changed_fn(
                                    lambda model, r=rig: setattr(r, "manual_obstruction", model.as_bool))
                                ui.Button("Ride / inspect cabin", height=28,
                                          clicked_fn=lambda e=elevator_id: self.view_cabin(e))
                    ui.Label("Alarm is a local indicator; it does not contact a service.\n"
                             "Pause freezes simulation. Stop resets cars and requests.",
                             height=36, word_wrap=True)

    def _view(self, path, eye, target):
        from omni.kit.viewport.utility import get_active_viewport

        camera = UsdGeom.Camera.Define(self.scene.stage, path)
        camera.CreateFocalLengthAttr(20.0)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.03, 300.0))
        xform = UsdGeom.Xformable(camera)
        xform.ClearXformOpOrder()
        matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
        op = camera.GetPrim().GetAttribute("xformOp:transform")
        if op:
            matrix_op = UsdGeom.XformOp(op)
            xform.SetXformOpOrder([matrix_op])
        else:
            matrix_op = xform.AddTransformOp()
        matrix_op.Set(matrix)
        viewport = get_active_viewport()
        if viewport:
            viewport.camera_path = path

    def view_cabin(self, elevator_id):
        rig = self.scene.rigs[elevator_id]
        # This camera is parented to the cabin: a visual ride, independent of robots.
        self._view(rig.path + "/Cabin/InspectionCamera", (0, -0.85, 1.65), (0.70, 0.45, 1.35))

    def view_lobby(self, elevator_id, floor=0):
        rig = self.scene.rigs[elevator_id]
        matrix = UsdGeom.Xformable(rig.root).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        eye = matrix.Transform(Gf.Vec3d(0, -5.0, rig.heights[floor] + 1.65))
        target = matrix.Transform(Gf.Vec3d(0, 0, rig.heights[floor] + 1.35))
        self._view("/World/TestbedViewCamera", eye, target)

    def destroy(self):
        self._physics_subscription = None
        self._stage_subscription = None
        self._timeline_subscription = None
        self._update_subscription = None
        if self.window:
            self.window.destroy()
        self.window = None


_active_runtime = None


def attach(*, ui_enabled=True, selection_presses=True):
    """Attach to the already open testbed stage from Isaac Sim's Script Editor."""
    global _active_runtime
    if _active_runtime is not None:
        _active_runtime.destroy()
    _active_runtime = TestbedRuntime(ui_enabled=ui_enabled, selection_presses=selection_presses)
    return _active_runtime
