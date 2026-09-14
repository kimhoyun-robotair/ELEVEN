"""OpenUSD bindings for the simulator-independent elevator state machine.

This module deliberately has no Kit imports, so actual USD composition, door
transforms and selective lamp behavior can be validated without an RTX GPU.
All interactive changes go into the stage session layer by default.
"""

from __future__ import annotations

from dataclasses import dataclass

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from isaac_runtime.elevator import ElevatorController


def _value(prim, name, default=None):
    attr = prim.GetAttribute(name)
    value = attr.Get() if attr else None
    return default if value is None else value


class Motion:
    """A dedicated additive transform leaves the authored asset pose intact."""

    def __init__(self, prim):
        if not prim:
            raise ValueError("Required elevator motion prim is missing")
        self.prim = prim
        xform = UsdGeom.Xformable(prim)
        self.op = next(
            (op for op in xform.GetOrderedXformOps()
             if op.GetOpName() == "xformOp:translate:testbedMotion"), None
        )
        if self.op is None:
            self.op = xform.AddTranslateOp(
                precision=UsdGeom.XformOp.PrecisionDouble, opSuffix="testbedMotion"
            )
        self.set(0, 0, 0)

    def set(self, x=0.0, y=0.0, z=0.0):
        self.op.Set(Gf.Vec3d(float(x), float(y), float(z)))


@dataclass
class Button:
    path: str
    elevator_id: str
    action: str
    floor: int
    direction: str
    emissive_inputs: list
    was_lit: bool | None = None

    def show(self, lit):
        lit = bool(lit)
        if lit == self.was_lit:
            return
        color = Gf.Vec3f(3.0, 1.65, 0.32) if lit else Gf.Vec3f(0.0)
        for shader_input in self.emissive_inputs:
            shader_input.Set(color)
        self.was_lit = lit


class UsdElevatorRig:
    """One car, its independent landing doors, and its individually lit inputs."""

    def __init__(self, stage, root):
        self.stage = stage
        self.root = root
        self.path = str(root.GetPath())
        self.elevator_id = root.GetName()
        self.label = str(_value(root, "testbed:label", self.elevator_id))
        self.heights = list(_value(root, "testbed:floorHeights", []))
        if len(self.heights) < 2:
            raise ValueError(f"{self.path}: testbed:floorHeights needs at least two floors")
        self.floor_labels = list(_value(root, "testbed:floorLabels",
                                       [str(n + 1) for n in range(len(self.heights))]))
        if len(self.floor_labels) != len(self.heights):
            raise ValueError(f"{self.path}: testbed:floorLabels must match floorHeights")
        self.controller = ElevatorController(self.heights)
        self.travel = float(_value(root, "testbed:doorTravel", 0.72))
        self.cabin = Motion(stage.GetPrimAtPath(self.path + "/Cabin"))
        self.cabin_doors = self._doors("/Cabin/Doors")
        self.landing_doors = [self._doors(f"/LandingDoors/Floor_{n}")
                              for n in range(len(self.heights))]
        self.buttons = self._buttons()
        self.floor_displays = [(UsdGeom.Imageable(prim), int(_value(prim, "testbed:displayFloor")))
                               for prim in stage.Traverse()
                               if prim.HasAttribute("testbed:displayFloor")
                               and (_value(prim, "testbed:elevatorId") == self.elevator_id
                                    or prim.GetPath().HasPrefix(root.GetPath()))]
        self.manual_obstruction = False
        self._initial_z = self.heights[0]
        self.apply(self.controller.snapshot())

    def _doors(self, suffix):
        return tuple(Motion(self.stage.GetPrimAtPath(self.path + suffix + "/" + side))
                     for side in ("Left", "Right"))

    def _buttons(self):
        result = []
        for prim in Usd.PrimRange(self.root):
            action = _value(prim, "testbed:button")
            if not action:
                continue
            materials = []
            relationship = prim.GetRelationship("testbed:lightMaterial")
            if relationship and relationship.GetTargets():
                materials = [UsdShade.Material(self.stage.GetPrimAtPath(path))
                             for path in relationship.GetTargets()]
            else:
                material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
                if material:
                    materials = [material]
            emissions = []
            for material in materials:
                for child in Usd.PrimRange(material.GetPrim()):
                    shader = UsdShade.Shader(child)
                    if shader and shader.GetIdAttr().Get() == "UsdPreviewSurface":
                        emissions.append(shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f))
            if not emissions:
                raise ValueError(f"{prim.GetPath()}: button needs its own UsdPreviewSurface light material")
            result.append(Button(str(prim.GetPath()), self.elevator_id, str(action),
                                 int(_value(prim, "testbed:floor", 0)),
                                 str(_value(prim, "testbed:direction", "up")), emissions))
        return result

    def press(self, action, floor=0, direction="up"):
        if action == "floor":
            accepted = self.controller.request_floor(floor)
        elif action == "hall":
            accepted = self.controller.request_hall(floor, direction)
        elif action == "open":
            accepted = self.controller.open_doors()
        elif action == "close":
            accepted = self.controller.close_doors()
        elif action == "alarm":
            accepted = self.controller.press_alarm()
        else:
            raise ValueError(f"Unknown elevator action: {action}")
        self.apply(self.controller.snapshot())
        return accepted

    def update(self, dt, door_obstructed=False):
        self.controller.set_obstruction(self.manual_obstruction or door_obstructed)
        state = self.controller.tick(dt)
        self.apply(state)
        return state

    def apply(self, state):
        self.cabin.set(z=state["position"] - self._initial_z)
        opening = state["doorOpen"]
        self._slide(self.cabin_doors, opening)
        # Door interlock: only the stationary car's aligned landing can open.
        for floor, doors in enumerate(self.landing_doors):
            aligned = abs(state["position"] - self.heights[floor]) < 1e-4
            safe = aligned and abs(state["velocity"]) < 1e-6
            self._slide(doors, opening if safe else 0.0)
        for button in self.buttons:
            if button.action == "floor":
                lit = state["cabinLights"][button.floor]
            elif button.action == "hall":
                lit = state["hallLights"][button.floor][button.direction]
            else:
                key = {"open": "openButtonLit", "close": "closeButtonLit", "alarm": "alarmLit"}.get(button.action)
                lit = bool(state.get(key, False))
            button.show(lit)
        for display, floor in self.floor_displays:
            display.GetVisibilityAttr().Set(UsdGeom.Tokens.inherited if floor == state["floor"]
                                            else UsdGeom.Tokens.invisible)
        self.root.CreateAttribute("testbed:currentFloor", Sdf.ValueTypeNames.Int).Set(state["floor"])
        self.root.CreateAttribute("testbed:state", Sdf.ValueTypeNames.String).Set(state["state"])

    def _slide(self, doors, fraction):
        doors[0].set(x=-self.travel * fraction)
        doors[1].set(x=self.travel * fraction)

    def reset(self):
        self.controller = ElevatorController(self.heights)
        self.manual_obstruction = False
        self.apply(self.controller.snapshot())


class ElevatorScene:
    """Bind all cars and expose a prim-path input API for robots and GUI picking."""

    def __init__(self, stage, *, session_layer=True):
        self.stage = stage
        if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
            raise ValueError("Testbed worlds must be Z-up")
        if abs(UsdGeom.GetStageMetersPerUnit(stage) - 1.0) > 1e-9:
            raise ValueError("Testbed worlds must use metersPerUnit = 1")
        if session_layer:
            stage.SetEditTarget(stage.GetSessionLayer())
        parent = stage.GetPrimAtPath("/World/Elevators")
        if not parent:
            raise ValueError("No /World/Elevators in this stage")
        self.rigs = {root.GetName(): UsdElevatorRig(stage, root)
                     for root in parent.GetChildren()
                     if root.HasAttribute("testbed:floorHeights")}
        if not self.rigs:
            raise ValueError("No configured elevators found")
        self.buttons = {button.path: button for rig in self.rigs.values() for button in rig.buttons}
        self._check_distinct_lamps()

    def _check_distinct_lamps(self):
        owners = {}
        for button in self.buttons.values():
            for light in button.emissive_inputs:
                path = str(light.GetAttr().GetPath())
                if path in owners and owners[path] != button.path:
                    raise ValueError(f"Buttons share a lamp material: {owners[path]} and {button.path}")
                owners[path] = button.path

    def dispatch_press(self, prim_path):
        """Press a button or a descendant label/ring. Return False for non-buttons."""
        path = Sdf.Path(str(prim_path)).GetPrimPath()
        if path.isEmpty or not path.IsAbsolutePath():
            return False
        while path != Sdf.Path.absoluteRootPath:
            button = self.buttons.get(str(path))
            if button is not None:
                return self.rigs[button.elevator_id].press(button.action, button.floor, button.direction)
            path = path.GetParentPath()
        return False

    def reset(self):
        for rig in self.rigs.values():
            rig.reset()

    def validate_physics(self):
        """Fail early if a moving floor or door would be an animated static body."""
        errors = []
        for rig in self.rigs.values():
            paths = [rig.path + "/Cabin/Collision"]
            paths += [str(motion.prim.GetPath()) for pair in [rig.cabin_doors, *rig.landing_doors]
                      for motion in pair]
            for path in paths:
                prim = self.stage.GetPrimAtPath(path)
                api = UsdPhysics.RigidBodyAPI(prim) if prim else None
                if not api or not api.GetKinematicEnabledAttr().Get():
                    errors.append(f"{path}: requires kinematic UsdPhysics.RigidBodyAPI")
                if prim and not any(child.HasAPI(UsdPhysics.CollisionAPI) for child in Usd.PrimRange(prim)):
                    errors.append(f"{path}: no collision geometry")
        return errors
