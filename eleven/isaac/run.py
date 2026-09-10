#!/usr/bin/env python3
"""Run with Isaac Sim 5.1.0's python.sh; see docs/isaac_sim.md."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", choices=("office", "house"), default="office")
    parser.add_argument("--usd", type=Path, help="Override the scene USD path")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--renderer", choices=("RaytracedLighting", "PathTracing"), default="RaytracedLighting")
    parser.add_argument("--steps", type=int, default=0, help="Exit after N physics steps; 0 runs until closed")
    parser.add_argument("--request", action="append", default=[], metavar="E1:3",
                        help="Request a cabin destination at startup (1-based floor); repeatable")
    parser.add_argument("--state-output", type=Path, help="Write final elevator state JSON")
    parser.add_argument("--verify-ride", action="store_true",
                        help="Place dynamic passenger cubes in all cars, travel to top floors and assert final heights")
    args, kit_args = parser.parse_known_args()
    if args.steps < 0:
        parser.error("--steps must be nonnegative")
    if args.verify_ride and not args.steps:
        args.steps = 1800
    usd_path = (args.usd or ROOT / "scenes" / f"{args.scene}.usda").resolve()
    if not usd_path.is_file():
        parser.error(f"Scene does not exist: {usd_path}")
    # Keep only unknown Kit arguments: our --scene/--request flags are not Kit settings.
    sys.argv = [sys.argv[0], *kit_args]
    try:
        from isaacsim import SimulationApp
    except ImportError as error:
        raise SystemExit("Run this script with Isaac Sim 5.1.0's python.sh (or python.bat).") from error

    application = SimulationApp({
        "headless": args.headless, "renderer": args.renderer,
        "width": 1600, "height": 1000, "samples_per_pixel_per_frame": 64,
        "max_bounces": 8, "max_specular_transmission_bounces": 8,
    })
    runtime = None
    try:
        # Isaac/Omniverse imports must occur after SimulationApp initialization.
        import omni.usd
        from isaacsim.core.api import SimulationContext
        from isaacsim.core.utils.stage import is_stage_loading, open_stage
        from isaac.runtime import attach

        if not open_stage(str(usd_path)):
            raise RuntimeError(f"Could not load {usd_path}")
        application.update()
        application.update()
        while is_stage_loading():
            application.update()
        application.reset_render_settings()
        simulation = SimulationContext(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0,
                                       stage_units_in_meters=1.0, physics_prim_path="/World/PhysicsScene")
        runtime = attach(ui_enabled=not args.headless, selection_presses=not args.headless)
        passengers = {}
        if args.verify_ride:
            from pxr import Gf, Usd, UsdGeom, UsdPhysics

            for name, rig in runtime.scene.rigs.items():
                parent_transform = UsdGeom.Xformable(rig.root).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                position = parent_transform.Transform(Gf.Vec3d(0, 0, rig.heights[0] + 0.3))
                cube = UsdGeom.Cube.Define(runtime.scene.stage, f"/World/RideVerification/{name}")
                cube.CreateSizeAttr(0.30)
                cube.CreateDisplayColorAttr([Gf.Vec3f(0.08, 0.5, 0.9)])
                cube.AddTranslateOp().Set(position)
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
                UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
                UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(1.0)
                passengers[name] = cube
        simulation.initialize_physics()
        # Drain initialization timeline events before registering user requests.
        application.update()
        runtime.reset()
        for request in args.request:
            elevator_id, floor = request.split(":", 1)
            runtime.press(elevator_id, "floor", int(floor) - 1)
        if args.verify_ride:
            for name, rig in runtime.scene.rigs.items():
                runtime.press(name, "floor", len(rig.heights) - 1)
        if not args.headless:
            runtime.view_lobby(next(iter(runtime.scene.rigs)), 0)
        simulation.play()
        step = 0
        while application.is_running() and (not args.steps or step < args.steps):
            simulation.step(render=not args.headless)
            if simulation.is_playing():
                step += 1
            elif args.headless:
                break
        if args.state_output:
            args.state_output.parent.mkdir(parents=True, exist_ok=True)
            args.state_output.write_text(json.dumps({name: rig.controller.snapshot()
                                                    for name, rig in runtime.scene.rigs.items()}, indent=2) + "\n")
        if args.verify_ride:
            failures = []
            for name, cube in passengers.items():
                rig = runtime.scene.rigs[name]
                state = rig.controller.snapshot()
                actual = cube.ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
                parent_transform = UsdGeom.Xformable(rig.root).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                expected = parent_transform.Transform(Gf.Vec3d(0, 0, rig.heights[-1] + 0.15))
                error = (actual - expected).GetLength()
                if error > 0.06:
                    failures.append(f"{name}: passenger displacement error {error:.4f} m (limit 0.06 m)")
                if state["floor"] != len(rig.heights) - 1 or state["queue"]:
                    failures.append(f"{name}: trip incomplete: {state}")
                if any(state["cabinLights"]):
                    failures.append(f"{name}: cabin request lamps remained lit after service")
            if failures:
                raise AssertionError("Ride verification failed:\n" + "\n".join(failures))
            print(f"PASS: {len(passengers)} dynamic passenger cubes carried to top floors; lamps cleared.")
    finally:
        if runtime is not None:
            runtime.destroy()
        application.close()


if __name__ == "__main__":
    main()
