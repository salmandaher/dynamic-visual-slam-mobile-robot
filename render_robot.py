#!/usr/bin/env python3
"""Render a few views of a scene to PNG (Isaac Sim 5.1.0), after settling physics
so you can SEE the robot resting on its wheels/casters. Used to confirm ball-caster
placement and (optionally) compare robots.

    ./python.sh render_robot.py --scene <usd> --tag <name>
writes  renders/<tag>_top.png  <tag>_iso.png  <tag>_caster.png
"""
import argparse, math, os, sys

ap = argparse.ArgumentParser()
ap.add_argument("--scene", default="/home/salman/Documents/simsim/new_built_robot_ballcaster.usd")
ap.add_argument("--tag", default="ballcaster")
ap.add_argument("--settle", type=int, default=90, help="physics steps before capture")
args, _ = ap.parse_known_args()

OUT = "/home/salman/Documents/simsim/renders"
os.makedirs(OUT, exist_ok=True)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.usd
import omni.replicator.core as rep
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.stage import is_stage_loading


def log(m): print(f"[render] {m}", flush=True)


def main():
    omni.usd.get_context().open_stage(args.scene, None)
    simulation_app.update(); simulation_app.update()
    while is_stage_loading():
        simulation_app.update()
    log(f"opened {args.scene}")

    # settle on the ground so casters/wheels are loaded and resting
    sim = SimulationContext(physics_dt=1/120.0, rendering_dt=1/60.0, stage_units_in_meters=1.0)
    sim.play()
    for _ in range(max(1, args.settle)):
        sim.step(render=True)
    log("settled")

    views = {
        "top":    dict(position=(0.03, 0.0, 0.9),  look_at=(0.03, 0.0, 0.05)),  # straight down
        "iso":    dict(position=(0.55, -0.55, 0.4), look_at=(0.03, 0.0, 0.05)),  # 3/4 view
        "side":   dict(position=(0.9, 0.0, 0.1),   look_at=(0.0, 0.0, 0.05)),   # side-on (see pitch)
    }
    for name, v in views.items():
        try:
            cam = rep.create.camera(position=v["position"], look_at=v["look_at"])
            rp = rep.create.render_product(cam, (1280, 720))
            annot = rep.AnnotatorRegistry.get_annotator("rgb")
            annot.attach(rp)
            for _ in range(25):           # let RTX accumulate so it's not noisy/black
                rep.orchestrator.step(rt_subframes=8)
            data = annot.get_data()
            arr = np.asarray(data)
            if arr.size == 0:
                log(f"{name}: empty frame"); continue
            rgb = arr[..., :3].astype(np.uint8)
            path = os.path.join(OUT, f"{args.tag}_{name}.png")
            try:
                from PIL import Image
                Image.fromarray(rgb).save(path)
            except Exception:
                import imageio; imageio.imwrite(path, rgb)
            log(f"wrote {path}  shape={rgb.shape}  mean={rgb.mean():.1f}")
            annot.detach()
        except Exception as e:  # noqa: BLE001
            import traceback; log(f"{name}: FAILED {e!r}"); print(traceback.format_exc())
    sim.stop()
    return 0


if __name__ == "__main__":
    try:
        main()
    except BaseException as exc:  # noqa: BLE001
        import traceback; print("[render] FATAL", repr(exc)); print(traceback.format_exc())
    finally:
        simulation_app.close()
