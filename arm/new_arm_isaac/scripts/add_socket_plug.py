#!/usr/bin/env python3
"""Add the plug to an EXISTING new_arm.usd and write a standalone socket.usd — no re-import.

Idempotent. Mirrors scripts/add_ee_camera.py. The PLUG is baked as a rigid child of
end_effector_link inside new_arm.usd (so it follows the gripper). The SOCKET is static world
geometry, written to its own socket.usd (config.SOCKET_USD_PATH) so the demo / Replicator SDG
can reference it independently of the robot.

Run with Isaac Sim 5.1.0's bundled pxr (no SimulationApp boot needed):
    cd /media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64
    PKG=$(ls -d extscache/omni.usd.libs-*lx64.r.cp311 | head -1)
    PYTHONPATH="$PWD/$PKG:$PYTHONPATH" LD_LIBRARY_PATH="$PWD/$PKG/bin:$LD_LIBRARY_PATH" \
      ./python.sh /home/salman/Documents/our_work/new_arm_isaac/scripts/add_socket_plug.py --verify
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from new_arm_isaac import config  # noqa: E402
from new_arm_isaac.usd_socket_plug import bake_plug, bake_socket  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description="Add plug to new_arm.usd and write socket.usd")
    parser.add_argument("--usd", default=config.USD_PATH)
    parser.add_argument("--socket-usd", default=config.SOCKET_USD_PATH)
    parser.add_argument("--verify", action="store_true")
    args, _ = parser.parse_known_args()

    from pxr import Usd, UsdGeom

    if not os.path.exists(args.usd):
        print(f"ERROR: USD not found: {args.usd}; run import_urdf.py first.", file=sys.stderr)
        return 1

    # 1) Plug -> bake into the robot USD under end_effector_link.
    stage = Usd.Stage.Open(args.usd)
    dp = stage.GetDefaultPrim()
    top = dp.GetPath().pathString if dp else "/new_arm"
    print(f"[add_socket_plug] opened {args.usd} (defaultPrim {top})", flush=True)
    plug_path = bake_plug(stage, top, log=lambda m: print(m, flush=True))
    if not plug_path:
        return 1
    stage.GetRootLayer().Save()
    print(f"[add_socket_plug] saved {args.usd}", flush=True)

    # 2) Socket -> standalone socket.usd (defaultPrim = /Socket, so it can be referenced).
    os.makedirs(os.path.dirname(args.socket_usd), exist_ok=True)
    s_stage = Usd.Stage.CreateNew(args.socket_usd) if not os.path.exists(args.socket_usd) \
        else Usd.Stage.Open(args.socket_usd)
    sock_path = bake_socket(s_stage, "/Socket", log=lambda m: print(m, flush=True))
    s_stage.SetDefaultPrim(s_stage.GetPrimAtPath("/Socket"))
    UsdGeom.SetStageUpAxis(s_stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(s_stage, 1.0)
    s_stage.GetRootLayer().Save()
    print(f"[add_socket_plug] wrote {args.socket_usd} ({sock_path})", flush=True)

    # 3) Hole-centred socket marker -> socket_marker.usd (per-env RL VisualizationMarker).
    os.makedirs(os.path.dirname(config.SOCKET_MARKER_USD_PATH), exist_ok=True)
    m_stage = Usd.Stage.CreateNew(config.SOCKET_MARKER_USD_PATH) \
        if not os.path.exists(config.SOCKET_MARKER_USD_PATH) else Usd.Stage.Open(config.SOCKET_MARKER_USD_PATH)
    mk_path = bake_socket(m_stage, "/SocketMarker", log=lambda m: print(m, flush=True), marker=True)
    m_stage.SetDefaultPrim(m_stage.GetPrimAtPath("/SocketMarker"))
    UsdGeom.SetStageUpAxis(m_stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(m_stage, 1.0)
    m_stage.GetRootLayer().Save()
    print(f"[add_socket_plug] wrote {config.SOCKET_MARKER_USD_PATH} ({mk_path})", flush=True)

    if args.verify:
        v = Usd.Stage.Open(args.usd)
        plug = v.GetPrimAtPath(plug_path)
        kids = [c.GetName() for c in plug.GetChildren()]
        print(f"[verify] plug prim type={plug.GetTypeName()} children={kids}", flush=True)
        sv = Usd.Stage.Open(args.socket_usd)
        sp = sv.GetPrimAtPath("/Socket")
        skids = [f"{c.GetName()}({c.GetTypeName()})" for c in sp.GetChildren()]
        print(f"[verify] socket defaultPrim={sv.GetDefaultPrim().GetPath()} children={skids}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
