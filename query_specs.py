#!/usr/bin/env python3
"""Print the robot base + wheel specs from the composed (scaled) lizard.usd."""
import math
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.usd
from pxr import UsdGeom, Usd, Gf
from isaacsim.core.utils.stage import is_stage_loading
_OUT=open('/home/salman/Documents/simsim/specs_out.txt','w')
def w(*a):
    _OUT.write(' '.join(str(x) for x in a)+'\n'); _OUT.flush()

SCENE = "/home/salman/Documents/simsim/lizard.usd"
P = "/World/new_built_robot_ros/robot"
omni.usd.get_context().open_stage(SCENE, None)
app.update(); app.update()
while is_stage_loading():
    app.update()
stage = omni.usd.get_context().get_stage()
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                         [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])


def aabb(path):
    rng = cache.ComputeWorldBound(stage.GetPrimAtPath(path)).ComputeAlignedRange()
    mn, mx = rng.GetMin(), rng.GetMax()
    return mn, mx, (mx - mn), (mn + mx) * 0.5


def mass(path):
    a = stage.GetPrimAtPath(path).GetAttribute("physics:mass")
    return a.Get() if a and a.Get() is not None else None


w("\n================ ROBOT BASE ================")
mn, mx, sz, ctr = aabb(P + "/robot_base")
w(f"mass            : {mass(P + '/robot_base')} kg")
w(f"dimensions LxWxH: {sz[0]*1000:.1f} x {sz[1]*1000:.1f} x {sz[2]*1000:.1f} mm "
      f"({sz[0]:.4f} x {sz[1]:.4f} x {sz[2]:.4f} m)")
w(f"AABB z range    : {mn[2]*1000:.1f} .. {mx[2]*1000:.1f} mm")
w(f"geom. centre/COM: ({ctr[0]*1000:.1f}, {ctr[1]*1000:.1f}, {ctr[2]*1000:.1f}) mm (world)")

wheels = [("right_back  (_9055)",    "_9055_txm_4_inch_wheel"),
          ("left_back   (_9055_01)", "_9055_txm_4_inch_wheel_01"),
          ("right_front (_6466)",    "_6466_txm_4_inch_dual_omni_wheel"),
          ("left_front  (_6466_01)", "_6466_txm_4_inch_dual_omni_wheel_01")]
ctrs = {}
w("\n================ WHEELS ================")
for name, sub in wheels:
    mn, mx, sz, c = aabb(P + "/" + sub)
    ctrs[name.split()[0]] = c
    dims = sorted([sz[0], sz[1], sz[2]], reverse=True)
    dia, width = dims[0], dims[2]
    w(f"{name}: mass {mass(P + '/' + sub)} kg | dia {dia*1000:.1f} mm | width {width*1000:.1f} mm "
          f"| centre ({c[0]*1000:.0f},{c[1]*1000:.0f},{c[2]*1000:.0f}) mm")

w("\n================ TRACK / WHEELBASE ================")
def d2(a, b):
    return math.hypot(ctrs[a][0] - ctrs[b][0], ctrs[a][1] - ctrs[b][1])
def dx(a, b): return abs(ctrs[a][0] - ctrs[b][0])
def dy(a, b): return abs(ctrs[a][1] - ctrs[b][1])
w(f"track  back  (R-L lateral): {dy('right_back','left_back')*1000:.1f} mm  (centre-to-centre {d2('right_back','left_back')*1000:.1f} mm)")
w(f"track  front (R-L lateral): {dy('right_front','left_front')*1000:.1f} mm")
w(f"wheelbase (back->front)   : {dx('right_back','right_front')*1000:.1f} mm")

# actual PhysX masses/COM from the articulation (auto-computed COM)
try:
    from isaacsim.core.api import SimulationContext
    from isaacsim.core.prims import Articulation
    import numpy as np
    sim = SimulationContext(physics_dt=1/120, rendering_dt=1/60, stage_units_in_meters=1.0)
    sim.play(); app.update()
    art = Articulation(prim_paths_expr=P + "/robot_base", name="r"); art.initialize()
    bm = np.asarray(art.get_body_masses())[0]
    w("\n================ PHYSX (articulation) ================")
    w("body names :", list(art.body_names))
    w("body masses:", [round(float(x), 4) for x in bm], "kg")
    w(f"TOTAL robot mass: {float(bm.sum()):.3f} kg")
    try:
        coms = art.get_body_coms()
        w("body COMs  :", np.round(np.asarray(coms[0] if isinstance(coms, tuple) else coms)[0], 4).tolist())
    except Exception as e:
        w("body COMs  : (auto = collider centroid; per-link API n/a:", repr(e)[:60], ")")
    sim.stop()
except Exception as e:
    w("\n(physics query skipped:", repr(e)[:80], ")")

_OUT.close()
app.close()
