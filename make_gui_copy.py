#!/usr/bin/env python3
"""Copy new_built_robot.usd -> new_built_robot_gui.usd and zero the baked
back-wheel targetVelocity (it was 100 rad/s, which makes the robot lurch at
startup before the GUI driver takes over). Sdf-only (miniconda usd-core)."""
import shutil
from pxr import Sdf

SRC = "/home/salman/Documents/simsim/new_built_robot.usd"
DST = "/home/salman/Documents/simsim/new_built_robot_gui.usd"
BACK_JOINTS = [
    "/World/robot/_9055_txm_4_inch_wheel/node_/mesh_/right_back",
    "/World/robot/_9055_txm_4_inch_wheel_01/node_/mesh_/left_back",
]

CASTERS_MAT = "/World/Looks/casters"

shutil.copyfile(SRC, DST)
L = Sdf.Layer.FindOrOpen(DST)
assert L, DST
for jp in BACK_JOINTS:
    j = L.GetPrimAtPath(jp)
    assert j is not None, f"joint missing: {jp}"
    a = j.properties.get("drive:angular:physics:targetVelocity")
    if a is None:
        a = Sdf.AttributeSpec(j, "drive:angular:physics:targetVelocity",
                              Sdf.ValueTypeNames.Float, Sdf.VariabilityVarying)
    old = a.default
    a.default = 0.0
    print(f"{jp}: targetVelocity {old} -> 0.0 (driver is the authoritative writer)")

# Lower the front omni-wheel caster friction: the small TorqueNADO motors have to
# overcome this when skid-turning, so keep it minimal (the grippy drive wheels
# still control motion -> not slippery).
mat = L.GetPrimAtPath(CASTERS_MAT)
if mat is not None:
    for name, val in (("physics:staticFriction", 0.04), ("physics:dynamicFriction", 0.02)):
        a = mat.properties.get(name) or Sdf.AttributeSpec(mat, name, Sdf.ValueTypeNames.Float,
                                                          Sdf.VariabilityVarying)
        a.default = val
    print(f"{CASTERS_MAT}: caster friction -> static 0.04 / dynamic 0.02 (eases motor turning)")
L.Save()
print("SAVED:", DST)
