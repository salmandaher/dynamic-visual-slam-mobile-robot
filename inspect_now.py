from pxr import Sdf
import re
L = Sdf.Layer.FindOrOpen("/home/salman/Documents/simsim/new_built_robot.usd")
assert L, "cannot open new_built_robot.usd"
t = L.ExportToString()
open("/tmp/now.usda", "w").write(t)
print("chars:", len(t))
print("=== counts ===")
for kw in ['ArticulationRootAPI', 'def Camera', 'd435i', 'PhysicsScene', 'GroundPlane',
           'color_camera', 'depth_camera', 'PhysicsDriveAPI', 'physics:approximation']:
    print(f"  {len(re.findall(re.escape(kw), t)):2d}x  {kw}")
print("=== revolute joints + their bodies/drives ===")
for m in re.finditer(r'def PhysicsRevoluteJoint "([^"]+)"(.*?)\n    \}', t, re.S):
    name = m.group(1); body = m.group(2)
    dmp = re.search(r'drive:angular:physics:damping = ([-\d.e]+)', body)
    tv = re.search(r'drive:angular:physics:targetVelocity = ([-\d.e]+)', body)
    b1 = re.search(r'physics:body1 = (\S+)', body)
    print(f"   {name:12s} damping={dmp.group(1) if dmp else '-':>8} targetVel={tv.group(1) if tv else '-':>6} body1={b1.group(1) if b1 else '?'}")
print("=== camera prim paths (search) ===")
for m in re.finditer(r'def (Camera|Xform) "(color_camera|depth_camera|d435i_camera|camera_link)"', t):
    print("   ", m.group(0))
print("=== articulation root prim ===")
for m in re.finditer(r'def "?(\w+)"? \([^)]*ArticulationRootAPI', t, re.S):
    print("   root:", m.group(1))
print("=== wheel material friction ===")
for m in re.finditer(r'physics:(static|dynamic)Friction = ([\d.]+)', t):
    print("   ", m.group(0))
