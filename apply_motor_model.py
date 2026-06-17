#!/usr/bin/env python3
"""Make the two driven back-wheel joints behave like real TETRIX MAX TorqueNADO
DC gearmotors instead of ideal velocity servos.

Spec sheet (torquenado_dcmotorspecs.pdf): 12 VDC brushed, 8.7 A stall, output
shaft, per gearbox:
    60:1 (stock):  no-load 100 rpm,  stall 700 oz-in
    40:1        :  no-load 150 rpm,  stall 466 oz-in
    20:1        :  no-load 300 rpm,  stall 233 oz-in

A brushed DC motor at a given voltage follows a LINEAR torque-speed line:
    torque = throttle*stall  -  (stall/free_speed)*omega
A PhysX angular VELOCITY drive computes  torque = damping*(target - omega)
clamped to maxForce. So setting
    target  = throttle * free_speed
    damping = stall_torque / free_speed     (the back-EMF / motor constant)
    maxForce= stall_torque                  (can't exceed stall = stall current)
reproduces the motor's torque-speed curve EXACTLY, and the wheel can never spin
faster than free_speed. This bakes the stock 60:1 motor into the joints; the
drive script re-applies it for the chosen --gear at runtime.

    python3 apply_motor_model.py [--gear 60|40|20] [--scene PATH]
"""
import argparse, math
from pxr import Sdf

OZIN_TO_NM = 0.00706155      # ounce-inch -> newton-metre
RPM_TO_RADS = 2 * math.pi / 60.0
MOTOR = {                    # gearbox : (no-load rpm, stall oz-in, encoder cpr)
    60: (100, 700, 1440),
    40: (150, 466, 960),
    20: (300, 233, 480),
}
BACK_JOINTS = [
    "/World/robot/_9055_txm_4_inch_wheel/node_/mesh_/right_back",
    "/World/robot/_9055_txm_4_inch_wheel_01/node_/mesh_/left_back",
]

ap = argparse.ArgumentParser()
ap.add_argument("--gear", type=int, default=60, choices=[60, 40, 20])
ap.add_argument("--scene", default="/home/salman/Documents/simsim/new_built_robot_gui.usd")
args = ap.parse_args()


def motor_params(gear):
    rpm, ozin, cpr = MOTOR[gear]
    w_free = rpm * RPM_TO_RADS
    t_stall = ozin * OZIN_TO_NM
    return w_free, t_stall, t_stall / w_free, cpr


def set_attr(prim, name, value, vtype=Sdf.ValueTypeNames.Float):
    a = prim.properties.get(name) or Sdf.AttributeSpec(prim, name, vtype, Sdf.VariabilityVarying)
    a.default = value


def main():
    w_free, t_stall, k, cpr = motor_params(args.gear)
    print(f"TorqueNADO {args.gear}:1  ->  free_speed {w_free:.3f} rad/s "
          f"({MOTOR[args.gear][0]} rpm), stall {t_stall:.3f} N*m "
          f"({MOTOR[args.gear][1]} oz-in), damping(k)={k:.4f} N*m/(rad/s), encoder {cpr} cpr")
    L = Sdf.Layer.FindOrOpen(args.scene)
    assert L, args.scene
    for jp in BACK_JOINTS:
        j = L.GetPrimAtPath(jp)
        assert j is not None, f"joint missing: {jp}"
        set_attr(j, "drive:angular:physics:stiffness", 0.0)        # pure velocity (voltage) drive
        set_attr(j, "drive:angular:physics:damping", k)            # back-EMF slope
        set_attr(j, "drive:angular:physics:maxForce", t_stall)     # stall-torque (=stall-current) limit
        set_attr(j, "drive:angular:physics:targetVelocity", 0.0)
        print(f"  {jp}: damping={k:.4f}, maxForce={t_stall:.4f}, stiffness=0")
    L.Save()
    print("SAVED:", args.scene)


if __name__ == "__main__":
    main()
