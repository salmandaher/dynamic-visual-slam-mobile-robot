// Forward kinematics for new_arm, shared by the browser viz and node tests.
// Link origins/axes are taken verbatim from new_arm_description/urdf/new_arm.urdf
// so the schematic matches the real chain. Returns the joint-origin positions in
// the base frame: [base, j1, j2, j3, tip].
//
// Works both in the browser (global ArmFK) and under node (module.exports).
(function (global) {
  'use strict';

  // [translation (m) from previous link, rotation axis] for each joint, in order.
  var CHAIN = [
    { t: [0.00043239, -0.00184063, 0.08396122], axis: 'z' }, // after_base_full (yaw)
    { t: [0.00814224, -0.00043239, 0.00161718], axis: 'y' }, // link1 (pitch)
    { t: [0.001, -0.00856, 0.12839612], axis: 'y' },         // link2 (pitch)
    { t: [0.12531939, 0.01099, -0.05778452], axis: 'y' },    // end_effector (passive)
  ];

  function ident() {
    return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  }

  // row-major 4x4 multiply: returns a*b
  function mul(a, b) {
    var r = new Array(16);
    for (var i = 0; i < 4; i++) {
      for (var j = 0; j < 4; j++) {
        var s = 0;
        for (var k = 0; k < 4; k++) s += a[i * 4 + k] * b[k * 4 + j];
        r[i * 4 + j] = s;
      }
    }
    return r;
  }

  function trans(x, y, z) {
    var m = ident();
    m[3] = x; m[7] = y; m[11] = z;
    return m;
  }

  function rot(axis, th) {
    var c = Math.cos(th), s = Math.sin(th);
    var m = ident();
    if (axis === 'x') {
      m[5] = c; m[6] = -s; m[9] = s; m[10] = c;
    } else if (axis === 'y') {
      m[0] = c; m[2] = s; m[8] = -s; m[10] = c;
    } else { // z
      m[0] = c; m[1] = -s; m[4] = s; m[5] = c;
    }
    return m;
  }

  // thetas: array of joint angles (rad). Length 3 (actuated) or 4 (incl passive).
  // Missing passive angle defaults to 0. Returns [[x,y,z], ...] of length 5.
  function fk(thetas) {
    var T = ident();
    var pts = [[0, 0, 0]];
    for (var i = 0; i < CHAIN.length; i++) {
      var th = (thetas && thetas[i] != null) ? thetas[i] : 0;
      T = mul(T, trans(CHAIN[i].t[0], CHAIN[i].t[1], CHAIN[i].t[2]));
      T = mul(T, rot(CHAIN[i].axis, th));
      pts.push([T[3], T[7], T[11]]);
    }
    return pts;
  }

  var api = { fk: fk, CHAIN: CHAIN };
  global.ArmFK = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof window !== 'undefined' ? window : globalThis);
