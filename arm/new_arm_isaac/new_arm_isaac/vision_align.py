#!/usr/bin/env python3
"""Vision-as-force-surrogate: estimate plug<->socket alignment + seating from the wrist camera.

For the contact/fine stage we have no force sensor (the Hiwonder bus servos report only
position/voltage/temperature). Instead the wrist camera watches the plug prongs meet the socket
holes: this module turns a single RGB frame into an alignment error (pixels) and a seating signal,
which a controller can use the way it would use force feedback (correct toward zero error; stop
when seated).

The sim scene is high-contrast by construction (orange prongs, blue faceplate, black holes), so
classical colour segmentation is robust and fast — no learned model needed. On the real arm the
identical pipeline applies, with thresholds tuned to the real plug/socket colours (or a small
detector swapped in).

Pure numpy. detect_alignment(rgb) -> dict with the prong-tip + hole pixel positions, the matched
alignment error, and a `seated` flag (holes occluded by the prongs + prongs filling the faceplate).
"""

import numpy as np


def _masks(rgb, t):
    a = rgb.astype(np.int32)
    R, G, B = a[..., 0], a[..., 1], a[..., 2]
    # prongs render yellow (high R AND high G, low B); green holes are high G but LOW R.
    prong = (R > t["prong_r"]) & (G > t["prong_g"]) & (B < t["prong_b"])
    blue = (B > t["blue_b"]) & (B - R > t["blue_br"])
    green = (G > t["green_g"]) & (R < t["green_r"]) & (G - B > t["green_gb"]) & (G - R > t["green_gr"])
    return prong, blue, green


def _two_centroids(ys, xs, tip=False):
    """Split a point set into left/right by x-median; return up to two (x,y) points.

    tip=True returns the topmost (min-y) point per side (prong tip); else the centroid.
    """
    out = []
    if xs.size < 1:
        return out
    xm = np.median(xs)
    for sel in (xs <= xm, xs > xm):
        if sel.sum() < 5:
            continue
        sx, sy = xs[sel], ys[sel]
        if tip:
            i = int(np.argmin(sy))
            out.append((float(sx[i]), float(sy[i])))
        else:
            out.append((float(sx.mean()), float(sy.mean())))
    return sorted(out)  # left-to-right by x


DEFAULT_THRESH = dict(
    prong_r=150, prong_g=120, prong_b=120,      # yellow prongs: high R AND high G, low B
    blue_b=95, blue_br=20,                       # faceplate: high B, B>R
    green_g=120, green_r=120, green_gb=40, green_gr=40,  # green holes: high G, low R, G>>R,B
    lateral_tol_px=22.0,                          # "laterally aligned" |hole_x - prong_x|
    hole_yellow_rad=24,                            # window (px) around a hole to look for the prong
    hole_yellow_min=40,                            # min yellow pixels in that window = prong in hole
)


def detect_alignment(rgb, thresh=None):
    """rgb: (H,W,3) uint8 wrist frame -> alignment/seating estimate (dict).

    holes are tracked by their green colour; prong tips by the topmost yellow pixels. Alignment
    error = mean matched prong-tip<->hole pixel distance. seated = the green holes are occluded by
    the inserted prongs (head-on, an aligned prong covers its hole) while prongs are in view.
    """
    t = dict(DEFAULT_THRESH)
    if thresh:
        t.update(thresh)
    res = dict(socket_seen=False, holes=[], prong_tips=[], lateral_px=None,
               aligned=False, seated=False, green_frac=0.0, prong_frac=0.0, docked_holes=0)

    prong, blue, green = _masks(rgb, t)
    if blue.sum() < 50:               # no socket faceplate in view
        return res
    res["socket_seen"] = True

    gy, gx = np.where(green)
    res["green_frac"] = float(green.sum()) / float(green.size)
    res["holes"] = _two_centroids(gy, gx, tip=False)

    res["prong_frac"] = float(prong.sum()) / float(prong.size)
    py, px = np.where(prong)
    res["prong_tips"] = _two_centroids(py, px, tip=True)  # topmost yellow per side (for annotation only)

    # LOCAL matching: for each green hole, look in a small window for the prong (yellow). This is
    # robust to the prongs diverging toward the gripper (their centroid x != the hole x).
    if len(res["holes"]) >= 1:
        H, W = prong.shape
        rad = int(t["hole_yellow_rad"])
        offs, docked = [], 0
        for (hx, hy) in res["holes"]:
            x0, x1 = max(0, int(hx - rad)), min(W, int(hx + rad))
            y0, y1 = max(0, int(hy - rad)), min(H, int(hy + rad))
            win = prong[y0:y1, x0:x1]
            if int(win.sum()) > t["hole_yellow_min"]:        # a prong is at this hole
                docked += 1
                wy, wx = np.where(win)
                offs.append(abs((x0 + float(wx.mean())) - hx))  # prong's lateral offset within the hole
        res["docked_holes"] = docked
        if offs:
            res["lateral_px"] = float(np.mean(offs))
            res["aligned"] = res["lateral_px"] < t["lateral_tol_px"]
        # seated: a prong present at EVERY detected hole (>=1) and laterally centered in them.
        res["seated"] = (docked == len(res["holes"])) and res["aligned"]
    return res
