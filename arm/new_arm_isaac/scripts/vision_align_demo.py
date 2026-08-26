#!/usr/bin/env python3
"""Run the wrist-camera alignment detector over a folder of captured frames and annotate them.

Demonstrates 'vision as the force surrogate': for each frame it prints the plug<->socket alignment
error (pixels), the dark-hole / orange-prong fractions, and the `aligned` / `seated` flags, and
saves an annotated copy (cyan circles = detected holes, magenta crosses = prong tips). As the plug
approaches and seats, the alignment error -> 0 and seated flips True -> exactly the signal a
controller would use instead of force feedback.

    python scripts/vision_align_demo.py --frames /media/.../insert_cam_seq
"""

import argparse
import glob
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from new_arm_isaac.vision_align import detect_alignment  # noqa: E402


def main():
    p = argparse.ArgumentParser(description="Annotate frames with plug/socket alignment")
    p.add_argument("--frames", required=True, help="directory of PNG frames (or a glob)")
    p.add_argument("--out", default=None, help="output dir (default: <frames>/annotated)")
    args = p.parse_args()

    files = sorted(glob.glob(os.path.join(args.frames, "*.png"))
                   if os.path.isdir(args.frames) else glob.glob(args.frames))
    if not files:
        print(f"no PNGs found at {args.frames}")
        return 1
    outdir = args.out or os.path.join(args.frames if os.path.isdir(args.frames)
                                      else os.path.dirname(files[0]), "annotated")
    os.makedirs(outdir, exist_ok=True)

    print(f"{'frame':24s} holes prong lateral_px aligned seated")
    for f in files:
        rgb = np.asarray(Image.open(f).convert("RGB"))
        r = detect_alignment(rgb)
        im = Image.fromarray(rgb.copy())
        d = ImageDraw.Draw(im)
        for (x, y) in r["holes"]:
            d.ellipse([x - 8, y - 8, x + 8, y + 8], outline=(0, 255, 255), width=3)
        for (x, y) in r["prong_tips"]:
            d.line([x - 9, y, x + 9, y], fill=(255, 0, 255), width=3)
            d.line([x, y - 9, x, y + 9], fill=(255, 0, 255), width=3)
        lp = f"{r['lateral_px']:.0f}px" if r["lateral_px"] is not None else "--"
        d.text((6, 6), f"lateral={lp} aligned={r['aligned']} seated={r['seated']}", fill=(255, 255, 0))
        Image.fromarray(np.asarray(im)).save(os.path.join(outdir, "ann_" + os.path.basename(f)))
        lpc = f"{r['lateral_px']:8.1f}" if r["lateral_px"] is not None else "    --  "
        print(f"{os.path.basename(f):24s}  {len(r['holes'])}    {len(r['prong_tips'])}  "
              f"{lpc}     {int(r['aligned'])}      {int(r['seated'])}")
    print(f"annotated -> {outdir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
