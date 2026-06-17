#!/usr/bin/env python3
"""Subscribe to the robot's camera topics and REPORT what's actually being
published — to find out why depth looks black / the point cloud is missing.

Run with PLAIN ROS 2 Humble (NOT Isaac python), while robot_drive_and_publish.py
--pointcloud is running:
    source /opt/ros/humble/setup.bash
    python3 depth_probe.py [--secs 8]

Saves normalized previews to renders/ and prints stats: depth min/max/finite%,
color mean, and point-cloud point count.
"""
import argparse, os, struct, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo

OUT = "/home/salman/Documents/simsim/renders"
os.makedirs(OUT, exist_ok=True)

ap = argparse.ArgumentParser()
ap.add_argument("--secs", type=float, default=8.0)
ap.add_argument("--depth-topic", default="/camera/depth/image_rect_raw")
ap.add_argument("--color-topic", default="/camera/color/image_raw")
ap.add_argument("--pcl-topic", default="/camera/depth/points")
ap.add_argument("--depth-info", default="/camera/depth/camera_info")
args, _ = ap.parse_known_args()


def save_gray(arr, path):
    a = np.nan_to_num(arr, nan=0.0, posinf=0.0, neginf=0.0)
    finite = a[(a > 0) & np.isfinite(a)]
    if finite.size:
        lo, hi = np.percentile(finite, 2), np.percentile(finite, 98)
        norm = np.clip((a - lo) / max(1e-6, hi - lo), 0, 1)
    else:
        norm = np.zeros_like(a)
    img = (norm * 255).astype(np.uint8)
    try:
        from PIL import Image as PImage
        PImage.fromarray(img).save(path)
    except Exception:
        with open(path.replace(".png", ".pgm"), "wb") as f:
            f.write(b"P5\n%d %d\n255\n" % (img.shape[1], img.shape[0]))
            f.write(img.tobytes())


def save_rgb(arr, path):
    try:
        from PIL import Image as PImage
        PImage.fromarray(arr.astype(np.uint8)).save(path)
    except Exception:
        with open(path.replace(".png", ".ppm"), "wb") as f:
            f.write(b"P6\n%d %d\n255\n" % (arr.shape[1], arr.shape[0]))
            f.write(arr.astype(np.uint8).tobytes())


class Probe(Node):
    def __init__(self):
        super().__init__("depth_probe")
        self.got = {}
        self.create_subscription(Image, args.depth_topic, self.on_depth, 5)
        self.create_subscription(Image, args.color_topic, self.on_color, 5)
        self.create_subscription(PointCloud2, args.pcl_topic, self.on_pcl, 5)
        self.create_subscription(CameraInfo, args.depth_info, self.on_info, 5)

    def on_depth(self, m):
        if "depth" in self.got:
            return
        enc = m.encoding
        buf = bytes(m.data)
        if enc in ("32FC1", "32FC"):
            a = np.frombuffer(buf, np.float32).reshape(m.height, m.width)
        elif enc == "16UC1":
            a = np.frombuffer(buf, np.uint16).reshape(m.height, m.width).astype(np.float32) / 1000.0
        else:
            a = np.frombuffer(buf, np.uint8).reshape(m.height, m.width, -1).astype(np.float32)
        fin = a[np.isfinite(a)]
        pos = a[(a > 0) & np.isfinite(a)]
        self.got["depth"] = dict(
            enc=enc, hw=(m.height, m.width),
            min=float(np.min(fin)) if fin.size else None,
            max=float(np.max(fin)) if fin.size else None,
            mean=float(np.mean(pos)) if pos.size else None,
            finite_pct=100.0 * fin.size / a.size,
            positive_pct=100.0 * pos.size / a.size,
            zero_pct=100.0 * np.sum(a == 0) / a.size,
            inf_pct=100.0 * np.sum(np.isinf(a)) / a.size,
            nan_pct=100.0 * np.sum(np.isnan(a)) / a.size,
        )
        save_gray(a, os.path.join(OUT, "topic_depth.png"))

    def on_color(self, m):
        if "color" in self.got:
            return
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)
        self.got["color"] = dict(enc=m.encoding, hw=(m.height, m.width),
                                 mean=float(a.mean()), shape=a.shape)
        if a.shape[-1] >= 3:
            save_rgb(a[..., :3], os.path.join(OUT, "topic_color.png"))

    def on_pcl(self, m):
        npts = m.width * m.height
        # count finite points by reading x (first float) of each point
        finite = None
        try:
            data = np.frombuffer(bytes(m.data), np.uint8)
            if m.point_step and npts:
                xs = np.frombuffer(data[:npts * m.point_step].tobytes(), np.float32)
                # x is at offset 0 of each point: stride = point_step/4 floats
                stride = m.point_step // 4
                xs = xs[0::stride][:npts]
                finite = int(np.sum(np.isfinite(xs)))
        except Exception:
            pass
        self.got["pcl"] = dict(npts=npts, width=m.width, height=m.height,
                               point_step=m.point_step, row_step=m.row_step,
                               data_len=len(m.data), fields=[f.name for f in m.fields],
                               finite_pts=finite, frame=m.header.frame_id)

    def on_info(self, m):
        self.got["info"] = dict(hw=(m.height, m.width), K=list(m.k),
                                distortion=m.distortion_model, frame=m.header.frame_id)


def main():
    rclpy.init()
    node = Probe()
    t0 = time.time()
    while time.time() - t0 < args.secs and len(node.got) < 4:
        rclpy.spin_once(node, timeout_sec=0.2)
    print("\n================= DEPTH/CAMERA TOPIC PROBE =================")
    for key in ("color", "depth", "info", "pcl"):
        v = node.got.get(key)
        if v is None:
            print(f"  {key.upper():6}: *** NO MESSAGE RECEIVED ***")
        else:
            print(f"  {key.upper():6}: {v}")
    print("===========================================================")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
