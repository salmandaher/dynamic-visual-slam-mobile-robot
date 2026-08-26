import csv, numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

t, x, y, z = [], [], [], []
with open("/home/salman/Documents/simsim/traj.csv") as f:
    for r in csv.DictReader(f):
        t.append(float(r["t"])); x.append(float(r["x"])); y.append(float(r["y"])); z.append(float(r["z"]))
t = np.array(t); x = np.array(x); y = np.array(y); z = np.array(z)
t = t - t[0]
dist = float(np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(z)**2)))

fig, ax = plt.subplots(1, 2, figsize=(13, 5.2))

# (a) top-down XY path
sc = ax[0].scatter(x, y, c=t, cmap="viridis", s=10)
ax[0].plot(x, y, "-", color="0.6", lw=0.6, alpha=0.6, zorder=0)
ax[0].plot(x[0], y[0], "o", color="lime", ms=12, mec="k", label="start", zorder=5)
ax[0].plot(x[-1], y[-1], "s", color="red", ms=12, mec="k", label="end", zorder=5)
ax[0].set_xlabel("x [m]"); ax[0].set_ylabel("y [m]")
ax[0].set_title("cuVSLAM camera trajectory (top-down)")
ax[0].axis("equal"); ax[0].grid(alpha=0.3); ax[0].legend(loc="best")
cb = fig.colorbar(sc, ax=ax[0]); cb.set_label("time [s]")

# (b) x,y,z vs time
ax[1].plot(t, x, label="x", lw=2)
ax[1].plot(t, y, label="y", lw=2)
ax[1].plot(t, z, label="z", lw=2)
ax[1].set_xlabel("time [s]"); ax[1].set_ylabel("position [m]")
ax[1].set_title("Position vs. time")
ax[1].grid(alpha=0.3); ax[1].legend(loc="best")

fig.suptitle(f"Dynamic-VSLAM replay — SLAM path  |  {len(x)} poses, {t[-1]:.1f} s, path length {dist:.2f} m",
             fontsize=13, fontweight="bold")
fig.tight_layout(rect=[0, 0, 1, 0.96])
fig.savefig("/home/salman/Documents/simsim/slam_path.png", dpi=140)
print(f"saved slam_path.png | poses={len(x)} duration={t[-1]:.1f}s path_length={dist:.2f}m "
      f"end=({x[-1]:.2f},{y[-1]:.2f},{z[-1]:.2f})")
