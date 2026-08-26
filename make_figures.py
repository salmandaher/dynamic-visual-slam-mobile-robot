#!/usr/bin/env python3
"""Publication-quality figures for the dynamic-VSLAM + EV-charging navigation
experiments. Synthetic-but-realistic data (seeded, with noise) per the user's
request. Outputs vector PDFs (+ one PNG) to figures/.

  exp01_slam_map_and_mask.pdf        3-panel: RGB-D | dynamic mask | map+trajectory
  exp01_slam_ate_rpe_tracking.pdf    ATE/RPE box plots + tracking continuity
  exp02_socket_detection_overlay.png annotated eye-in-hand socket detection
  exp02_pose_error_by_condition.pdf  pose error vs range + detection rate
  exp03_navigation_costmap_trajectories.pdf  costmap + starts/goal/paths
  exp03_navigation_success_error.pdf success/time + final pose error
"""
import numpy as np
import matplotlib as mpl
mpl.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrow, Polygon, Circle, Ellipse, FancyArrowPatch, PathPatch
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
import matplotlib.path as mpath
from scipy.ndimage import distance_transform_edt, gaussian_filter

FIG = "/home/salman/Documents/simsim/figures"
rng = np.random.default_rng(20240617)

plt.rcParams.update({
    "font.family": "serif", "font.serif": ["DejaVu Serif"],
    "mathtext.fontset": "dejavuserif",
    "font.size": 10, "axes.titlesize": 11, "axes.labelsize": 10,
    "axes.titleweight": "bold",
    "xtick.labelsize": 9, "ytick.labelsize": 9, "legend.fontsize": 8.5,
    "axes.grid": True, "grid.alpha": 0.30, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.linewidth": 0.9, "lines.linewidth": 1.8,
    "figure.dpi": 120, "savefig.dpi": 320, "savefig.bbox": "tight",
    "pdf.fonttype": 42, "ps.fonttype": 42,
})
C = {"blue": "#0072B2", "orange": "#E69F00", "green": "#009E73",
     "red": "#D55E00", "purple": "#CC79A7", "gray": "#555555",
     "yellow": "#F0E442", "sky": "#56B4E9", "ink": "#222222"}


def save(fig, name):
    p = f"{FIG}/{name}"
    fig.savefig(p)
    plt.close(fig)
    print("wrote", p)


def smooth_path(pts, n=300, noise=0.0):
    """Catmull-Rom-ish smooth curve through pts (k=2 spline), optional jitter."""
    from scipy.interpolate import splprep, splev
    pts = np.asarray(pts, float)
    tck, _ = splprep([pts[:, 0], pts[:, 1]], s=0, k=min(3, len(pts) - 1))
    u = np.linspace(0, 1, n)
    x, y = splev(u, tck)
    if noise:
        x = x + gaussian_filter(rng.normal(0, noise, n), 6)
        y = y + gaussian_filter(rng.normal(0, noise, n), 6)
    return np.column_stack([x, y])


# ======================================================================= FIG 1
def fig_slam_map_and_mask():
    fig, axs = plt.subplots(1, 3, figsize=(12.4, 4.1))

    def draw_room(ax):
        ax.set_xlim(0, 10); ax.set_ylim(0, 7); ax.set_aspect("equal"); ax.axis("off")
        # back wall + floor (simple perspective)
        ax.add_patch(Polygon([(0, 7), (10, 7), (10, 3.1), (0, 3.1)], closed=True,
                             fc="#c9d2dc", ec="none"))                       # wall
        ax.add_patch(Polygon([(0, 3.1), (10, 3.1), (8.2, 0), (1.8, 0)], closed=True,
                             fc="#a7896b", ec="none"))                       # floor
        for t in np.linspace(0.06, 0.94, 9):                                  # floor grid
            ax.plot([1.8 + (10 - 8.2) * 0 + t * 6.4 + 1.8 * 0, 0 + t * 10],
                    [0, 3.1], color="#8c7053", lw=0.6, alpha=0.6)
        for yy in np.linspace(0.3, 2.9, 5):
            f = yy / 3.1
            ax.plot([1.8 * (1 - f), 10 - 1.8 * (1 - f)], [yy, yy],
                    color="#8c7053", lw=0.6, alpha=0.5)
        # static structures (boxes / shelving stubs)
        for (x, y, w, h, c) in [(1.0, 3.1, 1.4, 1.7, "#7f8c99"),
                                (8.1, 3.1, 1.3, 2.2, "#7f8c99"),
                                (4.0, 3.1, 1.0, 0.9, "#9aa6b2")]:
            ax.add_patch(Rectangle((x, y), w, h, fc=c, ec="#445", lw=0.8))
        return ax

    # ---- (A) RGB-D input with a moving object ----
    ax = draw_room(axs[0])
    # depth ramp inset (top-right) to signal RGB-D
    gx = np.linspace(0, 1, 60)[None, :].repeat(8, 0)
    ax.imshow(gx, extent=(6.2, 9.7, 6.35, 6.85), cmap="turbo", aspect="auto", zorder=5)
    ax.text(6.2, 6.95, "depth", fontsize=7.5, color="#222")
    ax.text(6.2, 6.18, "near", fontsize=6.5, color="#222"); ax.text(9.2, 6.18, "far", fontsize=6.5)
    # moving person (dynamic object) + motion blur + arrow
    px, py = 5.6, 1.5
    for k, a in zip(range(3), [0.18, 0.32, 1.0]):
        dx = -0.55 * (2 - k)
        ax.add_patch(Circle((px + dx, py + 1.55), 0.34, fc=C["red"], ec="none", alpha=a))
        ax.add_patch(Ellipse((px + dx, py + 0.7), 0.62, 1.35, fc=C["red"], ec="none", alpha=a))
    ax.add_patch(FancyArrow(px + 0.55, py + 0.8, 0.95, 0.0, width=0.05,
                            head_width=0.28, head_length=0.3, fc=C["ink"], ec="none", zorder=8))
    ax.text(px + 0.7, py + 1.15, "moving", fontsize=8, style="italic")
    ax.set_title("(a)  RGB-D input (dynamic object)", loc="left")

    # ---- (B) dynamic-object mask / rejected features ----
    ax = draw_room(axs[1])
    # static (kept) features — green +
    sx = rng.uniform(0.7, 9.3, 70); sy = rng.uniform(3.2, 6.6, 70)
    keep = ~((sx > 4.5) & (sx < 6.8) & (sy < 4.0))
    ax.scatter(sx[keep], sy[keep], marker="+", s=26, c=C["green"], lw=1.0, zorder=7)
    fx = rng.uniform(2.0, 8.0, 40); fy = rng.uniform(0.2, 3.0, 40)
    ax.scatter(fx, fy, marker="+", s=22, c=C["green"], lw=0.9, zorder=7)
    # dynamic object mask (semi-transparent) + rejected features (red x)
    mask = Ellipse((5.6, 2.2), 1.5, 3.2, fc=C["red"], ec=C["red"], alpha=0.28, lw=1.6, zorder=6)
    ax.add_patch(mask)
    dxr = rng.uniform(5.0, 6.2, 16); dyr = rng.uniform(0.9, 3.5, 16)
    ax.scatter(dxr, dyr, marker="x", s=30, c=C["red"], lw=1.6, zorder=8)
    ax.set_title("(b)  Dynamic mask + rejected features", loc="left")
    ax.legend(handles=[Line2D([], [], marker="+", color=C["green"], lw=0, ms=8, label="static (inliers)"),
                       Line2D([], [], marker="x", color=C["red"], lw=0, ms=7, label="dynamic (rejected)")],
              loc="upper left", framealpha=0.9, handletextpad=0.2)

    # ---- (C) final map + estimated trajectory ----
    ax = axs[2]; ax.set_aspect("equal")
    grid = np.full((140, 200), 0.55)                      # unknown grey
    grid[10:130, 10:190] = 1.0                            # free (white)
    for (a, b, c, d) in [(10, 14, 10, 190), (126, 130, 10, 190),
                         (10, 130, 10, 14), (10, 130, 186, 190),
                         (30, 95, 60, 66), (60, 66, 60, 150), (95, 100, 120, 190)]:
        grid[a:b, c:d] = 0.0                              # walls (black)
    ax.imshow(grid, cmap="gray", origin="lower", extent=(0, 20, 0, 14), vmin=0, vmax=1)
    gt = smooth_path([(2.5, 2), (5, 3), (8, 3.2), (11, 6), (14, 8.5), (17, 11)], noise=0.0)
    est = smooth_path([(2.5, 2.0), (5, 2.7), (8, 3.5), (11, 6.4), (14, 8.1), (17, 10.6)], noise=0.06)
    ax.plot(gt[:, 0], gt[:, 1], "--", color=C["gray"], lw=1.6, label="ground truth")
    ax.plot(est[:, 0], est[:, 1], "-", color=C["blue"], lw=2.2, label="VSLAM estimate")
    ax.scatter([2.5], [2], marker="o", s=55, c=C["green"], zorder=5, ec="k", lw=0.6)
    ax.scatter([17], [10.6], marker="*", s=160, c=C["red"], zorder=5, ec="k", lw=0.6)
    ax.set_title("(c)  Reconstructed map + trajectory", loc="left")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.grid(False)
    ax.legend(loc="lower right", framealpha=0.92)
    fig.tight_layout()
    save(fig, "exp01_slam_map_and_mask.pdf")


# ======================================================================= FIG 2
def fig_slam_ate_rpe():
    conds = ["Static", "One moving\nobject", "Repeated\nmotion"]
    ate_mu, ate_sd = [2.1, 3.6, 5.4], [0.55, 0.95, 1.6]
    rpe_mu, rpe_sd = [1.0, 1.7, 2.6], [0.28, 0.5, 0.85]
    loss_mu, loss_sd = [0.1, 1.3, 3.4], [0.3, 0.7, 1.2]
    N = 32
    ate = [np.clip(rng.normal(m, s, N), 0.4, None) for m, s in zip(ate_mu, ate_sd)]
    rpe = [np.clip(rng.normal(m, s, N), 0.2, None) for m, s in zip(rpe_mu, rpe_sd)]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10.4, 4.1))
    pos = np.arange(3)
    w = 0.34
    for i, (data, off, col, lab) in enumerate([(ate, -w / 2 - 0.02, C["blue"], "ATE"),
                                               (rpe, +w / 2 + 0.02, C["orange"], "RPE")]):
        bp = ax1.boxplot(data, positions=pos + off, widths=w, patch_artist=True,
                         showfliers=True, flierprops=dict(marker="o", ms=2.5, mfc=col, mec="none", alpha=0.5),
                         medianprops=dict(color="k", lw=1.3), whiskerprops=dict(color=col),
                         capprops=dict(color=col), boxprops=dict(facecolor=col, alpha=0.55, ec=col))
    ax1.set_xticks(pos); ax1.set_xticklabels(conds)
    ax1.set_ylabel("error  [cm]")
    ax1.set_title("(a)  Trajectory accuracy (ATE / RPE)", loc="left")
    ax1.legend(handles=[Rectangle((0, 0), 1, 1, fc=C["blue"], alpha=0.55, label="ATE (abs. traj.)"),
                        Rectangle((0, 0), 1, 1, fc=C["orange"], alpha=0.55, label="RPE (rel. pose)")],
               loc="upper left", framealpha=0.92)
    ax1.set_ylim(0, None)

    bars = ax2.bar(pos, loss_mu, yerr=loss_sd, width=0.55, color=C["red"], alpha=0.75,
                   ec="k", lw=0.7, capsize=4, error_kw=dict(lw=1.0))
    for x, m, s in zip(pos, loss_mu, loss_sd):
        ax2.text(x, m + s + 0.20, f"{m:.1f}", ha="center", fontsize=9, color=C["red"])
    ax2.set_xticks(pos); ax2.set_xticklabels(conds)
    ax2.set_ylabel("tracking-loss events / run", color=C["red"])
    ax2.tick_params(axis="y", labelcolor=C["red"])
    ax2.set_title("(b)  Tracking continuity", loc="left")
    ax2.set_ylim(0, max(np.array(loss_mu) + np.array(loss_sd)) + 1.2)
    # secondary: valid-tracking distance %
    ax2b = ax2.twinx(); ax2b.spines["top"].set_visible(False)
    vtd = [99.6, 95.4, 88.1]
    ax2b.plot(pos, vtd, "-D", color=C["green"], ms=6, lw=1.6, label="valid-tracking distance")
    ax2b.set_ylabel("valid-tracking distance [%]", color=C["green"])
    ax2b.tick_params(axis="y", labelcolor=C["green"]); ax2b.set_ylim(80, 102); ax2b.grid(False)
    ax2b.legend(loc="upper right", framealpha=0.92)
    fig.tight_layout()
    save(fig, "exp01_slam_ate_rpe_tracking.pdf")


# ======================================================================= FIG 3
def fig_socket_overlay():
    fig, ax = plt.subplots(figsize=(7.2, 5.4))
    ax.set_xlim(0, 16); ax.set_ylim(0, 12); ax.set_aspect("equal"); ax.axis("off")
    # vignetted wall background (photo-like)
    yy, xx = np.mgrid[0:12:240j, 0:16:320j]
    vig = 1 - 0.35 * (((xx - 8) / 9) ** 2 + ((yy - 6) / 7) ** 2)
    base = np.dstack([0.30 * vig, 0.33 * vig, 0.37 * vig])
    base += rng.normal(0, 0.012, base.shape)               # sensor noise
    ax.imshow(np.clip(base, 0, 1), extent=(0, 16, 0, 12), origin="lower", zorder=0)
    # charging panel
    ax.add_patch(Rectangle((4.2, 2.6), 7.6, 6.9, fc="#3c4654", ec="#222", lw=1.2, zorder=1))
    ax.add_patch(Rectangle((4.3, 2.7), 7.4, 6.7, fc="none", ec="#6a7686", lw=0.8, zorder=1))
    cx, cy = 8.0, 6.1
    # socket recess + contacts (Type-2-like)
    ax.add_patch(Circle((cx, cy), 2.45, fc="#23272e", ec="#11141a", lw=2.0, zorder=2))
    ax.add_patch(Circle((cx, cy), 2.45, fc="none", ec="#7d8794", lw=1.0, zorder=2))
    ax.add_patch(Ellipse((cx, cy + 1.55), 2.9, 1.0, fc="#23272e", ec="#11141a", lw=1.5, zorder=2))  # flat top
    for ang, r, rad in [(90, 1.15, 0.34), (210, 1.5, 0.42), (330, 1.5, 0.42),
                        (150, 1.7, 0.3), (30, 1.7, 0.3), (250, 1.0, 0.28), (290, 1.0, 0.28)]:
        a = np.deg2rad(ang)
        ax.add_patch(Circle((cx + r * np.cos(a), cy + r * np.sin(a)), rad,
                            fc="#c9b067", ec="#6b5a25", lw=0.8, zorder=3))   # brass contacts
    # ---- detection overlays ----
    # detected boundary (keypoint polygon)
    th = np.linspace(0, 2 * np.pi, 9)[:-1]
    bx = cx + 2.62 * np.cos(th) + rng.normal(0, 0.04, 8)
    by = cy + 2.62 * np.sin(th) + 0.18 * np.sin(2 * th) + rng.normal(0, 0.04, 8)
    ax.add_patch(Polygon(np.column_stack([bx, by]), closed=True, fill=False,
                         ec=C["green"], lw=2.0, zorder=6))
    ax.scatter(bx, by, s=34, c=C["green"], ec="white", lw=0.7, zorder=7)
    # estimated 6-DoF pose axes at socket centre
    L = 2.3
    ax.add_patch(FancyArrowPatch((cx, cy), (cx + L, cy - 0.15), color=C["red"], lw=2.6,
                                 arrowstyle="-|>", mutation_scale=16, zorder=8))   # X
    ax.add_patch(FancyArrowPatch((cx, cy), (cx - 0.35, cy + L), color=C["green"], lw=2.6,
                                 arrowstyle="-|>", mutation_scale=16, zorder=8))   # Y
    ax.add_patch(FancyArrowPatch((cx, cy), (cx - 1.25, cy - 1.0), color=C["blue"], lw=2.6,
                                 arrowstyle="-|>", mutation_scale=16, zorder=8))   # Z (out)
    ax.text(cx + L + 0.1, cy - 0.2, "x", color=C["red"], fontsize=11, weight="bold")
    ax.text(cx - 0.55, cy + L + 0.1, "y", color=C["green"], fontsize=11, weight="bold")
    ax.text(cx - 1.7, cy - 1.25, "z", color=C["blue"], fontsize=11, weight="bold")
    # plug-tip frame (small offset target)
    tx, ty = cx + 0.15, cy + 0.1
    ax.add_patch(Circle((tx, ty), 0.16, fc=C["yellow"], ec="k", lw=0.8, zorder=9))
    ax.plot([tx - 0.5, tx + 0.5], [ty, ty], color=C["yellow"], lw=1.2, zorder=9)
    ax.plot([tx, tx], [ty - 0.5, ty + 0.5], color=C["yellow"], lw=1.2, zorder=9)
    ax.text(tx + 0.35, ty - 0.7, "plug-tip frame", color=C["yellow"], fontsize=8.5, zorder=9)
    # measured range + detection score (HUD)
    ax.text(0.4, 11.3, "Logitech C920  ·  eye-in-hand", color="white", fontsize=9)
    ax.add_patch(Rectangle((10.0, 0.4), 5.6, 1.5, fc="black", alpha=0.55, zorder=9))
    ax.text(10.25, 1.32, "range  = 0.42 m", color="white", fontsize=9.5, zorder=10)
    ax.text(10.25, 0.66, "score = 0.94   yaw = 3.1°", color="#9fe", fontsize=9, zorder=10)
    ax.set_title("Eye-in-hand socket detection and pose estimate", loc="left")
    save(fig, "exp02_socket_detection_overlay.png")


# ======================================================================= FIG 4
def fig_pose_error():
    rng_m = np.linspace(0.20, 1.20, 6)
    light = [("bright", C["blue"], 1.0), ("normal", C["orange"], 1.35), ("dim", C["red"], 1.9)]
    fig = plt.figure(figsize=(11.2, 4.2))
    gs = fig.add_gridspec(1, 3, width_ratios=[1, 1, 0.85], wspace=0.34)
    axT = fig.add_subplot(gs[0, 0]); axY = fig.add_subplot(gs[0, 1]); axD = fig.add_subplot(gs[0, 2])
    for name, col, k in light:
        te = (4 + 17 * rng_m) * k + rng.normal(0, 1.0, 6)          # mm
        tsd = (1.2 + 3 * rng_m) * k
        ye = (0.5 + 2.1 * rng_m) * k + rng.normal(0, 0.25, 6)      # deg
        ysd = (0.3 + 0.8 * rng_m) * k
        axT.plot(rng_m, te, "-o", color=col, ms=5, label=name)
        axT.fill_between(rng_m, te - tsd, te + tsd, color=col, alpha=0.15)
        axY.plot(rng_m, ye, "-s", color=col, ms=5, label=name)
        axY.fill_between(rng_m, ye - ysd, ye + ysd, color=col, alpha=0.15)
    axT.set_xlabel("range  [m]"); axT.set_ylabel("translation error  [mm]")
    axT.set_title("(a)  Translation error", loc="left"); axT.legend(title="lighting", framealpha=0.9)
    axY.set_xlabel("range  [m]"); axY.set_ylabel("yaw error  [deg]")
    axY.set_title("(b)  Yaw error", loc="left"); axY.legend(title="lighting", framealpha=0.9)
    # detection rate bars by lighting
    names = [l[0] for l in light]; cols = [l[1] for l in light]
    dr = [98.5, 94.0, 83.5]; drsd = [1.2, 2.4, 4.1]
    axD.bar(names, dr, yerr=drsd, color=cols, alpha=0.8, ec="k", lw=0.7, capsize=4)
    for i, v in enumerate(dr):
        axD.text(i, v + 1.0, f"{v:.0f}%", ha="center", fontsize=9)
    axD.set_ylabel("detection rate  [%]"); axD.set_ylim(70, 104)
    axD.set_title("(c)  Detection rate", loc="left")
    save(fig, "exp02_pose_error_by_condition.pdf")


# ======================================================================= FIG 5
def fig_nav_costmap():
    H, W = 160, 220
    occ = np.zeros((H, W))                       # 0 free
    # boundary walls
    occ[:6, :] = 1; occ[-6:, :] = 1; occ[:, :6] = 1; occ[:, -6:] = 1
    obstacles = [(40, 95, 60, 78), (95, 150, 120, 138), (30, 60, 150, 168),
                 (110, 128, 40, 95)]
    for (a, b, c, d) in obstacles:
        occ[a:b, c:d] = 1
    # charging station (goal wall fixture)
    occ[70:90, 200:214] = 1
    # inflation -> costmap
    dist = distance_transform_edt(1 - occ)
    infl = np.clip(1.0 - dist / 14.0, 0, 1)
    cost = np.maximum(occ.astype(float), infl * 0.85)
    cost = gaussian_filter(cost, 0.6)

    fig, ax = plt.subplots(figsize=(9.6, 7.0))
    from matplotlib.colors import LinearSegmentedColormap
    cm = LinearSegmentedColormap.from_list("nav", ["#f7f7f7", "#cfe8ff", "#5b9bd5", "#1f3b66", "#101010"])
    ext = (0, W * 0.05, 0, H * 0.05)             # 0.05 m / cell -> 11 x 8 m
    ax.imshow(cost, cmap=cm, origin="lower", extent=ext, vmin=0, vmax=1, interpolation="bilinear")
    ax.set_xlim(*ext[:2]); ax.set_ylim(*ext[2:]); ax.set_aspect("equal")
    # goal (charging socket)
    goal = (10.0, 4.0)
    ax.scatter(*goal, marker="*", s=300, c=C["yellow"], ec="k", lw=1.0, zorder=9, label="approach goal")
    ax.annotate("charger", goal, (goal[0] - 1.4, goal[1] + 0.5), fontsize=8.5, color="k")
    starts = [((1.2, 2.0), 20, C["blue"], "S1 front"),
              ((1.5, 6.2), -25, C["green"], "S2 45° offset"),
              ((2.2, 4.3), 5, C["purple"], "S3 cluttered")]
    waypoints = {0: [(1.2, 2.0), (3.5, 2.4), (6.0, 3.0), (8.4, 3.7), goal],
                 1: [(1.5, 6.2), (3.6, 5.4), (6.2, 4.6), (8.5, 4.2), goal],
                 2: [(2.2, 4.3), (4.2, 3.4), (5.6, 4.6), (8.0, 3.9), goal]}
    for i, ((sp, hd, col, lab)) in enumerate(starts):
        a = np.deg2rad(hd)
        ax.add_patch(FancyArrow(sp[0], sp[1], 0.55 * np.cos(a), 0.55 * np.sin(a), width=0.06,
                                head_width=0.26, head_length=0.24, fc=col, ec="k", lw=0.5, zorder=8))
        planned = smooth_path(waypoints[i], noise=0.0)
        executed = smooth_path(waypoints[i], noise=0.10)
        ax.plot(planned[:, 0], planned[:, 1], "--", color=col, lw=1.4, alpha=0.7, zorder=6)
        ax.plot(executed[:, 0], executed[:, 1], "-", color=col, lw=2.4, zorder=7, label=lab)
        ax.scatter(*sp, s=42, c=col, ec="k", lw=0.6, zorder=8)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.grid(False)
    ax.set_title("Navigation costmap with planned (– –) and executed (—) trajectories", loc="left")
    h = [Line2D([], [], color="k", ls="--", label="planned path"),
         Line2D([], [], color="k", ls="-", lw=2.4, label="executed path"),
         Line2D([], [], marker="*", color=C["yellow"], mec="k", lw=0, ms=14, label="approach goal")]
    h += [Line2D([], [], color=s[2], lw=2.4, label=s[3]) for s in starts]
    ax.legend(handles=h, loc="upper left", framealpha=0.94, ncol=2)
    save(fig, "exp03_navigation_costmap_trajectories.pdf")


# ======================================================================= FIG 6
def fig_nav_success():
    scen = ["S1\nfront", "S2\n45° offset", "S3\ncluttered"]
    pos = np.arange(3)
    succ = [100, 93.3, 86.7]; succ_n = [30, 30, 30]
    time_mu, time_sd = [12.4, 16.8, 22.1], [1.1, 1.9, 3.0]
    perr_mu, perr_ci = [3.1, 4.7, 6.2], [0.6, 0.9, 1.3]
    herr_mu, herr_ci = [2.0, 3.3, 4.8], [0.5, 0.8, 1.2]

    fig, (axA, axB) = plt.subplots(1, 2, figsize=(10.6, 4.2))
    b = axA.bar(pos, succ, width=0.55, color=C["green"], alpha=0.78, ec="k", lw=0.7, label="success rate")
    for x, v in zip(pos, succ):
        axA.text(x, v + 1.2, f"{v:.0f}%", ha="center", fontsize=9)
    axA.set_ylabel("success rate  [%]", color=C["green"]); axA.set_ylim(0, 112)
    axA.tick_params(axis="y", labelcolor=C["green"])
    axA.set_xticks(pos); axA.set_xticklabels(scen)
    axA.set_title("(a)  Approach success and travel time", loc="left")
    axT = axA.twinx(); axT.spines["top"].set_visible(False)
    axT.errorbar(pos, time_mu, yerr=time_sd, fmt="-o", color=C["orange"], ms=6, lw=1.8,
                 capsize=4, label="travel time")
    axT.set_ylabel("mean travel time  [s]", color=C["orange"])
    axT.tick_params(axis="y", labelcolor=C["orange"]); axT.set_ylim(0, 30); axT.grid(False)
    axA.legend(loc="lower left", framealpha=0.9); axT.legend(loc="lower right", framealpha=0.9)

    w = 0.36
    axB.bar(pos - w / 2, perr_mu, yerr=perr_ci, width=w, color=C["blue"], alpha=0.8,
            ec="k", lw=0.7, capsize=4, label="position error")
    axB.set_ylabel("final position error  [cm]", color=C["blue"])
    axB.tick_params(axis="y", labelcolor=C["blue"]); axB.set_ylim(0, 9)
    axB.set_xticks(pos); axB.set_xticklabels(scen)
    axB.set_title("(b)  Final pose error (95% CI)", loc="left")
    axH = axB.twinx(); axH.spines["top"].set_visible(False)
    axH.bar(pos + w / 2, herr_mu, yerr=herr_ci, width=w, color=C["purple"], alpha=0.8,
            ec="k", lw=0.7, capsize=4, label="heading error")
    axH.set_ylabel("final heading error  [deg]", color=C["purple"])
    axH.tick_params(axis="y", labelcolor=C["purple"]); axH.set_ylim(0, 7); axH.grid(False)
    axB.legend(loc="upper left", framealpha=0.9); axH.legend(loc="upper right", framealpha=0.9)
    fig.tight_layout()
    save(fig, "exp03_navigation_success_error.pdf")


if __name__ == "__main__":
    fig_slam_map_and_mask()
    fig_slam_ate_rpe()
    fig_socket_overlay()
    fig_pose_error()
    fig_nav_costmap()
    fig_nav_success()
    print("ALL FIGURES DONE")
