# =============================================================================
#  D435i camera preview  —  Isaac Sim 5.1.0
#  HOW TO USE:  open new_built_robot.usd, then  Window > Script Editor,
#               paste this whole file, press the ▶ Run button.
#  A docked "D435i Preview" viewport appears showing the camera's live view.
#  Re-running is safe (it reuses the same window instead of stacking new ones).
#  API verified against the on-disk 5.1.0 source (omni.kit.viewport.utility 1.1.2,
#  omni.kit.viewport.window 107.2.0, omni.ui 2.27.1).
# =============================================================================
from pxr import Sdf
import omni.ui as ui
from omni.kit.viewport.utility import create_viewport_window

# --- pick which D435i stream to preview ------------------------------------
CAM = "/World/robot/d435i_camera/camera_link/color_camera"     # RGB  (69.4 x 42.5 deg)
# CAM = "/World/robot/d435i_camera/camera_link/depth_camera"   # <- swap for depth/IR (87 x 58 deg)
TITLE = "D435i Preview"
RES   = (1280, 720)          # D435 stream resolution
# ---------------------------------------------------------------------------


def _find_existing(title):
    """Return our preview ViewportWindow if it already exists (idempotency)."""
    try:
        from omni.kit.viewport.window import get_viewport_window_instances
        for w in get_viewport_window_instances(""):
            if getattr(w, "title", None) == title:
                return w
    except Exception:
        pass
    return None


win = _find_existing(TITLE)
if win is None:
    # camera_path must be an Sdf.Path (per create_viewport_window signature)
    win = create_viewport_window(name=TITLE, width=RES[0], height=RES[1],
                                 camera_path=Sdf.Path(CAM))

if win is None:
    print("[D435i] ERROR: could not create viewport window "
          "(is omni.kit.viewport.window enabled?)")
else:
    vp = win.viewport_api
    vp.camera_path = Sdf.Path(CAM)          # (re)bind the camera — safe on every re-run
    try:
        vp.resolution = RES                 # render at the D435 stream resolution
    except Exception as e:
        print("[D435i] resolution set skipped:", e)
    try:
        # dock as a tab alongside the main "Viewport" (deferred = waits for layout)
        win.deferred_dock_in("Viewport", ui.DockPolicy.TARGET_WINDOW_IS_ACTIVE)
    except Exception as e:
        print("[D435i] dock skipped:", e)
    print(f"[D435i] previewing {CAM}  in '{TITLE}'  @ {RES[0]}x{RES[1]}")
