# Socket detection with isaac_ros_centerpose — runbook

Goal: locate the wall **socket** from the **wrist camera** with NVIDIA `isaac_ros_centerpose`,
so the arm can bring the **plug** (on the end effector) to it.

This repo provides everything that does **not** need the Isaac ROS container:
the **scene** (socket + plug), the **synthetic-data generator**, and the **camera→ROS bridge**.
The remaining steps (Docker, Isaac ROS, TensorRT, TAO training) are yours to run — they cannot be
done on this box as-is. Honest status of each step is marked **[done] / [you]**.

> Why CenterPose is the hard path here: it is **category-level** (trained per object class, not per
> instance) and **scale-ambiguous**, there is **no pretrained socket model** (only the 9 Objectron
> classes: bike/book/bottle/camera/cereal_box/chair/cup/laptop/shoe), and it runs **only inside the
> Isaac ROS Docker container** with **TensorRT**. So you must *train a socket model on synthetic data*
> and *stand up the Isaac ROS stack*. If you ever want a faster route: an **AprilTag** on the socket
> gives 6-DoF from the same RGB camera with `apriltag_ros` **natively in your ROS 2 Humble** (no
> Docker, no training); `isaac_ros_foundationpose` is markerless+CAD-exact but needs **RGBD** (our cam
> is RGB-only) and also Docker.

---

## 0. Scene assets — [done]
- Robot USD with the wrist camera **and plug** baked on `end_effector_link`:
  `config.USD_PATH` (`scripts/import_urdf.py`, or `scripts/add_socket_plug.py` to patch in place).
- Standalone socket: `config.SOCKET_USD_PATH` (`/Socket`, faceplate + 2 holes + stand, `class=socket`).
- Visual check / approach demo: `./python.sh scripts/socket_demo.py` (IK-drives the arm to the socket;
  docked **EE Camera** viewport shows the wrist view). All geometry/poses are in `config.py` `SOCKET_*`/`PLUG_*`.

## 1. Synthetic data for training — [done script, you run it]
Generate domain-randomized images + 3D-cuboid/pose ground truth of the socket (no Docker; Replicator
runs in Isaac Sim standalone):
```bash
cd /media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64
./python.sh /home/salman/Documents/our_work/new_arm_isaac/scripts/sdg_centerpose.py \
    --num-frames 3000 --out /media/salman/DataDrive1/new_arm_isaac/sdg/socket
```
- Output (BasicWriter): `rgb`, `bounding_box_3d`, `camera_params`, `semantic_segmentation` per frame, on DataDrive1.
- Scale frames up if eval mAP is low (CenterPose for one object: a few thousand DR frames is a sane start).
- **Known friction:** feeding Replicator output into TAO CenterPose has format gaps (forum reports of
  `KeyError: 'AR_data'`). Expect to write a small converter from the BasicWriter cuboid/camera JSON into
  the CenterPose label schema (keypoints of the 3D cuboid + intrinsics) the TAO notebook ingests.

## 2. Train the CenterPose model with TAO — [you] (needs Docker + NGC)
```bash
# Prereqs: Docker + nvidia-container-toolkit, an NGC API key (ngc.nvidia.com).
# Use the NVIDIA TAO CenterPose notebook from the tao_tutorials repo.
pip install nvidia-tao            # launcher (host)
ngc registry model list nvidia/tao/centerpose*     # browse base weights
# In the TAO CenterPose notebook: point the dataset config at the converted dataset from step 1,
# train/fine-tune, evaluate, then EXPORT to ONNX:
#   tao model centerpose export -m <trained.tlt> -o socket_centerpose.onnx ...
```
Result: `socket_centerpose.onnx`. (TAO runs in its own container; keep images/checkpoints on DataDrive1.)

## 3. Stand up Isaac ROS + build the engine — [you] (Docker + TensorRT)
```bash
# Install Docker + nvidia-container-toolkit, then the Isaac ROS dev container:
#   https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html
# NOTE: current Isaac ROS targets ROS 2 Jazzy; you have Humble. Use an Isaac ROS release that still
#       supports Humble, or run the provided Jazzy container (it ships its own ROS, independent of host).
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_pose_estimation   # contains isaac_ros_centerpose
# Inside the dev container, convert the ONNX to a TensorRT engine:
/usr/src/tensorrt/bin/trtexec --onnx=socket_centerpose.onnx --saveEngine=socket_centerpose.plan \
    --fp16   # (match the input dims your model expects)
```

## 4. Stream the sim wrist camera to ROS 2 — [done script]
This needs **no Isaac ROS** — just Isaac's own bridge + ROS 2 Humble:
```bash
source /opt/ros/humble/setup.bash
cd /media/salman/DataDrive1/isaac-sim-standalone-5.1.0-linux-x86_64
./python.sh /home/salman/Documents/our_work/new_arm_isaac/scripts/ros_camera_bridge.py --headless
# verify:
ros2 topic hz /rgb
ros2 topic echo /camera_info --once
ros2 run rqt_image_view rqt_image_view      # pick /rgb
```
Publishes `/rgb` (sensor_msgs/Image, rgb8) + `/camera_info` (CameraInfo with the wrist-cam K), frame
`wrist_cam`. The intrinsics come from `config.CAMERA_*` (focal 14.846 mm, apertures 11.787×20.955 mm,
1080×1920). Runs alongside the rest of the sim in one process (OmniGraph, C++ side — does not stall the loop).

## 5. Run CenterPose against the live camera — [you] (in the Isaac ROS container)
```bash
# In the Isaac ROS container, with /rgb + /camera_info reachable (same ROS_DOMAIN_ID / network):
ros2 launch isaac_ros_centerpose isaac_ros_centerpose.launch.py \
    model_file_path:=socket_centerpose.onnx engine_file_path:=socket_centerpose.plan
# remap the node's input image/camera_info to /rgb and /camera_info (or relaunch the bridge with
# --rgb-topic/--info-topic matching the node's expected names).
# Outputs:
ros2 topic echo /centerpose/detections        # vision_msgs/Detection3DArray (6-DoF socket pose)
ros2 run rqt_image_view rqt_image_view        # pick /centerpose/image_visualized (annotated)
```
The 6-DoF socket pose in the camera frame + the camera's pose on `end_effector_link` (known from the
URDF/FK) gives the socket pose in the base frame → feed it to `kinematics.inverse_kinematics` to drive
the plug to the socket (close the perception→action loop).

---

### Effort & blockers, plainly
- **[done]** scene, plug, socket, SDG generator, camera→ROS bridge, IK approach demo.
- **[you, hours]** install Docker + nvidia-container-toolkit; run SDG (long render); set up the
  camera→ROS verify.
- **[you, days]** write the Replicator→CenterPose label converter; train+eval in TAO (iterate on mAP);
  trtexec; bring up Isaac ROS; wire and tune CenterPose.
- Disk: every container image + dataset + checkpoint must live on **DataDrive1** (system disk is full).
- Reality check: for a single socket, **AprilTag (native, today)** or **FoundationPose (markerless, but
  needs RGBD + Docker)** are easier than training category-level CenterPose. Say the word and I'll wire
  the AprilTag path instead — it would actually run end-to-end on this machine now.
