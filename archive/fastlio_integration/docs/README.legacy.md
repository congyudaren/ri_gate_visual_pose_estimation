# roi_lidar_corner

`roi_lidar_corner` is the ROI and LiDAR-corner perception layer used with the current FAST-LIO workspace. It:

- generates per-object front-face structure ROI masks from image frames
- keeps a causal lookback window of recent LiDAR clouds
- projects cached LiDAR points into the image plane
- filters projected points inside ROI masks
- tracks front-face structures and restores four camera-frame front-face corners
- publishes debug images, RViz markers, and solver diagnostics

It works from upstream topics and does not patch `fast_lio` source files.

## Current Workspace Reality

In this workspace, the maintained entry path is the `scripts/` wrapper pair:

```bash
source ./scripts/env_fastlio.sh
./scripts/run_fastlio_with_roi.sh
```

For NX:

```bash
source ./scripts/env_fastlio_nx.sh
./scripts/run_fastlio_with_roi_nx.sh
```

Use the wrappers first. They load the workspace environment, apply the maintained defaults, and fall back to source-launch mode automatically if `roi_lidar_corner` is not available as an installed ROS package.

## Detector Backends

Supported detector backend values are:

- `pt`
- `ultralytics`
- `onnx`

Current default:

- backend: `pt`
- model: `src/fast_lio_lx/roi_lidar_corner/models/best.pt`
- class names: `src/fast_lio_lx/roi_lidar_corner/models/detect.names`

Explicit ONNX override is still supported:

```bash
./scripts/run_fastlio_with_roi_nx.sh \
  detector_backend:=onnx \
  detector_model_path:=/home/sy/code/ws_fastlio_nx/src/fast_lio_lx/roi_lidar_corner/models/best_detect.onnx
```

`pt` and `ultralytics` both route to the same Ultralytics-based detector implementation.

## Runtime Inputs And Outputs

Required upstream inputs:

- image
- camera info
- point cloud
- odometry

If any of them are missing, nodes may start but valid `/roi_lidar_corner/front_face_corners` output should not be expected.

Main outputs:

- `/roi_lidar_corner/front_face_rois`
- `/roi_lidar_corner/roi_debug`
- `/roi_lidar_corner/front_face_corners`
- `/roi_lidar_corner/front_face_debug`
- `/roi_lidar_corner/front_face_markers`
- `/roi_lidar_corner/solver_diag`
- `/roi_lidar_corner/solver_debug_uv`

Offline static-scene validation helper:

```bash
python3 -m roi_lidar_corner.offline_front_face_validation \
  --image analysis_artifacts/exposure_from_nx/lidar_projection_exposure_20260423_220029_0p5s.png \
  --points analysis_artifacts/exposure_from_nx/lidar_projection_exposure_20260423_221950_20s_points.npz
```

## Long-Exposure Projection Capture

Use this helper to capture an independent long-exposure overlay of LiDAR points projected into the camera image:

```bash
./scripts/capture_lidar_projection_exposure.sh \
  --duration-sec 20 \
  --window-sec 20 \
  --output-image /tmp/lidar_projection_exposure.png \
  --output-json /tmp/lidar_projection_exposure.json \
  --image-topic /camera/color/image_raw \
  --pointcloud-topic /cloud_registered \
  --odom-topic /Odometry \
  --camera-info-topic /camera/color/camera_info
```

The tool subscribes to the source image, camera info, point cloud, and odometry topics directly. It does not depend on `/roi_lidar_corner/solver_debug_uv` or other ROI solver debug topics.
By default it only projects clouds with odometry within `--max-time-diff-odom 0.5` seconds, matching the solver default.

## Launch Modes

Recommended integrated launch:

```bash
ros2 launch roi_lidar_corner fastlio_with_roi.launch.py
```

Current integrated defaults in the launch file:

- image: `/camera/color/image_raw`
- camera info: `/camera/color/camera_info`
- point cloud: `/cloud_registered`
- odometry: `/Odometry`
- Livox driver: enabled
- D435i launch: enabled
- detector backend: `pt`

Current maintained wrapper defaults shared by dev and NX:

- `enable_d435i=true`
- `image_topic=/camera/color/image_raw`
- `camera_info_topic=/camera/color/camera_info`
- `detector_backend=pt`
- `detector_use_gpu=true`
- `min_points=2`
- `max_time_diff_cloud=1.0`
- `max_time_diff_odom=0.5`
- `history_window_sec=1.0`
- `max_window_frames=30`
- `corner_target_points=6`
- `publish_debug_image=true`
- `open_debug_window=false`
- `rviz=false`

Stand-alone launch is still defined in `launch/roi_lidar_corner.launch.py`, but in the current workspace layout the root `colcon` scan does not discover `roi_lidar_corner` as an independent top-level package. Treat the wrappers and direct source launch files as the maintained path unless package discovery is fixed.

## Key Solver Defaults

Current maintained wrapper solver defaults:

- `history_window_sec=1.0`
- `max_window_frames=30`
- `cache_voxel_size=0.1`
- `bbox_expand_ratio=0.15`
- `corner_target_points=6`
- `corner_target_frames=2`
- `corner_cap_points=96`
- `post_max_z_jump_m=0.8`
- `min_range=0.2`
- `max_range=30.0`

The integrated path and ROI solver both read Fast-LIO camera extrinsics from `fast_lio/config/mid360.yaml` by default.

## Latest Verification

Latest checked on `2026-04-22` in this workspace:

- `bash -n scripts/env_fastlio.sh scripts/env_fastlio_nx.sh scripts/run_fastlio_with_roi.sh scripts/run_fastlio_with_roi_nx.sh` passed
- `python3 -m py_compile` passed for:
  - `launch/roi_lidar_corner.launch.py`
  - `launch/fastlio_with_roi.launch.py`
  - `roi_lidar_corner/roi_generator_node.py`
- sourcing `scripts/env_fastlio.sh` and `scripts/env_fastlio_nx.sh` reports default detector backend `pt`
- model files currently present under `roi_lidar_corner/models/`:
  - `best.pt`
  - `best_detect.onnx`
  - `detect.names`
- `colcon list` from the workspace root currently reports only:
  - `fast_lio`
  - `livox_ros_driver2`
- `colcon build --packages-select roi_lidar_corner` from the workspace root currently returns:
  - `ignoring unknown package 'roi_lidar_corner'`

Not re-verified in this session:

- live camera input
- live Livox topics
- full integrated NX smoke run
- end-to-end 3D corner output on real data
