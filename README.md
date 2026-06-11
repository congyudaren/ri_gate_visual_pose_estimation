# ROI LiDAR Corner

This repository is now an ROI-only ROS 2 workspace centered on the standalone package:

```text
src/roi_lidar_corner
```

The package consumes upstream image, camera info, point cloud, and odometry topics, then publishes ROI and front-face corner outputs. FAST-LIO and Livox are no longer part of this repository; if needed, run them from external workspaces as upstream data producers.

## Expected Inputs

Default topics:

- point cloud: `/points`
- odometry: `/odom`
- image: `/camera/color/image_raw`
- camera info: `/camera/color/camera_info`

These can be overridden through `roi_lidar_corner.launch.py` arguments or the package config.

## Build And Launch

```bash
colcon list
colcon build --symlink-install --packages-select roi_lidar_corner
source install/setup.bash
ros2 launch roi_lidar_corner roi_lidar_corner.launch.py
```

## Verification

```bash
colcon list --packages-select roi_lidar_corner
python3 -m py_compile src/roi_lidar_corner/launch/*.py src/roi_lidar_corner/roi_lidar_corner/*.py
python3 -m pytest src/roi_lidar_corner/tests
```

## Legacy Integration Archive

Legacy FAST-LIO/Livox integration reference files are retained under:

```text
archive/fastlio_integration
```

Those files are non-runtime reference material only. They are not maintained launch entrypoints for this repository.
