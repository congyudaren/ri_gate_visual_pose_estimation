# roi_lidar_corner

`roi_lidar_corner` is a standalone ROS 2 package for ROI generation and LiDAR-backed front-face corner solving. It consumes standard upstream topics and publishes ROI, debug, and `FrontFaceCorners` outputs.

## Build

```bash
colcon build --symlink-install --packages-select roi_lidar_corner
source install/setup.bash
```

The package keeps `FrontFaceCorners` in an external message package. The current provider is `rotor_swarm_msgs`; if that package name changes, update `package.xml`, `CMakeLists.txt`, and the Python imports together.

## Launch

```bash
ros2 launch roi_lidar_corner roi_lidar_corner.launch.py
```

The default launch file loads:

```text
config/roi_lidar_corner.yaml
```

Override the config file with:

```bash
ros2 launch roi_lidar_corner roi_lidar_corner.launch.py \
  config_file:=/path/to/roi_lidar_corner.yaml
```

Common launch overrides:

```bash
ros2 launch roi_lidar_corner roi_lidar_corner.launch.py \
  image_topic:=/camera/color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  pointcloud_topic:=/cloud_registered \
  odom_topic:=/Odometry
```

## Configuration

Camera-to-output-frame projection uses generic ROI parameters:

```yaml
corner_lidar_solver_node:
  ros__parameters:
    camera_extrinsic_t: [0.049, 0.29671, 0.01812]
    camera_extrinsic_r: [0.0, 0.0, 1.0,
                         1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0]
```

Topic defaults, detector options, solver thresholds, and debug settings live in `config/roi_lidar_corner.yaml`.

## Verification

```bash
colcon list --packages-select roi_lidar_corner
colcon build --symlink-install --packages-select roi_lidar_corner
ros2 launch roi_lidar_corner roi_lidar_corner.launch.py --show-args
python3 -m py_compile src/roi_lidar_corner/launch/*.py src/roi_lidar_corner/roi_lidar_corner/*.py
python3 -m pytest src/roi_lidar_corner/tests
```
