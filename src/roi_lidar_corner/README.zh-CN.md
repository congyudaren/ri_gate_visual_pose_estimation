# roi_lidar_corner

`roi_lidar_corner` 是一个独立的 ROS 2 包，用于 ROI 生成和基于点云的前表面角点求解。它从上游图像、相机内参、点云和里程计 topic 读取数据，并发布 ROI、调试信息和 `FrontFaceCorners` 输出。

## 构建

```bash
colcon build --symlink-install --packages-select roi_lidar_corner
source install/setup.bash
```

`FrontFaceCorners` 仍由外部消息包提供。当前依赖包名是 `rotor_swarm_msgs`；如果后续包名变更，需要同时更新 `package.xml`、`CMakeLists.txt` 和 Python import。

## 启动

```bash
ros2 launch roi_lidar_corner roi_lidar_corner.launch.py
```

默认配置文件为：

```text
config/roi_lidar_corner.yaml
```

可通过 launch 参数替换配置：

```bash
ros2 launch roi_lidar_corner roi_lidar_corner.launch.py \
  config_file:=/path/to/roi_lidar_corner.yaml
```

常用 topic 覆盖：

```bash
ros2 launch roi_lidar_corner roi_lidar_corner.launch.py \
  image_topic:=/camera/color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  pointcloud_topic:=/cloud_registered \
  odom_topic:=/Odometry
```

## 配置

相机到输出坐标系的投影外参使用通用 ROI 参数：

```yaml
corner_lidar_solver_node:
  ros__parameters:
    camera_extrinsic_t: [0.049, 0.29671, 0.01812]
    camera_extrinsic_r: [0.0, 0.0, 1.0,
                         1.0, 0.0, 0.0,
                         0.0, 1.0, 0.0]
```

Topic 默认值、检测器选项、求解器阈值和调试设置都放在 `config/roi_lidar_corner.yaml`。

## 验证

```bash
colcon list --packages-select roi_lidar_corner
colcon build --symlink-install --packages-select roi_lidar_corner
ros2 launch roi_lidar_corner roi_lidar_corner.launch.py --show-args
python3 -m py_compile src/roi_lidar_corner/launch/*.py src/roi_lidar_corner/roi_lidar_corner/*.py
python3 -m pytest src/roi_lidar_corner/tests
```
