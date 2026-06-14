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

## 节点与工作流

`roi_lidar_corner` 运行时主要由 `src/roi_lidar_corner/roi_lidar_corner/` 下的三个节点组成，并通过 `src/roi_lidar_corner/launch/roi_lidar_corner.launch.py` 串联起来。

### 运行时节点

- `roi_generator_node.py`
  - 主文件：`src/roi_lidar_corner/roi_lidar_corner/roi_generator_node.py`
  - 作用：订阅相机图像，运行检测器，构建结构体 ROI 和前表面 ROI，并发布 ROI 相关输出。
  - 关键输入：`image_topic`、检测器参数、ROI 细化参数。
  - 关键输出：`roi_output_topic`、`point_output_topic`、`debug_image_topic`、`debug_uv_output_topic`。

- `corner_lidar_solver_node.py`
  - 主文件：`src/roi_lidar_corner/roi_lidar_corner/corner_lidar_solver_node.py`
  - 作用：订阅 ROI、点云、里程计和相机信息，筛选 LiDAR 支撑点，跟踪前表面，并发布最终的 3D 角点结果和调试信息。
  - 关键输入：`roi_topic`、`pointcloud_topic`、`odom_topic`、`camera_info_topic`。
  - 关键输出：`point_output_topic`、`debug_output_topic`、`diag_output_topic`、`debug_uv_output_topic`。

- `roi_lidar_debug_markers.py`
  - 主文件：`src/roi_lidar_corner/roi_lidar_corner/roi_lidar_debug_markers.py`
  - 作用：可选可视化节点，把 `FrontFaceCorners` 转成 RViz 的 `MarkerArray`。
  - 关键输入：`corner_topic`。
  - 关键输出：`marker_topic`。

### 支撑文件

- 启动串联：`src/roi_lidar_corner/launch/roi_lidar_corner.launch.py`
  - 声明 launch 参数。
  - 加载 `config/roi_lidar_corner.yaml`。
  - 启动 `roi_generator_node`、`corner_lidar_solver_node`，并可选启动 `roi_lidar_debug_markers`。

- 运行时默认值：`src/roi_lidar_corner/config/roi_lidar_corner.yaml`
  - 保存 topic 默认值、检测器参数、求解器阈值、跟踪限制和调试开关。

- 消息定义：`src/roi_lidar_corner/msg/*.msg`
  - 定义节点之间使用的包内 ROI 和调试消息。

- 外部角点消息依赖：`rotor_swarm_msgs/FrontFaceCorners`
  - 最终角点输出消息来自 `package.xml` 中引用的外部消息包。

### 基本工作流

1. `roi_generator_node` 订阅相机图像，生成前表面 ROI 候选和可选的调试叠加图。
2. `corner_lidar_solver_node` 订阅 ROI、上游点云、里程计和相机信息。
3. 求解器把 LiDAR 点和 ROI 对齐，持续跟踪前表面，并发布最终的 `FrontFaceCorners` 结果和诊断信息。
4. `roi_lidar_debug_markers` 可以订阅 `FrontFaceCorners` 并发布 RViz marker，用于可视化。
5. `roi_lidar_debug_view.py` 等可选调试工具可以订阅调试图像 topic 做离线检查，但不属于默认启动链路。

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

`structure_semantics` 是同时传给 `roi_generator_node` 和 `corner_lidar_solver_node`
的共享配置来源，用于定义 2D 结构标签到物理语义的映射：

- `normal`：结构标签沿用图像空间顺序。
- `inverted_camera`：结构标签使用当前倒装相机和默认相机/body 外参下的物理/body 语义。
  `TOP_BEAM` 使用图像下方水平边，`LEFT_POST` 使用图像右侧立柱，
  `RIGHT_POST` 使用图像左侧立柱。

默认值是 `inverted_camera`。在 `/roi_lidar_corner/front_face_corners` 中，
`top_left` 和 `top_right` 表示经过 `output_frame_id` 与 `output_extrinsic_*`
变换后的物理上边缘点。`left` 和 `right` 名称按配置的 `structure_semantics`
使用对应的物理/body 约定。

Topic 默认值、检测器选项、求解器阈值和调试设置都放在 `config/roi_lidar_corner.yaml`。

## 验证

```bash
colcon list --packages-select roi_lidar_corner
colcon build --symlink-install --packages-select roi_lidar_corner
ros2 launch roi_lidar_corner roi_lidar_corner.launch.py --show-args
python3 -m py_compile src/roi_lidar_corner/launch/*.py src/roi_lidar_corner/roi_lidar_corner/*.py
python3 -m pytest src/roi_lidar_corner/tests
```
