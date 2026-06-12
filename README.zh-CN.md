# ROI LiDAR Corner

这个仓库现在是一个以 `roi_lidar_corner` 为中心的 ROI-only ROS 2 工作空间：

```text
src/roi_lidar_corner
```

该包消费上游的图像、相机内参、点云和里程计 topic，然后发布 ROI 和前表面角点输出。FAST-LIO 和 Livox 已不再作为本仓库的一部分；如有需要，请从外部工作空间启动它们，作为上游数据生产者。

## 期望输入

默认 topic 如下：

- 点云：`/cloud_registered`
- 里程计：`/Odometry`
- 图像：`/camera/color/image_raw`
- 相机内参：`/camera/color/camera_info`

这些都可以通过 `roi_lidar_corner.launch.py` 的参数或包内配置覆盖。

## 构建与启动

```bash
colcon list
colcon build --symlink-install --packages-select roi_lidar_corner
source install/setup.bash
ros2 launch roi_lidar_corner roi_lidar_corner.launch.py
```

## 节点与工作流

这个工作空间里，`roi_lidar_corner` 运行时主要包含三个节点，串联关系由 `src/roi_lidar_corner/launch/roi_lidar_corner.launch.py` 负责。

### 运行时节点

- `roi_generator_node.py`
  - 主文件：`src/roi_lidar_corner/roi_lidar_corner/roi_generator_node.py`
  - 作用：订阅图像流，运行检测器，构建结构体 ROI 和前表面 ROI，并发布 ROI 输出。
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

## 验证

```bash
colcon list --packages-select roi_lidar_corner
colcon build --symlink-install --packages-select roi_lidar_corner
ros2 launch roi_lidar_corner roi_lidar_corner.launch.py --show-args
python3 -m py_compile src/roi_lidar_corner/launch/*.py src/roi_lidar_corner/roi_lidar_corner/*.py
python3 -m pytest src/roi_lidar_corner/tests
```

## 旧版集成归档

历史上的 FAST-LIO/Livox 集成参考文件保留在：

```text
archive/fastlio_integration
```

这些文件仅作为非运行时参考，不再作为本仓库维护的 launch 入口。
