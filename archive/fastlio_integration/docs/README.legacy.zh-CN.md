# roi_lidar_corner

`roi_lidar_corner` 是当前 FAST-LIO 工作区中的 ROI 与 LiDAR 角点感知层，负责：

- 从图像生成每个目标的前表面结构 ROI
- 维护因果型 LiDAR 回看时间窗
- 将缓存点云投影到图像平面
- 按 ROI mask 过滤投影点
- 跟踪前表面结构并恢复相机坐标系下的四个前表面角点
- 发布调试图像、RViz marker 和 solver 诊断信息

它依赖上游 topic 工作，不直接修改 `fast_lio` 源码。

## 当前工作区现实

在这个工作区里，当前维护中的推荐入口是 `scripts/` 目录下的 wrapper：

```bash
source ./scripts/env_fastlio.sh
./scripts/run_fastlio_with_roi.sh
```

NX 上使用：

```bash
source ./scripts/env_fastlio_nx.sh
./scripts/run_fastlio_with_roi_nx.sh
```

优先使用这两组 wrapper。它们会加载工作区环境、注入当前维护中的默认参数，并在 `roi_lidar_corner` 没有作为已安装 ROS 包被发现时自动回退到源码 launch 方式。

## Detector 后端

当前支持的 detector backend 值：

- `pt`
- `ultralytics`
- `onnx`

当前默认值：

- backend: `pt`
- model: `src/fast_lio_lx/roi_lidar_corner/models/best.pt`
- class names: `src/fast_lio_lx/roi_lidar_corner/models/detect.names`

如果要显式切回 `onnx`：

```bash
./scripts/run_fastlio_with_roi_nx.sh \
  detector_backend:=onnx \
  detector_model_path:=/home/sy/code/ws_fastlio_nx/src/fast_lio_lx/roi_lidar_corner/models/best_detect.onnx
```

`pt` 和 `ultralytics` 在代码里都会走同一套 Ultralytics detector 实现。

## 运行时输入与输出

上游至少要提供：

- 图像
- 相机内参
- 点云
- 里程计

如果这些输入不完整，节点可能能启动，但不应期待得到有效的 `/roi_lidar_corner/front_face_corners` 输出。

主要输出：

- `/roi_lidar_corner/front_face_rois`
- `/roi_lidar_corner/roi_debug`
- `/roi_lidar_corner/front_face_corners`
- `/roi_lidar_corner/front_face_debug`
- `/roi_lidar_corner/front_face_markers`
- `/roi_lidar_corner/solver_diag`
- `/roi_lidar_corner/solver_debug_uv`

离线静态场景验证 helper：

```bash
python3 -m roi_lidar_corner.offline_front_face_validation \
  --image analysis_artifacts/exposure_from_nx/lidar_projection_exposure_20260423_220029_0p5s.png \
  --points analysis_artifacts/exposure_from_nx/lidar_projection_exposure_20260423_221950_20s_points.npz
```

## 长曝光投影采集

可使用下面的 helper 采集一张独立的 LiDAR 点云到相机图像的长曝光投影叠加图：

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

该工具直接订阅源图像、相机内参、点云和里程计 topic，不依赖 `/roi_lidar_corner/solver_debug_uv` 或其他 ROI solver 调试 topic。
默认只投影时间差在 `--max-time-diff-odom 0.5` 秒内的点云和里程计组合，与 solver 默认值一致。

## 启动方式

推荐的集成启动：

```bash
ros2 launch roi_lidar_corner fastlio_with_roi.launch.py
```

当前 `fastlio_with_roi.launch.py` 默认值：

- image: `/camera/color/image_raw`
- camera info: `/camera/color/camera_info`
- point cloud: `/cloud_registered`
- odometry: `/Odometry`
- Livox driver: enabled
- D435i launch: enabled
- detector backend: `pt`

当前维护中的 dev / NX wrapper 共享默认值：

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

独立启动文件 `launch/roi_lidar_corner.launch.py` 仍然存在，但在当前工作区结构下，根目录 `colcon` 扫描不会把 `roi_lidar_corner` 发现成一个独立顶层包。因此除非包发现逻辑后续被修复，否则请把 wrapper 和直接源码 launch 当成当前维护路径。

## Solver 关键默认参数

当前维护中的 wrapper solver 默认值：

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

当前集成路径下，ROI solver 默认会和 `fast_lio` 一样读取 `fast_lio/config/mid360.yaml` 中的相机外参。

## 最新验证结果

本次会话中重新验证过的内容，时间为 `2026-04-22`：

- `bash -n scripts/env_fastlio.sh scripts/env_fastlio_nx.sh scripts/run_fastlio_with_roi.sh scripts/run_fastlio_with_roi_nx.sh` 通过
- `python3 -m py_compile` 通过，检查了：
  - `launch/roi_lidar_corner.launch.py`
  - `launch/fastlio_with_roi.launch.py`
  - `roi_lidar_corner/roi_generator_node.py`
- `source ./scripts/env_fastlio.sh` 和 `source ./scripts/env_fastlio_nx.sh` 后，默认 detector backend 都是 `pt`
- `roi_lidar_corner/models/` 目录下当前存在：
  - `best.pt`
  - `best_detect.onnx`
  - `detect.names`
- 从工作区根目录运行 `colcon list`，当前只发现：
  - `fast_lio`
  - `livox_ros_driver2`
- 从工作区根目录运行 `colcon build --packages-select roi_lidar_corner`，当前返回：
  - `ignoring unknown package 'roi_lidar_corner'`

本次没有重新验证的内容：

- 实时相机输入
- 实时 Livox topic
- NX 上完整集成 smoke
- 真机数据上的 3D 角点输出
