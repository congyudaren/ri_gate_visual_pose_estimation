import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def _arg(name: str, default_value: str, description: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default_value, description=description)


def generate_launch_description() -> LaunchDescription:
    roi_lidar_corner_share = get_package_share_directory("roi_lidar_corner")
    default_config_file = os.path.join(roi_lidar_corner_share, "config", "roi_lidar_corner.yaml")
    detector_default_model = os.path.join(roi_lidar_corner_share, "models", "best.pt")
    detector_default_names = os.path.join(roi_lidar_corner_share, "models", "detect.names")

    # 配置文件：YAML 保存 ROI-only 运行时默认值。
    # 下面的 launch 参数用于暴露常用覆盖项，方便命令行调整。
    config_file_arg = _arg("config_file", default_config_file, "[配置] ROI 包 YAML 参数文件。")

    # 检测器设置：roi_generator_node 使用这些参数选择图像目标检测后端、
    # 模型资源和过滤阈值，然后生成 front-face ROI。
    detector_backend_arg = _arg("detector_backend", "pt", "[检测器] 后端类型，例如 pt 或 onnx。")
    detector_model_path_arg = _arg("detector_model_path", detector_default_model, "[检测器] 目标检测模型路径。")
    detector_names_path_arg = _arg("detector_names_path", detector_default_names, "[检测器] 检测类别名称文件路径。")
    detector_conf_threshold_arg = _arg("detector_conf_threshold", "0.25", "[检测器] 最小检测置信度。")
    detector_iou_threshold_arg = _arg("detector_iou_threshold", "0.45", "[检测器] NMS IoU 阈值。")
    detector_input_size_arg = _arg("detector_input_size", "640", "[检测器] 检测器方形输入尺寸，单位像素。")
    detector_use_gpu_arg = _arg("detector_use_gpu", "true", "[检测器] 后端支持时允许使用 GPU。")
    detector_class_filter_arg = _arg("detector_class_filter", "[]", "[检测器] 保留类别 id/name 的 JSON 风格列表。")

    # 上游传感器 topic：由外部相机、LiDAR 和里程计系统发布。
    # 本包只订阅这些 topic，不负责启动对应硬件或上游节点。
    image_topic_arg = _arg("image_topic", "/camera/color/image_raw", "[输入] 相机图像 topic。")
    pointcloud_topic_arg = _arg("pointcloud_topic", "/cloud_registered", "[输入] 外部 LiDAR 流水线发布的 PointCloud2 topic。")
    odom_topic_arg = _arg("odom_topic", "/Odometry", "[输入] 外部定位/里程计系统发布的 Odometry topic。")
    camera_info_topic_arg = _arg("camera_info_topic", "/camera/color/camera_info", "[输入] 与 image_topic 对应的 CameraInfo topic。")

    # ROI 几何与图像细化：目标检测之后用这些参数稳定 front-face ROI 角点。
    corner_radius_arg = _arg("corner_radius", "5", "[ROI] 检测/前脸角点周围使用的像素半径。")
    neo_canny_low_arg = _arg("neo_canny_low", "50", "[ROI] ROI 细化使用的 Canny 低阈值。")
    neo_canny_high_arg = _arg("neo_canny_high", "200", "[ROI] ROI 细化使用的 Canny 高阈值。")
    neo_hough_threshold_arg = _arg("neo_hough_threshold", "50", "[ROI] ROI 细化使用的 Hough 直线投票阈值。")
    neo_min_line_length_arg = _arg("neo_min_line_length", "50", "[ROI] ROI 细化接受的最小直线长度。")
    neo_max_line_gap_arg = _arg("neo_max_line_gap", "50", "[ROI] ROI 细化允许的最大 Hough 直线间隙。")
    neo_blur_kernel_size_arg = _arg("neo_blur_kernel_size", "5", "[ROI] 边缘提取前的模糊核尺寸。")
    neo_border_ratio_arg = _arg("neo_border_ratio", "0.15", "[ROI] ROI 几何细化使用的边界比例。")
    structure_semantics_arg = _arg(
        "structure_semantics",
        "inverted_camera",
        "[ROI] 结构标签语义：normal 使用图像上边作为 TOP_BEAM，inverted_camera 使用图像下边作为物理 TOP_BEAM。",
    )

    # 调试输出开关：启用可选图像叠加、UV 诊断和 RViz marker，
    # 不改变核心 ROI/角点输出。
    publish_debug_image_arg = _arg(
        "publish_debug_image", "true", "[调试] 发布 ROI 叠加图像。"
    )
    publish_debug_uv_arg = _arg("publish_debug_uv", "true", "[调试] 发布 solver UV 诊断信息。")
    debug_overlay_frame_count_arg = _arg("debug_overlay_frame_count", "1", "[调试] 叠加图保留的帧数。")
    subscribe_corner3d_debug_arg = _arg("subscribe_corner3d_debug", "true", "[调试] 为叠加图订阅角点输出。")
    subscribe_solver_debug_uv_arg = _arg("subscribe_solver_debug_uv", "true", "[调试] 订阅 solver UV 调试流。")
    enable_debug_markers_arg = _arg("enable_debug_markers", "true", "[调试] 启动 RViz marker 发布节点。")
    debug_image_topic_arg = _arg(
        "debug_image_topic", "/roi_lidar_corner/roi_debug", "[调试] ROI 叠加图像输出 topic。"
    )
    corner3d_topic_arg = _arg(
        "corner3d_topic",
        "/roi_lidar_corner/corners3d",
        "[调试] legacy Corner3DArray 调试输入 topic，必须与 FrontFaceCorners 输出 topic 分离。",
    )

    # ROI 包输出 topic：这些是本包对外发布的主要结果。
    roi_output_topic_arg = _arg("roi_output_topic", "/roi_lidar_corner/front_face_rois", "[输出] 前脸 ROI 数组 topic。")
    point_output_topic_arg = _arg(
        "point_output_topic", "/roi_lidar_corner/front_face_corners", "[输出] FrontFaceCorners topic。"
    )
    debug_output_topic_arg = _arg("debug_output_topic", "/roi_lidar_corner/front_face_debug", "[输出] solver 调试消息 topic。")
    diag_output_topic_arg = _arg("diag_output_topic", "/roi_lidar_corner/solver_diag", "[输出] solver 诊断字符串 topic。")
    debug_uv_output_topic_arg = _arg(
        "debug_uv_output_topic", "/roi_lidar_corner/solver_debug_uv", "[输出] solver UV 诊断 topic。"
    )

    # 求解器关联过滤：生成 LiDAR 支撑角点前，限制点云/里程计的时间差和距离范围。
    min_points_arg = _arg("min_points", "2", "[求解/过滤] 每个角点候选需要的最少 LiDAR 点数。")
    max_time_diff_cloud_arg = _arg("max_time_diff_cloud", "1.0", "[求解/过滤] 点云与 ROI 的最大时间差，单位秒。")
    max_time_diff_odom_arg = _arg("max_time_diff_odom", "0.5", "[求解/过滤] 里程计与 ROI 的最大时间差，单位秒。")
    min_range_arg = _arg("min_range", "0.2", "[求解/过滤] 接受点的最小距离，单位米。")
    max_range_arg = _arg("max_range", "30.0", "[求解/过滤] 接受点的最大距离，单位米。")

    # 求解器跟踪与聚合：控制回看窗口、体素缓存、ROI 扩展和角点稳定性要求。
    history_window_sec_arg = _arg("history_window_sec", "1.0", "[求解/跟踪] 回看窗口时长，单位秒。")
    max_window_frames_arg = _arg("max_window_frames", "30", "[求解/跟踪] 最多保留的历史帧数。")
    cache_voxel_size_arg = _arg("cache_voxel_size", "0.1", "[求解/跟踪] 缓存点降采样体素尺寸。")
    bbox_expand_ratio_arg = _arg("bbox_expand_ratio", "0.15", "[求解/跟踪] ROI 边界框扩展比例。")
    corner_target_points_arg = _arg("corner_target_points", "6", "[求解/跟踪] 接受角点前的目标点数。")
    corner_target_frames_arg = _arg("corner_target_frames", "2", "[求解/跟踪] 每个角点需要的目标支撑帧数。")
    corner_cap_points_arg = _arg("corner_cap_points", "96", "[求解/跟踪] 每个角点最多保留的点数。")
    post_max_z_jump_m_arg = _arg("post_max_z_jump_m", "0.8", "[求解/跟踪] 后处理允许的最大垂直跳变，单位米。")

    # 输出坐标系变换：将相机坐标系下求解出的角点转换到配置的输出坐标系。
    # 这里使用通用 ROI 参数，不再读取 FAST-LIO 配置。
    output_frame_id_arg = _arg("output_frame_id", "body", "[变换] 发布 3D 角点使用的 frame id。")
    output_extrinsic_t_x_arg = _arg("output_extrinsic_t_x", "0.0", "[变换] 输出变换平移 x，单位米。")
    output_extrinsic_t_y_arg = _arg("output_extrinsic_t_y", "0.0", "[变换] 输出变换平移 y，单位米。")
    output_extrinsic_t_z_arg = _arg("output_extrinsic_t_z", "0.0", "[变换] 输出变换平移 z，单位米。")
    output_extrinsic_r_00_arg = _arg("output_extrinsic_r_00", "0.0", "[变换] 输出旋转矩阵第 0 行第 0 列。")
    output_extrinsic_r_01_arg = _arg("output_extrinsic_r_01", "0.0", "[变换] 输出旋转矩阵第 0 行第 1 列。")
    output_extrinsic_r_02_arg = _arg("output_extrinsic_r_02", "1.0", "[变换] 输出旋转矩阵第 0 行第 2 列。")
    output_extrinsic_r_10_arg = _arg("output_extrinsic_r_10", "1.0", "[变换] 输出旋转矩阵第 1 行第 0 列。")
    output_extrinsic_r_11_arg = _arg("output_extrinsic_r_11", "0.0", "[变换] 输出旋转矩阵第 1 行第 1 列。")
    output_extrinsic_r_12_arg = _arg("output_extrinsic_r_12", "0.0", "[变换] 输出旋转矩阵第 1 行第 2 列。")
    output_extrinsic_r_20_arg = _arg("output_extrinsic_r_20", "0.0", "[变换] 输出旋转矩阵第 2 行第 0 列。")
    output_extrinsic_r_21_arg = _arg("output_extrinsic_r_21", "1.0", "[变换] 输出旋转矩阵第 2 行第 1 列。")
    output_extrinsic_r_22_arg = _arg("output_extrinsic_r_22", "0.0", "[变换] 输出旋转矩阵第 2 行第 2 列。")

    # LaunchConfiguration 句柄与上面的参数分类保持一致。
    # 它们传入节点参数表，使命令行覆盖优先于 YAML 默认值。
    config_file = LaunchConfiguration("config_file")

    # 检测器设置。
    detector_backend = LaunchConfiguration("detector_backend")
    detector_model_path = LaunchConfiguration("detector_model_path")
    detector_names_path = LaunchConfiguration("detector_names_path")
    detector_conf_threshold = LaunchConfiguration("detector_conf_threshold")
    detector_iou_threshold = LaunchConfiguration("detector_iou_threshold")
    detector_input_size = LaunchConfiguration("detector_input_size")
    detector_use_gpu = LaunchConfiguration("detector_use_gpu")
    detector_class_filter = LaunchConfiguration("detector_class_filter")

    # 上游传感器 topic。
    image_topic = LaunchConfiguration("image_topic")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")

    # ROI 几何与图像细化。
    corner_radius = LaunchConfiguration("corner_radius")
    neo_canny_low = LaunchConfiguration("neo_canny_low")
    neo_canny_high = LaunchConfiguration("neo_canny_high")
    neo_hough_threshold = LaunchConfiguration("neo_hough_threshold")
    neo_min_line_length = LaunchConfiguration("neo_min_line_length")
    neo_max_line_gap = LaunchConfiguration("neo_max_line_gap")
    neo_blur_kernel_size = LaunchConfiguration("neo_blur_kernel_size")
    neo_border_ratio = LaunchConfiguration("neo_border_ratio")
    structure_semantics = LaunchConfiguration("structure_semantics")

    # 调试输出开关。
    publish_debug_image = LaunchConfiguration("publish_debug_image")
    publish_debug_uv = LaunchConfiguration("publish_debug_uv")
    debug_overlay_frame_count = LaunchConfiguration("debug_overlay_frame_count")
    subscribe_corner3d_debug = LaunchConfiguration("subscribe_corner3d_debug")
    subscribe_solver_debug_uv = LaunchConfiguration("subscribe_solver_debug_uv")
    enable_debug_markers = LaunchConfiguration("enable_debug_markers")
    debug_image_topic = LaunchConfiguration("debug_image_topic")
    corner3d_topic = LaunchConfiguration("corner3d_topic")

    # ROI 包输出 topic。
    roi_output_topic = LaunchConfiguration("roi_output_topic")
    point_output_topic = LaunchConfiguration("point_output_topic")
    debug_output_topic = LaunchConfiguration("debug_output_topic")
    diag_output_topic = LaunchConfiguration("diag_output_topic")
    debug_uv_output_topic = LaunchConfiguration("debug_uv_output_topic")

    # 求解器关联过滤。
    min_points = LaunchConfiguration("min_points")
    max_time_diff_cloud = LaunchConfiguration("max_time_diff_cloud")
    max_time_diff_odom = LaunchConfiguration("max_time_diff_odom")
    min_range = LaunchConfiguration("min_range")
    max_range = LaunchConfiguration("max_range")

    # 求解器跟踪与聚合。
    history_window_sec = LaunchConfiguration("history_window_sec")
    max_window_frames = LaunchConfiguration("max_window_frames")
    cache_voxel_size = LaunchConfiguration("cache_voxel_size")
    bbox_expand_ratio = LaunchConfiguration("bbox_expand_ratio")
    corner_target_points = LaunchConfiguration("corner_target_points")
    corner_target_frames = LaunchConfiguration("corner_target_frames")
    corner_cap_points = LaunchConfiguration("corner_cap_points")
    post_max_z_jump_m = LaunchConfiguration("post_max_z_jump_m")

    # 输出坐标系变换。
    output_frame_id = LaunchConfiguration("output_frame_id")
    output_extrinsic_t_x = LaunchConfiguration("output_extrinsic_t_x")
    output_extrinsic_t_y = LaunchConfiguration("output_extrinsic_t_y")
    output_extrinsic_t_z = LaunchConfiguration("output_extrinsic_t_z")
    output_extrinsic_r_00 = LaunchConfiguration("output_extrinsic_r_00")
    output_extrinsic_r_01 = LaunchConfiguration("output_extrinsic_r_01")
    output_extrinsic_r_02 = LaunchConfiguration("output_extrinsic_r_02")
    output_extrinsic_r_10 = LaunchConfiguration("output_extrinsic_r_10")
    output_extrinsic_r_11 = LaunchConfiguration("output_extrinsic_r_11")
    output_extrinsic_r_12 = LaunchConfiguration("output_extrinsic_r_12")
    output_extrinsic_r_20 = LaunchConfiguration("output_extrinsic_r_20")
    output_extrinsic_r_21 = LaunchConfiguration("output_extrinsic_r_21")
    output_extrinsic_r_22 = LaunchConfiguration("output_extrinsic_r_22")

    roi_generator = Node(
        package="roi_lidar_corner",
        executable="roi_generator_node.py",
        name="roi_generator_node",
        output="screen",
        parameters=[
            config_file,
            {
                # 上游图像输入。
                "image_topic": image_topic,
                # 检测器与 ROI 细化。
                "corner_radius": corner_radius,
                "detector_backend": detector_backend,
                "detector_model_path": detector_model_path,
                "detector_names_path": detector_names_path,
                "detector_conf_threshold": detector_conf_threshold,
                "detector_iou_threshold": detector_iou_threshold,
                "detector_input_size": detector_input_size,
                "detector_use_gpu": detector_use_gpu,
                "detector_class_filter": ParameterValue(detector_class_filter, value_type=str),
                # 发布的 ROI/调试 topic。
                "roi_output_topic": roi_output_topic,
                "corner3d_topic": corner3d_topic,
                "solver_debug_uv_topic": debug_uv_output_topic,
                "subscribe_corner3d_debug": subscribe_corner3d_debug,
                "subscribe_solver_debug_uv": subscribe_solver_debug_uv,
                "publish_debug_image": publish_debug_image,
                "debug_image_topic": debug_image_topic,
                "neo_canny_low": neo_canny_low,
                "neo_canny_high": neo_canny_high,
                "neo_hough_threshold": neo_hough_threshold,
                "neo_min_line_length": neo_min_line_length,
                "neo_max_line_gap": neo_max_line_gap,
                "neo_blur_kernel_size": neo_blur_kernel_size,
                "neo_border_ratio": neo_border_ratio,
                "structure_semantics": structure_semantics,
            }
        ],
    )

    corner_solver = Node(
        package="roi_lidar_corner",
        executable="corner_lidar_solver_node.py",
        name="corner_lidar_solver_node",
        output="screen",
        parameters=[
            config_file,
            {
                # 来自 roi_generator_node 和外部上游生产者的输入。
                "roi_topic": roi_output_topic,
                "image_topic": image_topic,
                "pointcloud_topic": pointcloud_topic,
                "odom_topic": odom_topic,
                "camera_info_topic": camera_info_topic,
                # 求解器输出。
                "point_output_topic": point_output_topic,
                "debug_output_topic": debug_output_topic,
                "diag_output_topic": diag_output_topic,
                "debug_uv_output_topic": debug_uv_output_topic,
                "publish_debug_uv": publish_debug_uv,
                "debug_overlay_frame_count": debug_overlay_frame_count,
                # 点云/时间关联过滤。
                "min_points": min_points,
                "max_time_diff_cloud": max_time_diff_cloud,
                "max_time_diff_odom": max_time_diff_odom,
                "min_range": min_range,
                "max_range": max_range,
                # 回看、跟踪与角点稳定性控制。
                "history_window_sec": history_window_sec,
                "max_window_frames": max_window_frames,
                "cache_voxel_size": cache_voxel_size,
                "bbox_expand_ratio": bbox_expand_ratio,
                "corner_target_points": corner_target_points,
                "corner_target_frames": corner_target_frames,
                "corner_cap_points": corner_cap_points,
                "post_max_z_jump_m": post_max_z_jump_m,
                "structure_semantics": structure_semantics,
                # 输出坐标系变换。
                "output_frame_id": output_frame_id,
                "output_extrinsic_t_x": output_extrinsic_t_x,
                "output_extrinsic_t_y": output_extrinsic_t_y,
                "output_extrinsic_t_z": output_extrinsic_t_z,
                "output_extrinsic_r_00": output_extrinsic_r_00,
                "output_extrinsic_r_01": output_extrinsic_r_01,
                "output_extrinsic_r_02": output_extrinsic_r_02,
                "output_extrinsic_r_10": output_extrinsic_r_10,
                "output_extrinsic_r_11": output_extrinsic_r_11,
                "output_extrinsic_r_12": output_extrinsic_r_12,
                "output_extrinsic_r_20": output_extrinsic_r_20,
                "output_extrinsic_r_21": output_extrinsic_r_21,
                "output_extrinsic_r_22": output_extrinsic_r_22,
            }
        ],
    )

    debug_markers = Node(
        package="roi_lidar_corner",
        executable="roi_lidar_debug_markers.py",
        name="roi_lidar_debug_markers",
        output="screen",
        condition=IfCondition(enable_debug_markers),
        parameters=[
            {
                # 对已发布 FrontFaceCorners 的可选 RViz 可视化。
                "corner_topic": point_output_topic,
                "marker_topic": "/roi_lidar_corner/front_face_markers",
                "show_invalid": False,
                "point_scale": 0.08,
                "text_scale": 0.08,
                "frame_id_fallback": "map",
                "use_text": True,
            }
        ],
    )

    return LaunchDescription(
        [
            # 配置文件。
            config_file_arg,

            # 检测器设置。
            detector_backend_arg,
            detector_model_path_arg,
            detector_names_path_arg,
            detector_conf_threshold_arg,
            detector_iou_threshold_arg,
            detector_input_size_arg,
            detector_use_gpu_arg,
            detector_class_filter_arg,

            # 上游传感器 topic。
            image_topic_arg,
            pointcloud_topic_arg,
            odom_topic_arg,
            camera_info_topic_arg,

            # ROI 几何与图像细化。
            corner_radius_arg,
            neo_canny_low_arg,
            neo_canny_high_arg,
            neo_hough_threshold_arg,
            neo_min_line_length_arg,
            neo_max_line_gap_arg,
            neo_blur_kernel_size_arg,
            neo_border_ratio_arg,
            structure_semantics_arg,

            # 调试输出开关。
            publish_debug_image_arg,
            publish_debug_uv_arg,
            debug_overlay_frame_count_arg,
            subscribe_corner3d_debug_arg,
            subscribe_solver_debug_uv_arg,
            enable_debug_markers_arg,
            debug_image_topic_arg,
            corner3d_topic_arg,

            # ROI 包输出 topic。
            roi_output_topic_arg,
            point_output_topic_arg,
            debug_output_topic_arg,
            diag_output_topic_arg,
            debug_uv_output_topic_arg,

            # 求解器关联过滤。
            min_points_arg,
            max_time_diff_cloud_arg,
            max_time_diff_odom_arg,
            min_range_arg,
            max_range_arg,

            # 求解器跟踪与聚合。
            history_window_sec_arg,
            max_window_frames_arg,
            cache_voxel_size_arg,
            bbox_expand_ratio_arg,
            corner_target_points_arg,
            corner_target_frames_arg,
            corner_cap_points_arg,
            post_max_z_jump_m_arg,

            # 输出坐标系变换。
            output_frame_id_arg,
            output_extrinsic_t_x_arg,
            output_extrinsic_t_y_arg,
            output_extrinsic_t_z_arg,
            output_extrinsic_r_00_arg,
            output_extrinsic_r_01_arg,
            output_extrinsic_r_02_arg,
            output_extrinsic_r_10_arg,
            output_extrinsic_r_11_arg,
            output_extrinsic_r_12_arg,
            output_extrinsic_r_20_arg,
            output_extrinsic_r_21_arg,
            output_extrinsic_r_22_arg,

            # 运行时节点。
            roi_generator,
            corner_solver,
            debug_markers,
        ]
    )
