import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    fast_lio_share = get_package_share_directory('fast_lio')
    roi_lidar_corner_share = get_package_share_directory('roi_lidar_corner')
    livox_driver_share = get_package_share_directory('livox_ros_driver2')
    try:
        realsense_share = get_package_share_directory('realsense2_camera')
    except Exception:
        realsense_share = None
    fastlio_default_cfg = os.path.join(fast_lio_share, 'config')
    fastlio_default_rviz = os.path.join(fast_lio_share, 'rviz', 'fastlio.rviz')
    detector_default_model = os.path.join(roi_lidar_corner_share, 'models', 'best.pt')
    detector_default_names = os.path.join(roi_lidar_corner_share, 'models', 'detect.names')
    use_sim_time = LaunchConfiguration("use_sim_time")
    fastlio_config_path = LaunchConfiguration("fastlio_config_path")
    fastlio_config_file = LaunchConfiguration("fastlio_config_file")
    semantic_mapping_en = LaunchConfiguration("semantic_mapping_en")
    rviz = LaunchConfiguration("rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    enable_livox_driver = LaunchConfiguration("enable_livox_driver")
    livox_driver_launch = LaunchConfiguration("livox_driver_launch")
    enable_d435i = LaunchConfiguration("enable_d435i")
    d435i_camera_name = LaunchConfiguration("d435i_camera_name")

    image_topic = LaunchConfiguration("image_topic")
    corner_radius = LaunchConfiguration("corner_radius")
    detector_backend = LaunchConfiguration("detector_backend")
    detector_model_path = LaunchConfiguration("detector_model_path")
    detector_names_path = LaunchConfiguration("detector_names_path")
    detector_conf_threshold = LaunchConfiguration("detector_conf_threshold")
    detector_iou_threshold = LaunchConfiguration("detector_iou_threshold")
    detector_input_size = LaunchConfiguration("detector_input_size")
    detector_use_gpu = LaunchConfiguration("detector_use_gpu")
    detector_class_filter = LaunchConfiguration("detector_class_filter")
    roi_output_topic = LaunchConfiguration("roi_output_topic")
    point_output_topic = LaunchConfiguration("point_output_topic")
    debug_output_topic = LaunchConfiguration("debug_output_topic")
    diag_output_topic = LaunchConfiguration("diag_output_topic")
    debug_uv_output_topic = LaunchConfiguration("debug_uv_output_topic")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    min_points = LaunchConfiguration("min_points")
    max_time_diff_cloud = LaunchConfiguration("max_time_diff_cloud")
    max_time_diff_odom = LaunchConfiguration("max_time_diff_odom")
    min_range = LaunchConfiguration("min_range")
    max_range = LaunchConfiguration("max_range")
    history_window_sec = LaunchConfiguration("history_window_sec")
    max_window_frames = LaunchConfiguration("max_window_frames")
    cache_voxel_size = LaunchConfiguration("cache_voxel_size")
    bbox_expand_ratio = LaunchConfiguration("bbox_expand_ratio")
    corner_target_points = LaunchConfiguration("corner_target_points")
    corner_target_frames = LaunchConfiguration("corner_target_frames")
    corner_cap_points = LaunchConfiguration("corner_cap_points")
    post_max_z_jump_m = LaunchConfiguration("post_max_z_jump_m")
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
    publish_debug_image = LaunchConfiguration("publish_debug_image")
    publish_debug_uv = LaunchConfiguration("publish_debug_uv")
    debug_overlay_frame_count = LaunchConfiguration("debug_overlay_frame_count")
    subscribe_corner3d_debug = LaunchConfiguration("subscribe_corner3d_debug")
    subscribe_solver_debug_uv = LaunchConfiguration("subscribe_solver_debug_uv")
    enable_debug_markers = LaunchConfiguration("enable_debug_markers")
    debug_image_topic = LaunchConfiguration("debug_image_topic")
    open_debug_window = LaunchConfiguration("open_debug_window")
    neo_canny_low = LaunchConfiguration("neo_canny_low")
    neo_canny_high = LaunchConfiguration("neo_canny_high")
    neo_hough_threshold = LaunchConfiguration("neo_hough_threshold")
    neo_min_line_length = LaunchConfiguration("neo_min_line_length")
    neo_max_line_gap = LaunchConfiguration("neo_max_line_gap")
    neo_blur_kernel_size = LaunchConfiguration("neo_blur_kernel_size")
    neo_border_ratio = LaunchConfiguration("neo_border_ratio")

    include_fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_share, "launch", "mapping.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "config_path": fastlio_config_path,
            "config_file": fastlio_config_file,
            "semantic_mapping_en": semantic_mapping_en,
            "rviz": rviz,
            "rviz_cfg": rviz_cfg,
        }.items(),
    )

    include_livox_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([livox_driver_share, "launch_ROS2", livox_driver_launch])
        ),
        condition=IfCondition(enable_livox_driver),
    )

    include_d435i_launch = None
    if realsense_share is not None:
        include_d435i_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_share, "launch", "rs_launch.py")
            ),
            launch_arguments={
                "camera_name": d435i_camera_name,
                "config_file": "''",
            }.items(),
            condition=IfCondition(enable_d435i),
        )

    roi_generator = Node(
        package="roi_lidar_corner",
        executable="roi_generator_node.py",
        name="roi_generator_node",
        output="screen",
        parameters=[
            {
                "image_topic": image_topic,
                "corner_radius": corner_radius,
                "detector_backend": detector_backend,
                "detector_model_path": detector_model_path,
                "detector_names_path": detector_names_path,
                "detector_conf_threshold": detector_conf_threshold,
                "detector_iou_threshold": detector_iou_threshold,
                "detector_input_size": detector_input_size,
                "detector_use_gpu": detector_use_gpu,
                "detector_class_filter": ParameterValue(detector_class_filter, value_type=str),
                "roi_output_topic": roi_output_topic,
                "corner3d_topic": "/roi_lidar_corner/corners3d",
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
            }
        ],
    )

    corner_solver = Node(
        package="roi_lidar_corner",
        executable="corner_lidar_solver_node.py",
        name="corner_lidar_solver_node",
        output="screen",
        parameters=[
            {
                "roi_topic": roi_output_topic,
                "image_topic": image_topic,
                "pointcloud_topic": pointcloud_topic,
                "odom_topic": odom_topic,
                "camera_info_topic": camera_info_topic,
                "min_points": min_points,
                "max_time_diff_cloud": max_time_diff_cloud,
                "max_time_diff_odom": max_time_diff_odom,
                "min_range": min_range,
                "max_range": max_range,
                "history_window_sec": history_window_sec,
                "max_window_frames": max_window_frames,
                "cache_voxel_size": cache_voxel_size,
                "bbox_expand_ratio": bbox_expand_ratio,
                "corner_target_points": corner_target_points,
                "corner_target_frames": corner_target_frames,
                "corner_cap_points": corner_cap_points,
                "post_max_z_jump_m": post_max_z_jump_m,
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
                "fastlio_config_path": fastlio_config_path,
                "fastlio_config_file": fastlio_config_file,
                "point_output_topic": point_output_topic,
                "debug_output_topic": debug_output_topic,
                "diag_output_topic": diag_output_topic,
                "debug_uv_output_topic": debug_uv_output_topic,
                "publish_debug_uv": publish_debug_uv,
                "debug_overlay_frame_count": debug_overlay_frame_count,
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

    debug_view = Node(
        package="roi_lidar_corner",
        executable="roi_lidar_debug_view.py",
        name="roi_lidar_debug_view",
        output="screen",
        parameters=[
            {
                "enabled": open_debug_window,
                "image_topic": debug_image_topic,
                "window_name": "ROI Debug Window",
            }
        ],
    )

    launch_actions = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("fastlio_config_path", default_value=fastlio_default_cfg),
        DeclareLaunchArgument("fastlio_config_file", default_value="mid360.yaml"),
        DeclareLaunchArgument("semantic_mapping_en", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("rviz_cfg", default_value=fastlio_default_rviz),
        DeclareLaunchArgument("enable_livox_driver", default_value="true"),
        DeclareLaunchArgument("livox_driver_launch", default_value="msg_MID360_launch.py"),
        DeclareLaunchArgument("enable_d435i", default_value="true"),
        DeclareLaunchArgument("d435i_camera_name", default_value="camera"),
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("corner_radius", default_value="5"),
        DeclareLaunchArgument("detector_backend", default_value="pt"),
        DeclareLaunchArgument("detector_model_path", default_value=detector_default_model),
        DeclareLaunchArgument("detector_names_path", default_value=detector_default_names),
        DeclareLaunchArgument("detector_conf_threshold", default_value="0.25"),
        DeclareLaunchArgument("detector_iou_threshold", default_value="0.45"),
        DeclareLaunchArgument("detector_input_size", default_value="640"),
        DeclareLaunchArgument("detector_use_gpu", default_value="true"),
        DeclareLaunchArgument("detector_class_filter", default_value="[]"),
        DeclareLaunchArgument("roi_output_topic", default_value="/roi_lidar_corner/front_face_rois"),
        DeclareLaunchArgument("point_output_topic", default_value="/roi_lidar_corner/front_face_corners"),
        DeclareLaunchArgument("debug_output_topic", default_value="/roi_lidar_corner/front_face_debug"),
        DeclareLaunchArgument("diag_output_topic", default_value="/roi_lidar_corner/solver_diag"),
        DeclareLaunchArgument("debug_uv_output_topic", default_value="/roi_lidar_corner/solver_debug_uv"),
        DeclareLaunchArgument("pointcloud_topic", default_value="/cloud_registered"),
        DeclareLaunchArgument("odom_topic", default_value="/Odometry"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info"),
        DeclareLaunchArgument("min_points", default_value="2"),
        DeclareLaunchArgument("max_time_diff_cloud", default_value="1.0"),
        DeclareLaunchArgument("max_time_diff_odom", default_value="0.5"),
        DeclareLaunchArgument("min_range", default_value="0.2"),
        DeclareLaunchArgument("max_range", default_value="30.0"),
        DeclareLaunchArgument("history_window_sec", default_value="1.0"),
        DeclareLaunchArgument("max_window_frames", default_value="30"),
        DeclareLaunchArgument("cache_voxel_size", default_value="0.1"),
        DeclareLaunchArgument("bbox_expand_ratio", default_value="0.15"),
        DeclareLaunchArgument("corner_target_points", default_value="6"),
        DeclareLaunchArgument("corner_target_frames", default_value="2"),
        DeclareLaunchArgument("corner_cap_points", default_value="96"),
        DeclareLaunchArgument("post_max_z_jump_m", default_value="0.8"),
        DeclareLaunchArgument("output_frame_id", default_value="body"),
        DeclareLaunchArgument("output_extrinsic_t_x", default_value="0.0"),
        DeclareLaunchArgument("output_extrinsic_t_y", default_value="0.0"),
        DeclareLaunchArgument("output_extrinsic_t_z", default_value="0.0"),
        DeclareLaunchArgument("output_extrinsic_r_00", default_value="0.0"),
        DeclareLaunchArgument("output_extrinsic_r_01", default_value="0.0"),
        DeclareLaunchArgument("output_extrinsic_r_02", default_value="1.0"),
        DeclareLaunchArgument("output_extrinsic_r_10", default_value="1.0"),
        DeclareLaunchArgument("output_extrinsic_r_11", default_value="0.0"),
        DeclareLaunchArgument("output_extrinsic_r_12", default_value="0.0"),
        DeclareLaunchArgument("output_extrinsic_r_20", default_value="0.0"),
        DeclareLaunchArgument("output_extrinsic_r_21", default_value="1.0"),
        DeclareLaunchArgument("output_extrinsic_r_22", default_value="0.0"),
        DeclareLaunchArgument("publish_debug_image", default_value="true"),
        DeclareLaunchArgument("publish_debug_uv", default_value="true"),
        DeclareLaunchArgument("debug_overlay_frame_count", default_value="1"),
        DeclareLaunchArgument("subscribe_corner3d_debug", default_value="true"),
        DeclareLaunchArgument("subscribe_solver_debug_uv", default_value="true"),
        DeclareLaunchArgument("enable_debug_markers", default_value="true"),
        DeclareLaunchArgument("debug_image_topic", default_value="/roi_lidar_corner/roi_debug"),
        DeclareLaunchArgument("open_debug_window", default_value="false"),
        DeclareLaunchArgument("neo_canny_low", default_value="50"),
        DeclareLaunchArgument("neo_canny_high", default_value="200"),
        DeclareLaunchArgument("neo_hough_threshold", default_value="50"),
        DeclareLaunchArgument("neo_min_line_length", default_value="50"),
        DeclareLaunchArgument("neo_max_line_gap", default_value="50"),
        DeclareLaunchArgument("neo_blur_kernel_size", default_value="5"),
        DeclareLaunchArgument("neo_border_ratio", default_value="0.15"),
    ]
    launch_actions.extend(
        [
            include_livox_driver_launch,
            include_fastlio_launch,
        ]
    )
    if include_d435i_launch is not None:
        launch_actions.append(include_d435i_launch)
    launch_actions.extend(
        [
            roi_generator,
            corner_solver,
            debug_markers,
            debug_view,
        ]
    )
    return LaunchDescription(launch_actions)
