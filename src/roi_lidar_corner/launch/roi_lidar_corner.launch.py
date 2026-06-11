import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    roi_lidar_corner_share = get_package_share_directory("roi_lidar_corner")
    default_config_file = os.path.join(roi_lidar_corner_share, "config", "roi_lidar_corner.yaml")
    detector_default_model = os.path.join(roi_lidar_corner_share, "models", "best.pt")
    detector_default_names = os.path.join(roi_lidar_corner_share, "models", "detect.names")
    config_file_arg = DeclareLaunchArgument("config_file", default_value=default_config_file)
    detector_backend_arg = DeclareLaunchArgument("detector_backend", default_value="pt")
    detector_model_path_arg = DeclareLaunchArgument("detector_model_path", default_value=detector_default_model)
    detector_names_path_arg = DeclareLaunchArgument("detector_names_path", default_value=detector_default_names)
    detector_conf_threshold_arg = DeclareLaunchArgument("detector_conf_threshold", default_value="0.25")
    detector_iou_threshold_arg = DeclareLaunchArgument("detector_iou_threshold", default_value="0.45")
    detector_input_size_arg = DeclareLaunchArgument("detector_input_size", default_value="640")
    detector_use_gpu_arg = DeclareLaunchArgument("detector_use_gpu", default_value="true")
    detector_class_filter_arg = DeclareLaunchArgument("detector_class_filter", default_value="[]")
    image_topic_arg = DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw")
    corner_radius_arg = DeclareLaunchArgument("corner_radius", default_value="5")
    publish_debug_image_arg = DeclareLaunchArgument(
        "publish_debug_image", default_value="true"
    )
    publish_debug_uv_arg = DeclareLaunchArgument("publish_debug_uv", default_value="true")
    debug_overlay_frame_count_arg = DeclareLaunchArgument("debug_overlay_frame_count", default_value="1")
    subscribe_corner3d_debug_arg = DeclareLaunchArgument("subscribe_corner3d_debug", default_value="true")
    subscribe_solver_debug_uv_arg = DeclareLaunchArgument("subscribe_solver_debug_uv", default_value="true")
    enable_debug_markers_arg = DeclareLaunchArgument("enable_debug_markers", default_value="true")
    debug_image_topic_arg = DeclareLaunchArgument(
        "debug_image_topic", default_value="/roi_lidar_corner/roi_debug"
    )
    roi_output_topic_arg = DeclareLaunchArgument("roi_output_topic", default_value="/roi_lidar_corner/front_face_rois")
    point_output_topic_arg = DeclareLaunchArgument("point_output_topic", default_value="/roi_lidar_corner/front_face_corners")
    debug_output_topic_arg = DeclareLaunchArgument("debug_output_topic", default_value="/roi_lidar_corner/front_face_debug")
    diag_output_topic_arg = DeclareLaunchArgument("diag_output_topic", default_value="/roi_lidar_corner/solver_diag")
    debug_uv_output_topic_arg = DeclareLaunchArgument(
        "debug_uv_output_topic", default_value="/roi_lidar_corner/solver_debug_uv"
    )
    pointcloud_topic_arg = DeclareLaunchArgument("pointcloud_topic", default_value="/points")
    odom_topic_arg = DeclareLaunchArgument("odom_topic", default_value="/odom")
    camera_info_topic_arg = DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info")
    min_points_arg = DeclareLaunchArgument("min_points", default_value="2")
    max_time_diff_cloud_arg = DeclareLaunchArgument("max_time_diff_cloud", default_value="1.0")
    max_time_diff_odom_arg = DeclareLaunchArgument("max_time_diff_odom", default_value="0.5")
    min_range_arg = DeclareLaunchArgument("min_range", default_value="0.2")
    max_range_arg = DeclareLaunchArgument("max_range", default_value="30.0")
    history_window_sec_arg = DeclareLaunchArgument("history_window_sec", default_value="1.0")
    max_window_frames_arg = DeclareLaunchArgument("max_window_frames", default_value="30")
    cache_voxel_size_arg = DeclareLaunchArgument("cache_voxel_size", default_value="0.1")
    bbox_expand_ratio_arg = DeclareLaunchArgument("bbox_expand_ratio", default_value="0.15")
    corner_target_points_arg = DeclareLaunchArgument("corner_target_points", default_value="6")
    corner_target_frames_arg = DeclareLaunchArgument("corner_target_frames", default_value="2")
    corner_cap_points_arg = DeclareLaunchArgument("corner_cap_points", default_value="96")
    post_max_z_jump_m_arg = DeclareLaunchArgument("post_max_z_jump_m", default_value="0.8")
    output_frame_id_arg = DeclareLaunchArgument("output_frame_id", default_value="body")
    output_extrinsic_t_x_arg = DeclareLaunchArgument("output_extrinsic_t_x", default_value="0.0")
    output_extrinsic_t_y_arg = DeclareLaunchArgument("output_extrinsic_t_y", default_value="0.0")
    output_extrinsic_t_z_arg = DeclareLaunchArgument("output_extrinsic_t_z", default_value="0.0")
    output_extrinsic_r_00_arg = DeclareLaunchArgument("output_extrinsic_r_00", default_value="0.0")
    output_extrinsic_r_01_arg = DeclareLaunchArgument("output_extrinsic_r_01", default_value="0.0")
    output_extrinsic_r_02_arg = DeclareLaunchArgument("output_extrinsic_r_02", default_value="1.0")
    output_extrinsic_r_10_arg = DeclareLaunchArgument("output_extrinsic_r_10", default_value="1.0")
    output_extrinsic_r_11_arg = DeclareLaunchArgument("output_extrinsic_r_11", default_value="0.0")
    output_extrinsic_r_12_arg = DeclareLaunchArgument("output_extrinsic_r_12", default_value="0.0")
    output_extrinsic_r_20_arg = DeclareLaunchArgument("output_extrinsic_r_20", default_value="0.0")
    output_extrinsic_r_21_arg = DeclareLaunchArgument("output_extrinsic_r_21", default_value="1.0")
    output_extrinsic_r_22_arg = DeclareLaunchArgument("output_extrinsic_r_22", default_value="0.0")
    neo_canny_low_arg = DeclareLaunchArgument("neo_canny_low", default_value="50")
    neo_canny_high_arg = DeclareLaunchArgument("neo_canny_high", default_value="200")
    neo_hough_threshold_arg = DeclareLaunchArgument("neo_hough_threshold", default_value="50")
    neo_min_line_length_arg = DeclareLaunchArgument("neo_min_line_length", default_value="50")
    neo_max_line_gap_arg = DeclareLaunchArgument("neo_max_line_gap", default_value="50")
    neo_blur_kernel_size_arg = DeclareLaunchArgument("neo_blur_kernel_size", default_value="5")
    neo_border_ratio_arg = DeclareLaunchArgument("neo_border_ratio", default_value="0.15")

    config_file = LaunchConfiguration("config_file")
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
    publish_debug_image = LaunchConfiguration("publish_debug_image")
    publish_debug_uv = LaunchConfiguration("publish_debug_uv")
    debug_overlay_frame_count = LaunchConfiguration("debug_overlay_frame_count")
    subscribe_corner3d_debug = LaunchConfiguration("subscribe_corner3d_debug")
    subscribe_solver_debug_uv = LaunchConfiguration("subscribe_solver_debug_uv")
    enable_debug_markers = LaunchConfiguration("enable_debug_markers")
    debug_image_topic = LaunchConfiguration("debug_image_topic")
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
    neo_canny_low = LaunchConfiguration("neo_canny_low")
    neo_canny_high = LaunchConfiguration("neo_canny_high")
    neo_hough_threshold = LaunchConfiguration("neo_hough_threshold")
    neo_min_line_length = LaunchConfiguration("neo_min_line_length")
    neo_max_line_gap = LaunchConfiguration("neo_max_line_gap")
    neo_blur_kernel_size = LaunchConfiguration("neo_blur_kernel_size")
    neo_border_ratio = LaunchConfiguration("neo_border_ratio")

    roi_generator = Node(
        package="roi_lidar_corner",
        executable="roi_generator_node.py",
        name="roi_generator_node",
        output="screen",
        parameters=[
            config_file,
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
                "corner3d_topic": point_output_topic,
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
            config_file,
            {
                "roi_topic": roi_output_topic,
                "image_topic": image_topic,
                "pointcloud_topic": pointcloud_topic,
                "odom_topic": odom_topic,
                "camera_info_topic": camera_info_topic,
                "point_output_topic": point_output_topic,
                "debug_output_topic": debug_output_topic,
                "diag_output_topic": diag_output_topic,
                "debug_uv_output_topic": debug_uv_output_topic,
                "publish_debug_uv": publish_debug_uv,
                "debug_overlay_frame_count": debug_overlay_frame_count,
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

    return LaunchDescription(
        [
            config_file_arg,
            detector_backend_arg,
            detector_model_path_arg,
            detector_names_path_arg,
            detector_conf_threshold_arg,
            detector_iou_threshold_arg,
            detector_input_size_arg,
            detector_use_gpu_arg,
            detector_class_filter_arg,
            image_topic_arg,
            corner_radius_arg,
            publish_debug_image_arg,
            publish_debug_uv_arg,
            debug_overlay_frame_count_arg,
            subscribe_corner3d_debug_arg,
            subscribe_solver_debug_uv_arg,
            enable_debug_markers_arg,
            debug_image_topic_arg,
            roi_output_topic_arg,
            point_output_topic_arg,
            debug_output_topic_arg,
            diag_output_topic_arg,
            debug_uv_output_topic_arg,
            pointcloud_topic_arg,
            odom_topic_arg,
            camera_info_topic_arg,
            min_points_arg,
            max_time_diff_cloud_arg,
            max_time_diff_odom_arg,
            min_range_arg,
            max_range_arg,
            history_window_sec_arg,
            max_window_frames_arg,
            cache_voxel_size_arg,
            bbox_expand_ratio_arg,
            corner_target_points_arg,
            corner_target_frames_arg,
            corner_cap_points_arg,
            post_max_z_jump_m_arg,
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
            neo_canny_low_arg,
            neo_canny_high_arg,
            neo_hough_threshold_arg,
            neo_min_line_length_arg,
            neo_max_line_gap_arg,
            neo_blur_kernel_size_arg,
            neo_border_ratio_arg,
            roi_generator,
            corner_solver,
            debug_markers,
        ]
    )
