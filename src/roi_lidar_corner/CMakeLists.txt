cmake_minimum_required(VERSION 3.8)
project(roi_lidar_corner)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)

# External provider of FrontFaceCorners. Replace this package name if the
# agreed message package changes.
set(FRONT_FACE_CORNERS_MSG_PKG rotor_swarm_msgs)
find_package(${FRONT_FACE_CORNERS_MSG_PKG} REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CornerROI.msg"
  "msg/ObjectROI.msg"
  "msg/ObjectROIArray.msg"
  "msg/Corner3D.msg"
  "msg/Corner3DArray.msg"
  "msg/StructureROI.msg"
  "msg/FrontFaceROI.msg"
  "msg/FrontFaceROIArray.msg"
  "msg/FrontFaceDebug.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs ${FRONT_FACE_CORNERS_MSG_PKG}
)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    roi_lidar_corner/roi_generator_node.py
    roi_lidar_corner/corner_lidar_solver_node.py
    roi_lidar_corner/lidar_projection_exposure_capture.py
    roi_lidar_corner/roi_lidar_debug_markers.py
    roi_lidar_corner/roi_lidar_debug_view.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
