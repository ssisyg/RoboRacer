cmake_minimum_required(VERSION 3.0.2)
project(f1tenth_obstacle_avoidance)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  laser_geometry
  message_generation
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

add_message_files(
  FILES
  TrackedObject.msg
  TrackedObjectArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs geometry_msgs nav_msgs sensor_msgs tf2_ros laser_geometry
)

catkin_install_python(PROGRAMS
  nodes/lidar_perception_node.py
  nodes/behavioral_planner_fsm_node.py
  nodes/pure_pursuit_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)