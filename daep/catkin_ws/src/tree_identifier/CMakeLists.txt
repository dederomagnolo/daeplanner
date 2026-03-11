cmake_minimum_required(VERSION 2.8.3)
project(tree_identifier)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_sensor_msgs
)

add_message_files(
  FILES
  TreeDetection.msg
  TreeDetectionArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_sensor_msgs
)

catkin_install_python(PROGRAMS
  scripts/tree_id_tracker.py
  scripts/tree_detector_node.py
  scripts/tree_xy_plotter.py
  scripts/tree_cluster_xy_plotter.py
  scripts/tree_map_fuser.py
  scripts/tree_map_csv_plotter.py
  scripts/tree_cluster_pkl_plotter.py
  scripts/world_tree_ground_truth_plotter.py
  scripts/world_tree_compare_plotter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
