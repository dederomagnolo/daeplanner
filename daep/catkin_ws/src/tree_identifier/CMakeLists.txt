cmake_minimum_required(VERSION 2.8.3)
project(tree_identifier)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/tree_id_tracker.py
  scripts/tree_detector_node.py
  scripts/tree_xy_plotter.py
  scripts/tree_map_fuser.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
