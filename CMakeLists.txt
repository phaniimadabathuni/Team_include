cmake_minimum_required(VERSION 3.0.2)
project(team_include)

# 1. Find the ROS libraries we need
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  mavros_msgs
)

# 2. Python Setup
catkin_python_setup()

# 3. Define the build
catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs nav_msgs cv_bridge mavros_msgs
)

# 4. Install the Python Scripts so ROS can find them
# This makes your nodes executable automatically
catkin_install_python(PROGRAMS
  scripts/yolo_detector.py
  scripts/path_planner.py
  scripts/gps_denied_flight.py
  scripts/vins_to_mavros.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 5. Install the Launch Files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)