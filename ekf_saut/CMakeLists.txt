
cmake_minimum_required(VERSION 3.3)
project(ekf_saut)

find_package(catkin REQUIRED
 COMPONENTS
 roscpp
 rospy
 dsor_msgs
 gazebo_msgs
 auv_msgs
 std_msgs
 message_generation
 geometry_msgs
)



catkin_install_python(PROGRAMS sensor_read.py
    DESTINATION $(CATKIN_PACKAGE_BIN_DESTINATION)
)

catkin_python_setup()

add_message_files(
  FILES
  MsgEKF.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)


catkin_package(CATKIN_DEPENDS message_runtime)



