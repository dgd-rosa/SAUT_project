
cmake_minimum_required(VERSION 3.3)
project(ekf_saut)

find_package(catkin REQUIRED
 COMPONENTS
 rospy
 dsor_msgs
)



catkin_install_python(PROGRAMS sensor_read.py
    DESTINATION $(CATKIN_PACKAGE_BIN_DESTINATION)
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS)
