
cmake_minimum_required(VERSION 3.3)
project(ekf_saut)

find_package(catkin REQUIRED
 COMPONENTS
 rospy
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS)
