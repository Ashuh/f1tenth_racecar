cmake_minimum_required(VERSION 3.0.2)
project(f1tenth_racecar_bringup)

find_package(catkin REQUIRED)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
