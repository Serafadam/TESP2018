cmake_minimum_required(VERSION 2.8.3)
project(vib_touch_ui)

find_package(catkin REQUIRED std_msgs vib_touch_msgs)

catkin_package(
  CATKIN_DEPENDS std_msgs vib_touch_msgs
)