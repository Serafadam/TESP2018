cmake_minimum_required(VERSION 2.8.3)
project(vib_touch_msgs)

find_package(catkin REQUIRED std_msgs genmsg)

add_message_files(
  FILES
  VibArray.msg
  Spectrum.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
)

