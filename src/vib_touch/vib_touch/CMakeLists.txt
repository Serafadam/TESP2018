cmake_minimum_required(VERSION 2.8.3)
project(vib_touch)

add_definitions("-std=c++11 -march=native")

find_package(
  catkin REQUIRED COMPONENTS roscpp nodelet vib_touch_msgs
# rospy
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp nodelet
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/vib_touch_sensor.cpp
  src/vib_touch_display.cpp
  #src/vib_touch_stft.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${LIBS}
  ${catkin_LIBRARIES} asound fftw3f
)

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()
