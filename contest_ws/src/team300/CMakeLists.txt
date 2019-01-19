cmake_minimum_required(VERSION 2.8.3)
project(team300)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
)

set(OpenCV_STATIC OFF)
find_package(  OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES team300_lib
  CATKIN_DEPENDS roscpp
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(team300_lib
  src/imageprocessing.cpp
  src/signdetect.cpp
  src/lanedetect.cpp
  src/carcontrol.cpp
)

add_executable(team300 src/main.cpp)

add_dependencies(team300 ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  team300
  team300_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)
