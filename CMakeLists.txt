cmake_minimum_required(VERSION 3.0.2)
project(aruco_detect)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  sensor_msgs
  image_geometry
  geometry_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)

# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS ON)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge sensor_msgs image_transport geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


file(GLOB SOURCES_2
  src/aruco_detector.cpp
)

add_executable(${PROJECT_NAME}_node ${SOURCES_2})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
