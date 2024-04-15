cmake_minimum_required(VERSION 3.0.2)
project(map_downsample)

add_compile_options(-std=c++11 -O3)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)
