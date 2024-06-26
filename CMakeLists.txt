cmake_minimum_required(VERSION 3.0.2)
project(livox_filter_mig)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    pcl_ros
    tf2_ros
    pcl_conversions
    geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uwo_pack
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(filter_raw_cloud_node src/filter_raw_livox_cloud_node.cpp src/cloud_filter.cpp)
target_link_libraries(filter_raw_cloud_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(filter_raw_cloud_node ${catkin_EXPORTED_TARGETS})

add_executable(laser_scan_merge_node src/laser_scan_merge_node.cpp src/laser_scan_filter.cpp)
target_link_libraries(laser_scan_merge_node ${catkin_LIBRARIES})
add_dependencies(laser_scan_merge_node ${catkin_EXPORTED_TARGETS})
