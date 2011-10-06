cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(icp_align)

find_package(PCL 1.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (segmentation2 segmentation2.cpp)
target_link_libraries (segmentation2 ${PCL_LIBRARIES})
