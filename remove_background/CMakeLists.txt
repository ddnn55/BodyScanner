cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(remove_outliers)

find_package(PCL 1.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (background_removal background_removal.cpp)
target_link_libraries (background_removal ${PCL_LIBRARIES})

