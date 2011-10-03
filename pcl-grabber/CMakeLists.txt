cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(BodyScanner)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} "include")
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (BodyScanner BodyScanner.cpp)
target_link_libraries (BodyScanner ${PCL_LIBRARIES})
