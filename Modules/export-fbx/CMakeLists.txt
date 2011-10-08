cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(export-fbx)

find_package(PCL 1.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (MakeMesh MakeMesh.cpp)
target_link_libraries (MakeMesh ${PCL_LIBRARIES})
