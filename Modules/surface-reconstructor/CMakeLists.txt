cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(surface-reconstructor)

find_package(PCL 1.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (surface-reconstructor surface-reconstructor.cpp)
target_link_libraries (surface-reconstructor ${PCL_LIBRARIES})

configure_file(vtk2obj.py vtk2obj.py)

