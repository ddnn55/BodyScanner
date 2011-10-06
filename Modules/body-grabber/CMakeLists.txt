cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(BodyScanner)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} "include" /usr/include/openni)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (BodyScanner src/BodyScanner.cpp src/Body/Skeleton/Pose.cpp src/pcl_addons/io/openni_human_grabber.cpp src/pcl_addons/io/openni_camera/openni_driver_nite.cpp src/Body/BodySegmentation.cpp)
target_link_libraries (BodyScanner ${PCL_LIBRARIES})
