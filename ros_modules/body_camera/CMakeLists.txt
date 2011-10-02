cmake_minimum_required (VERSION 2.4.6)
include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set (ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init ()
rosbuild_add_boost_directories ()
rosbuild_check_for_sse ()

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Dynamic reconfigure
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

# OpenNI Driver standalone library
rosbuild_add_library (openni_driver src/openni_driver.cpp
                                    src/openni_device.cpp
                                    src/openni_exception.cpp
                                    src/openni_device_primesense.cpp
                                    src/openni_device_kinect.cpp
                                    src/openni_device_xtion.cpp
                                    src/openni_device_oni.cpp
                                    src/openni_image_yuv_422.cpp
                                    src/openni_image_bayer_grbg.cpp
                                    src/openni_image_rgb24.cpp
				    src/openni_ir_image.cpp
                                    src/openni_depth_image.cpp
                     )
rosbuild_add_compile_flags (openni_driver ${SSE_FLAGS})
target_link_libraries(openni_driver usb-1.0)

# ROS nodelets library
rosbuild_add_library (openni_nodelet src/nodelets/driver.cpp src/nodelets/openni_nodelet.cpp)
rosbuild_add_compile_flags (openni_nodelet ${SSE_FLAGS})
target_link_libraries (openni_nodelet openni_driver)

# Node version of ROS driver
rosbuild_add_executable (openni_node_unstable src/nodes/openni_node_unstable.cpp)
rosbuild_add_executable (openni_node src/nodes/openni_node.cpp)
