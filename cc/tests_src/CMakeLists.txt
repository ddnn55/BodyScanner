cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(main)



FIND_PACKAGE(Boost COMPONENTS REQUIRED program_options unit_test_framework)
find_package(PCL 1.1 REQUIRED)

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMakeModules/;${CMAKE_MODULE_PATH}")
FIND_PACKAGE(GMP REQUIRED)

INCLUDE_DIRECTORIES ("${CMAKE_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES ("${Boost_INCLUDE_DIRS}")
INCLUDE_DIRECTORIES ("${GMP_INCLUDE_DIRS}")
include_directories(${PCL_INCLUDE_DIRS} /home/webaba/Downloads/libdai/include/ )
link_directories(${PCL_LIBRARY_DIRS} /home/webaba/Downloads/libdai/lib/)
add_definitions(${PCL_DEFINITIONS})

SET (EXTRA_LIBS "${GMP_LIBRARIES}")

add_executable (main main.cpp CCPointSet.cpp Logger.cpp CCTools.cpp CCFactorGraph.cpp CCMesh.cpp CCModelMesh.cpp CCPartialMesh.cpp)
target_link_libraries (main ${PCL_LIBRARIES} dai ${EXTRA_LIBS})
