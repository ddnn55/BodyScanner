cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project("Kinect-Scanner")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

foreach(subdir 
          data-recorder
          scan-aligner
          surface-reconstructor
          geodesic
          segmentation	
          export-fbx
       )

  add_subdirectory(${subdir})

endforeach()
