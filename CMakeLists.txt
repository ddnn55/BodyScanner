cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project("BodyScanner")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

foreach(subdir 
          Modules/body-grabber
          #Modules/Inactive/scan-aligner
          #Modules/Inactive/cc
          Modules/surface-reconstructor
          Modules/geodesic
          Modules/segmentation	
          Modules/export-fbx
       )

  add_subdirectory(${subdir})

endforeach()
