cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(BodyPuppet)

find_package(PCL 1.2 REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ../Libraries/of_preRelease_v007_linux64/libs/openFrameworks
  ../Libraries/of_preRelease_v007_linux64/libs/openFrameworks/app
  ../Libraries/of_preRelease_v007_linux64/libs/openFrameworks/3d
  ../Libraries/of_preRelease_v007_linux64/libs/openFrameworks/communication
  ../Libraries/of_preRelease_v007_linux64/libs/openFrameworks/gl
  ../Libraries/of_preRelease_v007_linux64/libs/openFrameworks/graphics
  ../Libraries/of_preRelease_v007_linux64/libs/openFrameworks/utils
  ../Libraries/of_preRelease_v007_linux64/libs/openFrameworks/types
  ../Libraries/of_preRelease_v007_linux64/libs/openFrameworks/math
  ../Libraries/of_preRelease_v007_linux64/libs/openFrameworks/events
  ../Libraries/of_preRelease_v007_linux64/libs/openFrameworks/sound
  ../Libraries/of_preRelease_v007_linux64/libs/openFrameworks/video
  ../Libraries/of_preRelease_v007_linux64/addons/ofxOsc/src
  ../Libraries/of_preRelease_v007_linux64/libs/tess2/include
  ../Libraries/of_preRelease_v007_linux64/libs/poco/include
  ../Libraries/of_preRelease_v007_linux64/libs/cairo/include
  ../Libraries/of_preRelease_v007_linux64/libs/cairo/include/cairo
  ../Libraries/of_preRelease_v007_linux64/libs/kiss/include
  ../Libraries/of_preRelease_v007_linux64/addons/ofxOsc/libs/oscpack/include/osc
  ../Libraries/of_preRelease_v007_linux64/addons/ofxOsc/libs/oscpack/include/ip
  ../Libraries/of_preRelease_v007_linux64/addons/ofxAssimpModelLoader/src
  ../Libraries/of_preRelease_v007_linux64/addons/ofxAssimpModelLoader/libs/assimp/include
  ../Modules/body-grabber/include
  /usr/include/gstreamer-0.10
  /usr/include/glib-2.0
  /usr/lib/x86_64-linux-gnu/glib-2.0/include
  /usr/include/libxml2
  /usr/include/openni
)

cmake_policy(SET CMP0015 NEW) # so the oF relative path works
link_directories(
  ${PCL_LIBRARY_DIRS}
  ../Libraries/of_preRelease_v007_linux64/libs/openFrameworksCompiled/lib/linux64 
  ../Libraries/of_preRelease_v007_linux64/libs/tess2/lib/linux64 
  ../Libraries/of_preRelease_v007_linux64/libs/kiss/lib/linux64
  ../Libraries/of_preRelease_v007_linux64/libs/poco/lib/linux64
  ../Libraries/of_preRelease_v007_linux64/addons/ofxOsc/libs/oscpack/lib/linux64
  ../Libraries/of_preRelease_v007_linux64/addons/ofxAssimpModelLoader/libs/assimp/lib/linux64
)

add_definitions(${PCL_DEFINITIONS}

)

add_executable (BodyPuppet
  src/main.cpp
  src/BodyPuppetApp.cpp
  ../Libraries/of_preRelease_v007_linux64/addons/ofxOsc/src/ofxOscMessage.cpp
  ../Libraries/of_preRelease_v007_linux64/addons/ofxOsc/src/ofxOscReceiver.cpp
  ../Libraries/of_preRelease_v007_linux64/addons/ofxOsc/src/ofxOscSender.cpp
  ../Libraries/of_preRelease_v007_linux64/addons/ofxOsc/src/ofxOscBundle.cpp
  ../Libraries/of_preRelease_v007_linux64/addons/ofxAssimpModelLoader/src/ofxAssimpModelLoader.cpp
  ../Modules/body-grabber/src/Body/Skeleton/Pose.cpp
  ../Modules/body-grabber/src/Body/Skin.cpp
)

target_link_libraries (BodyPuppet
  ${PCL_LIBRARIES}
  openFrameworks openal mpg123 tess2 cairo GL kiss GLEW PocoNet PocoUtil PocoXML PocoFoundation sndfile freeimage glut GLU oscpack assimp
)
