cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

# to show include files in Qt Creator
file(GLOB_RECURSE HEADER_FILES include/*.h)

rosbuild_add_boost_directories()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(simulator src/simulator.cpp
                                  src/Object.cpp
                                  src/Robot.cpp
                                  src/Joint.cpp
                                  src/World.cpp
                                  src/Amigo.cpp
                                  src/Pico.cpp
                                  src/Box.cpp
                                  src/Octree.cpp
                                  src/Ray.cpp
                                  src/Sensor.cpp
                                  src/Sonar.cpp
                                  src/LRF.cpp
                                  src/Sonar.cpp
                                  src/Kinect.cpp
                                  src/Sprite.cpp
                                  src/Shape.cpp
                                  src/Event.cpp
                                  src/ModelParser.cpp
                                  src/util.cpp ${HEADER_FILES})
rosbuild_link_boost(simulator system program_options)
target_link_libraries(simulator tinyxml)

rosbuild_add_executable(amigo_teleporter src/amigo_teleporter.cpp)

