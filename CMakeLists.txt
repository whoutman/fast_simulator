cmake_minimum_required(VERSION 2.8.3)
project(fast_simulator)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  ed
  geolib2
  geometry_msgs
  image_geometry
  image_transport
  kdl_parser
  message_generation
  nav_msgs
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  tue_config
  tue_filesystem
  tue_manipulation
  tue_msgs
  virtual_cam
)

catkin_python_setup()

add_service_files(
  FILES
    SetObject.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package()

################################################
#                                              #
#                    BUILD                     #
#                                              #
################################################

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# SET(HEADER_FILES
#   include/fast_simulator/Amigo.h
#   include/fast_simulator/Event.h
#   include/fast_simulator/Joint.h
#   include/fast_simulator/Kinect.h
#   include/fast_simulator/LRF.h
#   include/fast_simulator/ModelParser.h
#   include/fast_simulator/Object.h
#   include/fast_simulator/Pera.h
#   include/fast_simulator/Pico.h
#   include/fast_simulator/Robot.h
#   include/fast_simulator/Sensor.h
#   include/fast_simulator/Sergio.h
#   include/fast_simulator/simulator.h
#   include/fast_simulator/Simulator.h
#   include/fast_simulator/SimulatorROS.h
#   include/fast_simulator/Sonar.h
#   include/fast_simulator/StandaloneKinect.h
#   include/fast_simulator/StandaloneLRF.h
#   include/fast_simulator/util.h
#   include/fast_simulator/World.h
# )

add_executable(simulator
  include/fast_simulator/Amigo.h
  include/fast_simulator/Event.h
  include/fast_simulator/Joint.h
  include/fast_simulator/Kinect.h
  include/fast_simulator/LRF.h
  include/fast_simulator/ModelParser.h
  include/fast_simulator/Object.h
  include/fast_simulator/Pera.h
  include/fast_simulator/Pico.h
  include/fast_simulator/Robot.h
  include/fast_simulator/Sensor.h
  include/fast_simulator/Sergio.h
  include/fast_simulator/simulator.h
  include/fast_simulator/Simulator.h
  include/fast_simulator/SimulatorROS.h
  include/fast_simulator/Sonar.h
  include/fast_simulator/StandaloneKinect.h
  include/fast_simulator/StandaloneLRF.h
  include/fast_simulator/util.h
  include/fast_simulator/World.h

  src/Amigo.cpp
  src/Event.cpp
  src/Joint.cpp
  src/Kinect.cpp
  src/LRF.cpp
  src/ModelParser.cpp
  src/Object.cpp
  src/Pera.cpp
  src/Pico.cpp
  src/Robot.cpp
  src/Sensor.cpp
  src/Sergio.cpp
  src/simulator.cpp
  src/Simulator.cpp
  src/SimulatorROS.cpp
  src/Sonar.cpp
  src/StandaloneKinect.cpp
  src/StandaloneLRF.cpp
  src/util.cpp
  src/World.cpp
)

target_link_libraries(simulator tinyxml ${catkin_LIBRARIES})
add_dependencies(simulator ${PROJECT_NAME}_gencpp)

