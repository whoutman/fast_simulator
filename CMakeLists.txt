cmake_minimum_required(VERSION 2.8.3)
project(fast_simulator)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    tue_msgs
    cv_bridge
    geolib2
    geometry_msgs
    image_transport
    image_geometry
    kdl_parser
    nav_msgs
    roslib
    sensor_msgs
    std_msgs
    tf
    tf_conversions
    tue_filesystem
    virtual_cam
    ed
    tue_config
)

################################################
#                                              #
#                PYTHON SETUP                  #
#                                              #
################################################

catkin_python_setup()

################################################
#                                              #
#      ROS MESSAGES, SERVICES AND ACTIONS      #
#                                              #
################################################

# Generate services in the 'srv' folder
add_service_files(
    FILES
    SetObject.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)

################################################
#                                              #
#       CATKIN SPECIFIC CONFIGURATION          #
#                                              #
################################################

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

#SET(HEADER_FILES
#    include/fast_simulator/simulator.h
#    include/fast_simulator/Joint.h
#    include/fast_simulator/Pico.h
#    include/fast_simulator/Object.h
#    include/fast_simulator/LRF.h
#    include/fast_simulator/util.h
#    include/fast_simulator/StandaloneLRF.h
#    include/fast_simulator/SimulatorROS.h
#    include/fast_simulator/Sensor.h
#    include/fast_simulator/StandaloneKinect.h
#    include/fast_simulator/Sergio.h
#    include/fast_simulator/ModelParser.h
#    include/fast_simulator/Pera.h
#    include/fast_simulator/Simulator.h
#    include/fast_simulator/World.h
#    include/fast_simulator/Robot.h
#    include/fast_simulator/Amigo.h
#    include/fast_simulator/Sonar.h
#    include/fast_simulator/Kinect.h
#    include/fast_simulator/Event.h
#)

add_executable(simulator
    src/simulator.cpp
    src/Simulator.cpp
    src/SimulatorROS.cpp
    src/Object.cpp
    src/Robot.cpp
    src/Joint.cpp
    src/World.cpp
    src/Amigo.cpp
    src/Sergio.cpp
    src/Pico.cpp
    src/Pera.cpp
    src/StandaloneKinect.cpp
    src/StandaloneLRF.cpp
    src/Sensor.cpp
    src/Sonar.cpp
    src/LRF.cpp
    src/Sonar.cpp
    src/Kinect.cpp
    src/Event.cpp
    src/ModelParser.cpp
    src/util.cpp

    include/fast_simulator/simulator.h
    include/fast_simulator/Joint.h
    include/fast_simulator/Pico.h
    include/fast_simulator/Object.h
    include/fast_simulator/LRF.h
    include/fast_simulator/util.h
    include/fast_simulator/StandaloneLRF.h
    include/fast_simulator/SimulatorROS.h
    include/fast_simulator/Sensor.h
    include/fast_simulator/StandaloneKinect.h
    include/fast_simulator/Sergio.h
    include/fast_simulator/ModelParser.h
    include/fast_simulator/Pera.h
    include/fast_simulator/Simulator.h
    include/fast_simulator/World.h
    include/fast_simulator/Robot.h
    include/fast_simulator/Amigo.h
    include/fast_simulator/Sonar.h
    include/fast_simulator/Kinect.h
    include/fast_simulator/Event.h
)

target_link_libraries(simulator tinyxml ${catkin_LIBRARIES})
add_dependencies(simulator ${PROJECT_NAME}_gencpp)

