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
    pcl_ros
    roslib
    sensor_msgs
    std_msgs
    tf
    tf_conversions
    tue_filesystem
    virtual_cam
    ed
)

find_package(Boost REQUIRED COMPONENTS system program_options)

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

catkin_package(
    CATKIN_DEPENDS message_runtime
)

################################################
#                                              #
#                    BUILD                     #
#                                              #
################################################

include_directories(include ${catkin_INCLUDE_DIRS})

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
    src/util.cpp ${HEADER_FILES})
#rosbuild_link_boost(simulator system program_options)
target_link_libraries(simulator tinyxml ${Boost_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(robot_teleporter src/robot_teleporter.cpp)
#target_link_libraries(robot_teleporter ${catkin_LIBRARIES})

# TODO: Add the following line for each target that uses one or more messages / services defined in this package:
add_dependencies(simulator ${PROJECT_NAME}_gencpp)

