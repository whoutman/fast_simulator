Fast Simulator
======

## Introduction

The fast_simulator is a robot simulator for ROS. It simulates simple kinematic movements, as well as laser and kinect sensor data.

## Installation

**Note: Currently only ROS Hydro is supported ** 

We assume you have successfully installed ROS and set-up a Catkin workspace. Check out the following packages in your workspace:

    cd <your_catkin_workspace>/src

    git clone https://github.com/tue-robotics/fast_simulator.git
    git clone https://github.com/tue-robotics/fast_simulator_data
    git clone https://github.com/tue-robotics/virtual_cam
    git clone https://github.com/tue-robotics/geolib2
    git clone https://github.com/tue-robotics/code_profiler
    git clone https://github.com/tue-robotics/tue_filesystem
    git clone https://github.com/tue-robotics/ed_object_models.git
    git clone https://github.com/tue-robotics/tue_config.git
    git clone https://github.com/tue-robotics/tue_msgs
   
You will also need the following system dependencies:

sudo apt-get install ros-hydro-geometry-msgs ros-hydro-kdl-parser yaml-cpp ros-hydro-roslib ros-hydro-navigation ros-hydro-pcl-ros ros-hydro-tf-conversions libassimp-dev ros-hydro-image-transport ros-hydro-common-msgs ros-hydro-ros-comm ros-hydro-message-generation ros-hydro-stereo-msgs ros-hydro-tf ros-hydro-opencv2 ros-hydro-std-msgs ros-hydro-message-runtime ros-hydro-sensor-msgs ros-hydro-cv-bridge ros-hydro-message-filters ros-hydro-roscpp ros-hydro-image-geometry 
    
This should be sufficient to successfully compile ED:

    cd <your_catkin_workspace>
    catkin_make
    
## Quick Start

...
