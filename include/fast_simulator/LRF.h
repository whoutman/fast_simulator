#ifndef _FAST_SIMULATOR_LRF_H_
#define _FAST_SIMULATOR_LRF_H_

#include "Sensor.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LRF : public Sensor {

public:

    LRF(const std::string& topic, const std::string& frame_id);

    virtual ~LRF();

    void step(World& world);

protected:

    std::string frame_id_;

    sensor_msgs::LaserScan scan;

    ros::Publisher pub_laser_scan;

    std::vector<tf::Vector3> laser_ray_deltas_;

};

#endif
