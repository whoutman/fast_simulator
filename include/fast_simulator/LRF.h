#ifndef _FAST_SIMULATOR_LRF_H_
#define _FAST_SIMULATOR_LRF_H_

#include "Object.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LRF : public Object {

public:

    LRF(const std::string& topic);

    virtual ~LRF();

    void publishScan();

protected:

    sensor_msgs::LaserScan scan;

    ros::Publisher pub_laser_scan;

    std::vector<tf::Vector3> laser_ray_deltas_;

};

#endif
