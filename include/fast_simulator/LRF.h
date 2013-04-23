#ifndef _FAST_SIMULATOR_LRF_H_
#define _FAST_SIMULATOR_LRF_H_

#include "Object.h"

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LRF : public Object {

public:

    LRF(const std::string& topic, const std::string& frame_id);

    virtual ~LRF();

    void publishScan();

protected:

    std::string frame_id_;

    sensor_msgs::LaserScan scan;

    ros::Publisher pub_laser_scan;

    std::vector<tf::Vector3> laser_ray_deltas_;

};

#endif
