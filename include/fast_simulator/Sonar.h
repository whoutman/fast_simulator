#ifndef _FAST_SIMULATOR_SONAR_H_
#define _FAST_SIMULATOR_SONAR_H_

#include "Sensor.h"

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

class Sonar : public Sensor {

public:

    Sonar(const std::string& topic, const std::string& frame_id);

    virtual ~Sonar();

    void step(World& world);

protected:

    std::string frame_id_;

    sensor_msgs::Range output_;

    ros::Publisher pub_;

    std::vector<geo::Vector3> ray_deltas_;

};

#endif
