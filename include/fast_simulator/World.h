#ifndef _FAST_SIMULATOR_WORLD_H_
#define _FAST_SIMULATOR_WORLD_H_

#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <string>

#include "fast_simulator/Object.h"

class World {

public:

    World();

    virtual ~World();

    void initFromTopic(const std::string& topic);

    void step(double dt);

    void addObject(Object* obj);

    bool isOccupied(const tf::Vector3& pos);

protected:

    nav_msgs::OccupancyGrid world_map_;

    tf::Transform map_transform_;
    tf::Transform map_transform_inverse_;

    std::vector<Object*> objects_;

    void callbackMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);

};


#endif
