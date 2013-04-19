#include <ros/ros.h>

#include "fast_simulator/World.h"

using namespace std;

World::World() {

}

World::~World() {

}

void World::step(double dt) {
    for(vector<Object*>::iterator it_obj = objects_.begin(); it_obj != objects_.end(); ++it_obj) {
        Object* obj = *it_obj;
        obj->step(dt);
    }
}

void World::addObject(Object* obj) {
    objects_.push_back(obj);
    obj->world_ = this;
}

void World::initFromTopic(const std::string &topic) {
    ros::NodeHandle nh;
    ros::Subscriber sub_map = nh.subscribe(topic, 10, &World::callbackMap, this);
    while(ros::ok() && world_map_.data.empty()) {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        ROS_INFO("Waiting for map at topic %s", sub_map.getTopic().c_str());
    }
    ROS_INFO("Map found at topic %s", sub_map.getTopic().c_str());
    sub_map.shutdown();
}

void World::callbackMap(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    world_map_ = *msg;
    tf::poseMsgToTF(world_map_.info.origin, map_transform_);
    map_transform_inverse_ = map_transform_.inverse();
}

bool World::isOccupied(const tf::Vector3& pos) {
    tf::Vector3 pos_map = map_transform_inverse_ * pos;

    int mx = (int)(pos_map.getX() / world_map_.info.resolution);
    int my = (int)(pos_map.getY() / world_map_.info.resolution);

    if (mx < 0 || mx >= (int)world_map_.info.width || my < 0 || my >= (int)world_map_.info.height) {
        return false;
    }

    if (world_map_.data[world_map_.info.width * my + mx] > 10 ) {
        return true;
    }

    return false;
}
