#include <ros/ros.h>

#include "fast_simulator/World.h"
#include "fast_simulator/Sprite.h"
#include "fast_simulator/util.h"

using namespace std;

World* World::instance_ = 0;

/*


*/

World::World() {
    /*
    boxes_.push_back(new Box(tf::Vector3(2.739, -2.532, 0), tf::Vector3(3.107, 1.237, 4)));
    boxes_.push_back(new Box(tf::Vector3(3.529, -2.514, 0), tf::Vector3(3.966, -2.028, 4)));
    boxes_.push_back(new Box(tf::Vector3(4.543, -2.555, 0), tf::Vector3(5.427, -0.478, 4)));
    boxes_.push_back(new Box(tf::Vector3(-0.131, 0.705, 0), tf::Vector3(1.160, 1.228, 4)));
    */

    //Sprite* s = new Sprite("/home/sdries/test.pgm", 0.05, 0, 2);

    Object* obj = new Object();
    obj->setBoundingBox(Box(tf::Vector3(-0.5, -0.5, 0.5), tf::Vector3(0.5, 0.5, 1.5)));
    obj->setShape(Sprite("/home/sdries/laser_body.pgm", 0.025, 0.5, 1.5));
    tf::Quaternion q;
    q.setRPY(0, 0, 1.57);
    obj->setPose(tf::Vector3(2, 0, 0), q);
    //obj->setBoundingBox(Box(tf::Vector3(0, 0, 0), tf::Vector3(1, 1, 1)));
    addObject("person1", obj);
}

World::~World() {

}

World& World::getInstance() {
    if (!instance_) {
        instance_ = new World();
    }
    return *instance_;
}

void World::step(double dt) {
    for(map<string, Object*>::iterator it_obj = objects_.begin(); it_obj != objects_.end(); ++it_obj) {
        Object* obj = it_obj->second;
        obj->step(dt);
    }
}

void World::addObject(const std::string& id, Object* obj) {
    objects_[id] = obj;
}

Object* World::getObject(const std::string& id) const {
    map<string, Object*>::const_iterator it_obj = objects_.find(id);
    if (it_obj != objects_.end()) {
        return it_obj->second;
    }
    return 0;
}

void World::createQuadTree(const nav_msgs::OccupancyGrid& map, unsigned int mx_min, unsigned int my_min,
                                                    unsigned int mx_max, unsigned int my_max, Object* parent, string indent) {

    bool has_cell = false;
    for(unsigned int mx = mx_min; mx < mx_max; ++mx) {
        for(unsigned int my = my_min; my < my_max; ++my) {
            if (map.data[map.info.width * my + mx] > 10 ) {
                has_cell = true;
            }
        }
    }

    if (!has_cell) {
        return;
    }

    Object* obj = new Object();
    tf::Vector3 min_map((double)mx_min * map.info.resolution,
                        (double)my_min * map.info.resolution, 0);
    tf::Vector3 max_map((double)mx_max * map.info.resolution,
                        (double)my_max * map.info.resolution, 0.5);
    obj->setBoundingBox(Box(map_transform_ * min_map, map_transform_ * max_map));
    // parent->addChild(obj, tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)));
    parent->addChild(obj);

    //cout << indent << mx_min << " - " << mx_max << ", " << my_min << " - " << my_max << endl;

    // cout << indent << toString(min_map) << ", " << toString(max_map) << endl;


    if (mx_max - mx_min < 10 || my_max - my_min < 10) {
        for(unsigned int mx = mx_min; mx < mx_max; ++mx) {
            for(unsigned int my = my_min; my < my_max; ++my) {
                if (map.data[map.info.width * my + mx] > 10 ) {
                    tf::Vector3 pos_map((double)mx * map.info.resolution,
                                        (double)my * map.info.resolution, 0);

                    tf::Vector3 pos = map_transform_ * pos_map;

                    Object* child = new Object();
                    child->setShape(Box(pos, tf::Vector3(pos.x() + map.info.resolution,
                                                       pos.y() + map.info.resolution,
                                                       0.5)));
                    obj->addChild(child, tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)));
                }
            }
        }
    } else {

        unsigned int cx = (mx_max + mx_min) / 2;
        unsigned int cy = (my_max + my_min) / 2;

        createQuadTree(map, mx_min, my_min, cx, cy, obj, indent + "    ");
        createQuadTree(map, cx , my_min, mx_max, cy, obj, indent + "    ");
        createQuadTree(map, mx_min, cy , cx, my_max, obj, indent + "    ");
        createQuadTree(map, cx , cy , mx_max, my_max, obj, indent + "    ");
    }
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

    Object* root = new Object();
    //root->pose_ = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 2, 3));
    addObject("world", root);

    createQuadTree(world_map_, 0, 0, world_map_.info.width, world_map_.info.height, root);

}

void World::callbackMap(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    world_map_ = *msg;
    tf::poseMsgToTF(world_map_.info.origin, map_transform_);
    map_transform_inverse_ = map_transform_.inverse();
}

bool World::intersect(const Ray& r, float t0, float t1, double& distance) const {
    distance = t1;
    bool has_intersection = false;
    for(map<string, Object*>::const_iterator it_obj = objects_.begin(); it_obj != objects_.end(); ++it_obj) {
        Object* obj = it_obj->second;
        double dist;
        if (obj->intersect(r, t0, t1, dist)) {
            has_intersection = true;
            distance = min (dist, distance);
        }
    }

    return has_intersection;

}

bool World::isOccupied(const tf::Vector3& pos) const {

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
