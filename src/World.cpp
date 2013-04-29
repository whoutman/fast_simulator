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


    [ INFO] [1366965139.472475142]: Setting pose: 8.738 2.636 -0.498 [frame=/map]
    [ INFO] [1366965147.293481815]: Setting pose: 7.772 5.019 -0.708 [frame=/map]
    */

    /*
    Object* large_table = new Object("table");
    large_table->setShape(Box(tf::Vector3(-1.9, -0.7, 0), tf::Vector3(1.9, 0.7, 1)));
    large_table->setPose(tf::Vector3(5.9, 7.7, 0.5), tf::Quaternion(0, 0, 0, 1));
    addObject("table-large", large_table);

    Object* fridge = new Object("fridge");
    fridge->setShape(Box(tf::Vector3(-0.25, -0.35, 0), tf::Vector3(0.25, 0.35, 0.8)));
    fridge->setPose(tf::Vector3(5.35, 3.77, 0.4), tf::Quaternion(0, 0, 0, 1));
    addObject("fridge", fridge);

    Object* chairs_and_table = new Object("furniture");
    chairs_and_table->setShape(Box(tf::Vector3(-0.5, -1.2, 0), tf::Vector3(0.5, 1.2, 0.6)));
    chairs_and_table->setPose(tf::Vector3(8.2, 3.8, 0.3), tf::Quaternion(0, 0, 0, 1));
    addObject("chairs-and-table-1", chairs_and_table);
    */
}

World::~World() {
    for(map<string, Object*>::const_iterator it_obj = objects_.begin(); it_obj != objects_.end(); ++it_obj) {
        delete it_obj->second;
    }
}

World::World(const World& orig) {
    for(map<string, Object*>::const_iterator it_obj = orig.objects_.begin(); it_obj != orig.objects_.end(); ++it_obj) {
        objects_[it_obj->first] = new Object(*it_obj->second);
    }
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
    obj->description_->id_ = id;
    objects_[id] = obj;
}

Object* World::getObject(const std::string& id) const {
    map<string, Object*>::const_iterator it_obj = objects_.find(id);
    if (it_obj != objects_.end()) {
        return it_obj->second;
    }
    return 0;
}

void World::removeObject(const std::string& id) {
        objects_.erase(id);
}

const std::map<std::string, Object*>& World::getObjects() const {
    return objects_;
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

    parent->addChild(obj);
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

    Object* root = new Object("world");
    //root->pose_ = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1, 2, 3));



    createQuadTree(world_map_, 0, 0, world_map_.info.width, world_map_.info.height, root);

    Object* floor = new Object();
    floor->setShape(Box(tf::Vector3(-100, -100, -0.2), tf::Vector3(100, 100, 0)));
    root->addChild(floor);

    addObject("world", root);



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
