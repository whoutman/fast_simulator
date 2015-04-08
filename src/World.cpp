#include <ros/ros.h>

#include "fast_simulator/World.h"
//#include "fast_simulator/Sprite.h"
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

void World::destroy()
{
    delete instance_;
}

void World::step(double dt) {
    for(map<string, Object*>::iterator it_obj = objects_.begin(); it_obj != objects_.end(); ++it_obj) {
        Object* obj = it_obj->second;
        obj->step(dt);
    }
}

void World::addObject(const std::string& id, Object* obj) {
    obj->id_ = id;
    objects_[id] = obj;
}

Object* World::getObject(const std::string& id) const {
    map<string, Object*>::const_iterator it_obj = objects_.find(id);
    if (it_obj != objects_.end()) {
        return it_obj->second;
    }
    return 0;
}

vector<Object*> World::getObjectsRecursive() const {
    vector<Object*> objects;

    for(map<string, Object*>::const_iterator it_obj = objects_.begin(); it_obj != objects_.end(); ++it_obj) {
        objects.push_back(it_obj->second);
        it_obj->second->getChildrenRecursive(objects);
    }

    return objects;
}

void World::removeObject(const std::string& id) {
    objects_.erase(id);
}

const std::map<std::string, Object*>& World::getObjects() const {
    return objects_;
}

bool World::intersect(const geo::Ray& r, float t0, float t1, double& distance) const {
    distance = t1;
    bool has_intersection = false;
    for(map<string, Object*>::const_iterator it_obj = objects_.begin(); it_obj != objects_.end(); ++it_obj) {
        Object* obj = it_obj->second;
        double dist;
        if (obj->intersect(r, t0, distance, dist)) {
            has_intersection = true;
            distance = min (dist, distance);
        }
    }

    return has_intersection;

}

/*
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
*/
