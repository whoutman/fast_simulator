#ifndef _FAST_SIMULATOR_WORLD_H_
#define _FAST_SIMULATOR_WORLD_H_

#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <string>

#include "fast_simulator/Object.h"
#include "fast_simulator/Box.h"

class World {

public:

    static World& getInstance();

    virtual ~World();

    World(const World& orig);

    World getCopy() const;

    void createQuadTree(const nav_msgs::OccupancyGrid& map, unsigned int mx_min, unsigned int my_min,
             unsigned int mx_max, unsigned int my_max, Object* parent, std::string indent = "");

    void initFromTopic(const std::string& topic);

    void step(double dt);

    void addObject(const std::string& id, Object* obj);

    Object* getObject(const std::string& id) const;

    void removeObject(const std::string& id);

    const std::map<std::string, Object*>& getObjects() const;

    bool isOccupied(const tf::Vector3& pos) const;

    bool intersect(const Ray& r, float t0, float t1, double& distance) const;

protected:

    static World* instance_;

    nav_msgs::OccupancyGrid world_map_;

    tf::Transform map_transform_;
    tf::Transform map_transform_inverse_;

    //std::vector<Object*> objects_;
    std::map<std::string, Object*> objects_;

    World();

    void callbackMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);

};


#endif
