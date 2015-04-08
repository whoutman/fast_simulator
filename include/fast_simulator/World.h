#ifndef _FAST_SIMULATOR_WORLD_H_
#define _FAST_SIMULATOR_WORLD_H_

#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <string>

#include "fast_simulator/Object.h"
//#include "fast_simulator/Box.h"

class World {

public:

    static World& getInstance();

    virtual ~World();

    static void destroy();

    World(const World& orig);

    World getCopy() const;

    void step(double dt);

    void addObject(const std::string& id, Object* obj);

    Object* getObject(const std::string& id) const;

    void removeObject(const std::string& id);

    const std::map<std::string, Object*>& getObjects() const;

    std::vector<Object*> getObjectsRecursive() const;

    //bool isOccupied(const tf::Vector3& pos) const;

    bool intersect(const geo::Ray& r, float t0, float t1, double& distance) const;

protected:

    static World* instance_;

    //std::vector<Object*> objects_;
    std::map<std::string, Object*> objects_;

    World();

};


#endif
