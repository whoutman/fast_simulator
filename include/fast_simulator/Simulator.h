#ifndef FAST_SIMULATOR_H_
#define FAST_SIMULATOR_H_

#include "fast_simulator/Object.h"
#include "fast_simulator/Joint.h"
#include "fast_simulator/World.h"
//#include "fast_simulator/Sprite.h"

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>
#include "fast_simulator/SetObject.h"

class Simulator {

public:

    Simulator();

    virtual ~Simulator();

    void step(double dt);

    void addObject(const std::string& id, Object* obj);

    Object* getObject(const std::string& id) const;

    std::map<std::string, Object*> getObjects() const;

    void removeObject(const std::string& id);

protected:

    World& world_;

};

#endif
