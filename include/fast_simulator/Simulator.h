#ifndef FAST_SIMULATOR_H_
#define FAST_SIMULATOR_H_

#include "fast_simulator/Object.h"
#include "fast_simulator/Joint.h"
#include "fast_simulator/World.h"
#include "fast_simulator/Sprite.h"

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

    void addModel(const std::string& name, const Object& obj);

    const Object* getModel(const std::string& name) const;

protected:

    World& world_;

    //std::string MODEL_DIR;

    int UNIQUE_VIS_ID;

    std::map<std::string, int> object_id_to_vis_id;

    std::map<std::string, Object> MODELS;

};

#endif
