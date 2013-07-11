#include "fast_simulator/Simulator.h"

using namespace std;

Simulator::Simulator() : world_(World::getInstance()), UNIQUE_VIS_ID(0) {

}

Simulator::~Simulator() {

}

void Simulator::step(double dt) {
    world_.step(dt);
}

void Simulator::addObject(const std::string& id, Object* obj) {
    world_.addObject(id, obj);
}

Object* Simulator::getObject(const std::string& id) const {
    return world_.getObject(id);
}

map<string, Object*> Simulator::getObjects() const {
    return world_.getObjects();
}

void Simulator::removeObject(const std::string& id) {
    world_.removeObject(id);
}

void Simulator::addModel(const std::string& name, const Object& obj) {
    MODELS[name] = obj;
}

const Object* Simulator::getModel(const std::string& name) const {
    map<string, Object>::const_iterator it = MODELS.find(name);
    if (it == MODELS.end()) {
        return 0;
    }
    return &it->second;
}
