#include "fast_simulator/Simulator.h"

using namespace std;

Simulator::Simulator() : world_(World::getInstance()) {

}

Simulator::~Simulator()
{
    World::destroy();
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
