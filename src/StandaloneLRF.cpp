#include "fast_simulator/StandaloneLRF.h"

using namespace std;

StandaloneLRF::StandaloneLRF(ros::NodeHandle& nh) : Robot(nh, "lrf") {
    LRF* lrf = new LRF("/external/laser", "/external/laser");
    this->registerSensor(lrf);
    this->addChild(lrf);
}

StandaloneLRF::~StandaloneLRF() {
}

void StandaloneLRF::step(double dt) {
    Robot::step(dt);
}
