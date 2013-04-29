#include "fast_simulator/Sensor.h"
#include "fast_simulator/World.h"

Sensor::Sensor() {
}

Sensor::~Sensor() {}

void Sensor::start() {
    worker_thread_ = boost::thread(&Sensor::run, this);
}

void Sensor::run() {
    ros::Rate r(10);
    while(ros::ok()) {
        World world(*getWorldHandle());
        step(world);
        r.sleep();
    }
}
