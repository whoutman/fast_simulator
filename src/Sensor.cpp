#include "fast_simulator/Sensor.h"
#include "fast_simulator/World.h"

Sensor::Sensor() {
    description_->type_ = "sensor";
}

Sensor::~Sensor() {}

void Sensor::start() {
    worker_thread_ = boost::thread(&Sensor::run, this);
}

void Sensor::run() {
    ros::Rate r(10);
    while(ros::ok()) {

        // create a copy of the world
        World world(World::getInstance());

        // calculate sensor data
        step(world);

        r.sleep();
    }
}
