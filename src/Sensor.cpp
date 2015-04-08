#include "fast_simulator/Sensor.h"
#include "fast_simulator/World.h"

#include <profiling/Timer.h>

Sensor::Sensor() : rate_(10) {
    type_ = "sensor";
}

Sensor::~Sensor()
{
    worker_thread_.join();
}

void Sensor::start() {
    worker_thread_ = boost::thread(&Sensor::run, this);
}

void Sensor::run() {
    ros::Rate r(rate_);
    while(ros::ok()) {
        // create a copy of the world
        World world(World::getInstance());

//        Timer timer;
//        timer.start();

        // calculate sensor data
        step(world);

//        std::cout << "Sensor took: " << timer.getElapsedTimeInMilliSec() << "ms" << std::endl;

        r.sleep();
    }
}
