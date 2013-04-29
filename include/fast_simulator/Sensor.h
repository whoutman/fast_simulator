#ifndef _FAST_SIMULATOR_SENSOR_H_
#define _FAST_SIMULATOR_SENSOR_H_

#include "fast_simulator/Object.h"

#include <boost/thread.hpp>


class Sensor : public Object {

public:

    Sensor();

    virtual ~Sensor();

    virtual void step(World& world) = 0;

    void start();

    void run();

protected:

    boost::thread worker_thread_;

};


#endif
