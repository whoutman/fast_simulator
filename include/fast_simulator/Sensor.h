#ifndef _FAST_SIMULATOR_SENSOR_H_
#define _FAST_SIMULATOR_SENSOR_H_

#include "fast_simulator/Object.h"

class Sensor : public Object {

public:

    Sensor() {}

    virtual ~Sensor() {}

    virtual void publish() = 0;

};


#endif
