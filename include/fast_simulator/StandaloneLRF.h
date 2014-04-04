#ifndef FAST_SIMULATOR_STANDALONELRF_H_
#define FAST_SIMULATOR_STANDALONELRF_H_

#include "fast_simulator/Robot.h"

class StandaloneLRF : public Robot {

public:

    StandaloneLRF(ros::NodeHandle& nh);

    virtual ~StandaloneLRF();

    void step(double dt);

};

#endif
