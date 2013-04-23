#ifndef _FAST_SIMULATOR_JOINT_H_
#define _FAST_SIMULATOR_JOINT_H_

#include <string>

class Joint {

public:

    Joint(double position, double max_velocity = 0.6);

    virtual ~Joint();

    void step(double dt);

    std::string name_;

    double position_;

    double reference_;

    double max_velocity_;

};


#endif
