#ifndef _FAST_SIMULATOR_JOINT_H_
#define _FAST_SIMULATOR_JOINT_H_

#include <string>

#include <kdl/segment.hpp>

class Object;

class Joint {

public:

    Joint(double position, double max_velocity = 0.6);

    virtual ~Joint();

    void step(double dt);

    std::string name_;

    double position_;

    double reference_;

    double max_velocity_;

    Object* link_;

    // easiest for now. TODO: get rid of this (maybe KDL in general
    // by creating new class Pose which can be configured with a joint
    // position
    KDL::Segment kdl_segment_;

};


#endif
