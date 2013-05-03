#ifndef _FAST_SIMULATOR_RAY_H_
#define _FAST_SIMULATOR_RAY_H_

#include <tf/tf.h>

class Ray {

public:

    Ray(const tf::Vector3 &o, const tf::Vector3 &d) ;

    tf::Vector3 origin;

    tf::Vector3 direction;

    tf::Vector3 inv_direction;

    int sign[3];


    mutable int nr_intersection_calcs_;
};

#endif
