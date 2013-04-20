#ifndef _FAST_SIMULATOR_BOX_H_
#define _FAST_SIMULATOR_BOX_H_

#include "fast_simulator/Ray.h"

class Box {

public:

    Box(const tf::Vector3 &min, const tf::Vector3 &max);

    bool intersect(const Ray &, float t0, float t1, double& distance) const;

    tf::Vector3 bounds[2];

};


#endif
