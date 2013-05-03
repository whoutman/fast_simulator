#ifndef _FAST_SIMULATOR_SHAPE_H_
#define _FAST_SIMULATOR_SHAPE_H_

#include "fast_simulator/Ray.h"

class Shape {

public:

    Shape();

    virtual Shape* clone() const = 0;

    virtual bool intersect(const Ray &, float t0, float t1, double& distance) const = 0;

    virtual void getBoundingBox(tf::Vector3 &min, tf::Vector3 &max) const = 0;

};


#endif
