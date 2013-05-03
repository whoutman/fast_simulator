#ifndef _FAST_SIMULATOR_BOX_H_
#define _FAST_SIMULATOR_BOX_H_

#include "Shape.h"

class Box : public Shape {

public:

    Box(const tf::Vector3 &min, const tf::Vector3 &max);

    Box* clone() const;

    bool intersect(const Ray &, float t0, float t1, double& distance) const;

    void getBoundingBox(tf::Vector3 &min, tf::Vector3 &max) const;

    tf::Vector3 bounds[2];

};


#endif
