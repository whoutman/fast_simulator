#include "fast_simulator/Box.h"

Box::Box(const tf::Vector3 &min, const tf::Vector3 &max) {
    bounds[0] = min;
    bounds[1] = max;
}

Box* Box::clone() const {
    return new Box(*this);
}

bool Box::intersect(const Ray &r, float t0, float t1, double& distance) const {

    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    tmin = (bounds[r.sign[0]].x() - r.origin.x()) * r.inv_direction.x();
    tmax = (bounds[1-r.sign[0]].x() - r.origin.x()) * r.inv_direction.x();
    tymin = (bounds[r.sign[1]].y() - r.origin.y()) * r.inv_direction.y();
    tymax = (bounds[1-r.sign[1]].y() - r.origin.y()) * r.inv_direction.y();

    if ( (tmin > tymax) || (tymin > tmax) )
        return false;
    if (tymin > tmin)
        tmin = tymin;
    if (tymax < tmax)
        tmax = tymax;
    tzmin = (bounds[r.sign[2]].z() - r.origin.z()) * r.inv_direction.z();
    tzmax = (bounds[1-r.sign[2]].z() - r.origin.z()) * r.inv_direction.z();
    if ( (tmin > tzmax) || (tzmin > tmax) )
        return false;
    if (tzmin > tmin)
        tmin = tzmin;
    if (tzmax < tmax)
        tmax = tzmax;

    distance = tmin;
    return t0 < tmax && tmin < t1;
}

