#include "fast_simulator/Ray.h"

Ray::Ray(const tf::Vector3 &o, const tf::Vector3 &d) : origin(o), direction(d), nr_intersection_calcs_(0) { // ASSUME d is normalized!
    inv_direction = tf::Vector3(1/direction.x(), 1/direction.y(), 1/direction.z());
    sign[0] = (inv_direction.x() < 0);
    sign[1] = (inv_direction.y() < 0);
    sign[2] = (inv_direction.z() < 0);
}

