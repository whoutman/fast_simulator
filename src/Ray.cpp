#include "fast_simulator/Ray.h"

Ray::Ray(tf::Vector3 &o, tf::Vector3 &d) {
    origin = o;
    direction = d; // ASSUME d is normalized!
    inv_direction = tf::Vector3(1/direction.x(), 1/direction.y(), 1/direction.z());
    sign[0] = (inv_direction.x() < 0);
    sign[1] = (inv_direction.y() < 0);
    sign[2] = (inv_direction.z() < 0);
}

