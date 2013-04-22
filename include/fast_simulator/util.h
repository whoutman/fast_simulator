#ifndef _FAST_SIMULATOR_UTIL_H_
#define _FAST_SIMULATOR_UTIL_H_

using namespace std;

#include <sstream>
string toString(const tf::Vector3& v) {
    stringstream s;
    s << "(" << v.getX() << ", " << v.getY() << ", " << v.getZ() << ")";
    return s.str();
}

string toString(const tf::Transform& tf) {
    stringstream s;
    s << toString(tf.getOrigin()) << ", "
      << "(" << tf.getRotation().getX() << ", " << tf.getRotation().getY() << ", " << tf.getRotation().getZ() << ", " << tf.getRotation().getW() << ")";
    return s.str();
}

#endif
