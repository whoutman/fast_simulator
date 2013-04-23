#ifndef _FAST_SIMULATOR_UTIL_H_
#define _FAST_SIMULATOR_UTIL_H_

#include <tf/tf.h>
#include <string>

std::string toString(const tf::Vector3& v);

std::string toString(const tf::Transform& tf);

#endif
