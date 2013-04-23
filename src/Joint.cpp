#include "fast_simulator/Joint.h"

#include <iostream>

using namespace std;

Joint::Joint(double position, double max_velocity) : position_(position), reference_(position), max_velocity_(max_velocity) {
}

Joint::~Joint() {
}

void Joint::step(double dt) {

    double diff = reference_ - position_;

    double abs_diff = diff;
    if (diff < 0) {
        abs_diff = -diff;
    }

    if (abs_diff < max_velocity_ * dt * 1.1) {
        position_ = reference_;
    } else {
        if (diff > 0) {
            position_ += max_velocity_ * dt;
        } else {
            position_ -= max_velocity_ * dt;
        }

    }
}
