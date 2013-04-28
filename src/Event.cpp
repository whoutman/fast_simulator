#include "fast_simulator/Event.h"

Event::Event() : time_(0) {
}

void Event::schedule(const ros::Time& time) {
    is_recurring_ = false;
    time_ = time;
}

void Event::scheduleRecurring(double rate) {
    time_ = ros::Time::now() + ros::Duration(1 / rate);
    rate_ = rate;
    is_recurring_ = true;
}

bool Event::isScheduled() {
    if (time_ == ros::Time(0)) {
        return false;
    }

    ros::Time time_now = ros::Time::now();

    if (time_now < time_) {
        return false;
    }

    if (is_recurring_) {
        // TODO: make more efficient
        while(time_ <= time_now) {
            time_ += ros::Duration(1.0 / rate_);
        }
    }

    return true;
}

