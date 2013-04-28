#ifndef _FAST_SIMULATOR_EVENT_H_
#define _FAST_SIMULATOR_EVENT_H_

#include "ros/ros.h"

class Event {

public:

    Event();

    void schedule(const ros::Time& time);

    void scheduleRecurring(double rate);

    bool isScheduled();

protected:

    ros::Time time_;

    double rate_;

    bool is_recurring_;

};

#endif
