#ifndef FAST_SIMULATOR_STANDALONEKINECT_H_
#define FAST_SIMULATOR_STANDALONEKINECT_H_

#include "fast_simulator/Robot.h"

class StandaloneKinect : public Robot {

public:

    StandaloneKinect(ros::NodeHandle& nh, const std::string& topic,
                     const std::string& frame_id, const std::string& model_dir);

    virtual ~StandaloneKinect();

    void step(double dt);

protected:

    Event event_loc_pub_;

    tf::StampedTransform tf_location_;

};

#endif
