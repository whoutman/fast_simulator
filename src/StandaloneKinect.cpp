#include "fast_simulator/StandaloneKinect.h"

#include <geolib/ros/tf_conversions.h>

using namespace std;

StandaloneKinect::StandaloneKinect(ros::NodeHandle& nh, const std::string& topic,
                                   const std::string& frame_id, const std::string& model_dir) : Robot(nh)
{
    tf_location_.frame_id_ = "/map";
    tf_location_.child_frame_id_ = frame_id;
    event_loc_pub_.scheduleRecurring(50);

    Kinect* kinect = new Kinect();

    kinect->setRGBDName(topic);
    kinect->setRGBFrame(frame_id);
    kinect->setDepthFrame(frame_id);

//    kinect->addModel("coke", model_dir + "/kinect/coke_cropped");

    this->registerSensor(kinect);
    this->addChild(kinect);
}

StandaloneKinect::~StandaloneKinect() {
}

void StandaloneKinect::step(double dt) {
    Robot::step(dt);

    if (event_loc_pub_.isScheduled()) {
        tf_location_.stamp_ = ros::Time::now();
        geo::convert(getAbsolutePose(), tf_location_);
        tf_broadcaster_.sendTransform(tf_location_);
    }
}
