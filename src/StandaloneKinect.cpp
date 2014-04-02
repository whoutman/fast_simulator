#include "fast_simulator/StandaloneKinect.h"

using namespace std;

StandaloneKinect::StandaloneKinect(ros::NodeHandle& nh, const std::string& model_dir) : Robot(nh, "kinect", false) {
    tf_location_.frame_id_ = "/map";
    tf_location_.child_frame_id_ = "/kinect/openni_rgb_optical_frame";
    event_loc_pub_.scheduleRecurring(50);

    Kinect* kinect = new Kinect();

    kinect->addRGBTopic("/kinect/rgb/image_rect_colorr");
    kinect->addDepthTopic("/kinect/depth_registered/image");
    kinect->addRGBCameraInfoTopic("/kinect/rgb/camera_info");
    kinect->addDepthCameraInfoTopic("/kinect/depth_registered/camera_info");
    kinect->addPointCloudTopic("/kinect/rgb/points");
    kinect->setRGBFrame("/kinect/openni_rgb_optical_frame");
    kinect->setDepthFrame("/kinect/openni_rgb_optical_frame");


    kinect->addModel("coke", model_dir + "/kinect/coke_cropped");

    this->registerSensor(kinect);
    this->addChild(kinect);
}

StandaloneKinect::~StandaloneKinect() {
}

void StandaloneKinect::step(double dt) {
    Robot::step(dt);

    if (event_loc_pub_.isScheduled()) {
        tf_location_.stamp_ = ros::Time::now();
        tf_location_.setOrigin(getAbsolutePose().getOrigin());
        tf_location_.setRotation(getAbsolutePose().getRotation());
        tf_broadcaster_.sendTransform(tf_location_);
    }
}
