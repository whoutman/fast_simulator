#ifndef _FAST_SIMULATOR_KINECT_H_
#define _FAST_SIMULATOR_KINECT_H_

#include "Sensor.h"

#include <ros/ros.h>

//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

#include <rgbd/Server.h>

//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/CameraInfo.h>

//#include <image_geometry/pinhole_camera_model.h>

#include <virtual_cam/Image.h>

#include <geolib/sensors/DepthCamera.h>

class Kinect : public Sensor {

public:

    Kinect();

    virtual ~Kinect();

    void setRGBDName(const std::string& name);

    void setRGBFrame(const std::string& frame_id);

    void setDepthFrame(const std::string& frame_id);

    void addModel(const std::string& type, const std::string& filename);

    void step(World& world);

protected:

    std::map<std::string, Image> type_to_image_;

    rgbd::Server rgbd_server_;

    geo::DepthCamera cam_model_;

    std::string rgb_frame_id_, depth_frame_id_;

    cv::Mat image_rgb_, image_depth_;


};

#endif
