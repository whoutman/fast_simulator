#ifndef _FAST_SIMULATOR_KINECT_H_
#define _FAST_SIMULATOR_KINECT_H_

#include "Sensor.h"

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

class ImageLoader;

class Kinect : public Sensor {

public:

    Kinect(const std::string& rgb_topic, const std::string& depth_topic, const std::string& info_topic, const std::string& frame_id);

    virtual ~Kinect();

    void addModel(const std::string& type, const std::string& filename);

    void publish();

protected:

    ImageLoader* image_loader_;

    std::string loaded_file_;

    std::map<std::string, std::string> type_to_filename_;

    ros::Publisher pub_rgb_;
    ros::Publisher pub_depth_;
    ros::Publisher pub_cam_info_;
    ros::Publisher pub_point_cloud_;

    sensor_msgs::CameraInfo cam_info_;
    cv_bridge::CvImage image_rgb_;
    cv_bridge::CvImage image_depth_;

    int grid_width_;
    int grid_height_;


    std::vector<tf::Vector3> ray_deltas_;


};

#endif
