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

#include <image_geometry/pinhole_camera_model.h>

#include <virtual_cam/Image.h>

#include <geolib/sensors/DepthCamera.h>

class Kinect : public Sensor {

public:

    Kinect();

    virtual ~Kinect();

    void addRGBTopic(const std::string& topic_name);

    void addDepthTopic(const std::string& topic_name);

    void addPointCloudTopic(const std::string& topic_name);

    void addRGBCameraInfoTopic(const std::string& topic_name);

    void addDepthCameraInfoTopic(const std::string& topic_name);

    void setRGBFrame(const std::string& frame_id);

    void setDepthFrame(const std::string& frame_id);

    void addModel(const std::string& type, const std::string& filename);

    void step(World& world);

protected:

    ros::NodeHandle* nh_;

    geo::DepthCamera camera_;

    int width_;
    int height_;

    int x_res_;
    int y_res_;

    std::map<std::string, Image> type_to_image_;

    std::vector<ros::Publisher> pubs_rgb_;
    std::vector<ros::Publisher> pubs_depth_;
    std::vector<ros::Publisher> pubs_cam_info_rgb_;
    std::vector<ros::Publisher> pubs_cam_info_depth_;
    std::vector<ros::Publisher> pubs_point_cloud_;

    sensor_msgs::CameraInfo cam_info_rgb_;
    sensor_msgs::CameraInfo cam_info_depth_;
    cv_bridge::CvImage image_rgb_;
    cv_bridge::CvImage image_depth_;

    image_geometry::PinholeCameraModel pinhole_model_;

    std::vector<std::vector<tf::Vector3> > ray_deltas_;

    std::vector<std::vector<double> > depth_ratios_;


};

#endif
