#include "fast_simulator/Kinect.h"
#include "fast_simulator/World.h"
#include "virtual_cam/Loader.h"

#include "fast_simulator/util.h"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

Kinect::Kinect(const string& rgb_topic, const string& depth_topic, const string& info_topic, const string& point_cloud_topic, const string& frame_id) {
    image_loader_ = new ImageLoader(rgb_topic, depth_topic, info_topic, frame_id);

    cam_info_.header.frame_id = frame_id;
    cam_info_.height = 480;
    cam_info_.width = 640;
    cam_info_.distortion_model = "plumb_bob";

    // D: [0.0, 0.0, 0.0, 0.0, 0.0]
    cam_info_.D.push_back(0);
    cam_info_.D.push_back(0);
    cam_info_.D.push_back(0);
    cam_info_.D.push_back(0);
    cam_info_.D.push_back(0);

    // K: [554.2559327880068, 0.0, 320.5, 0.0, 554.2559327880068, 240.5, 0.0, 0.0, 1.0]
    cam_info_.K[0] = 554.2559327880068;
    cam_info_.K[1] = 0;
    cam_info_.K[2] = 320.5;
    cam_info_.K[3] = 0;
    cam_info_.K[4] = 554.2559327880068;
    cam_info_.K[5] = 240.5;
    cam_info_.K[6] = 0;
    cam_info_.K[7] = 0;
    cam_info_.K[8] = 1;

    // R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    cam_info_.R[0] = 1;
    cam_info_.R[1] = 0;
    cam_info_.R[2] = 0;
    cam_info_.R[3] = 0;
    cam_info_.R[4] = 1;
    cam_info_.R[5] = 0;
    cam_info_.R[6] = 0;
    cam_info_.R[7] = 0;
    cam_info_.R[8] = 1;

    // P: [554.2559327880068, 0.0, 320.5, -0.0, 0.0, 554.2559327880068, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    cam_info_.P[0] = 554.2559327880068;
    cam_info_.P[1] = 0;
    cam_info_.P[2] = 320.5;
    cam_info_.P[3] = 0;
    cam_info_.P[4] = 0;
    cam_info_.P[5] = 554.2559327880068;
    cam_info_.P[6] = 240.5;
    cam_info_.P[7] = 0;
    cam_info_.P[8] = 0;
    cam_info_.P[9] = 0;
    cam_info_.P[10] = 1;
    cam_info_.P[11] = 0;

    cam_info_.binning_x = 0;
    cam_info_.binning_y = 0;

    cam_info_.roi.x_offset = 0;
    cam_info_.roi.y_offset = 0;
    cam_info_.roi.height = 0;
    cam_info_.roi.width = 0;
    cam_info_.roi.do_rectify = false;

    image_rgb_.header.frame_id = frame_id;
    image_rgb_.encoding = "rgb8";
    image_rgb_.image = cv::Mat(480, 64, CV_8UC3, cv::Scalar(0,0,255));

    image_depth_.header.frame_id = frame_id;
    image_depth_.encoding = "32FC1";
    image_depth_.image = cv::Mat(480, 640, CV_32FC1, 3);

/*
    for (int i = 0; i < 640 * 480 * 4; ++i) {
        image_depth_.data.push_back(0);
    }
    */

    ros::NodeHandle nh;
    pub_rgb_ = nh.advertise<sensor_msgs::Image>(rgb_topic, 10);
    pub_depth_ = nh.advertise<sensor_msgs::Image>(depth_topic, 10);
    pub_cam_info_ = nh.advertise<sensor_msgs::CameraInfo>(info_topic, 10);
    pub_point_cloud_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >(point_cloud_topic, 1);

    double width = 3.2;
    double height = 2.4;

    //grid_width_ = 128;
    //grid_height_ = 96;
    grid_width_ = 320;
    grid_height_ = 240;


    double x = -width / 2;
    double y = -height / 2;

    double dx = width / grid_width_;
    double dy = height / grid_height_;

    for(int iy = 0; iy < grid_width_; ++iy) {
        x = -width / 2;
        for(int ix = 0; ix < grid_height_; ++ix) {
            // x, 1, y      X
            // 1, x, y      X
            // x, y, 1      OK

            ray_deltas_.push_back(tf::Vector3(x, y, 1).normalize());
            //std::cout << x << ", " << y << std::endl;
            x += dx;
        }
        y += dy;
    }


}

Kinect::~Kinect() {
    delete image_loader_;
}

void Kinect::addModel(const std::string& type, const std::string& filename) {
    type_to_filename_[type] = filename;
}

void Kinect::step(World& world) {

    ros::Time t_start = ros::Time::now();

    //if (pub_cam_info_.getNumSubscribers() > 0 || pub_rgb_.getNumSubscribers() > 0 || pub_depth_.getNumSubscribers() > 0) {

    map<string, Object*> objects = world.getObjects();

        string filename = "";
        for(map<string, Object*>::const_iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj) {
            Object& obj = *it_obj->second;

            map<string, string>::const_iterator it_filename = type_to_filename_.find(obj.getType());
            if (it_filename == type_to_filename_.end()) {
                it_filename = type_to_filename_.find(obj.getID());
            }

            if (it_filename != type_to_filename_.end()) {
                tf::Transform tf_kinect_to_object = getAbsolutePose().inverseTimes(obj.getAbsolutePose());

                double x = tf_kinect_to_object.getOrigin().getX();
                double y = tf_kinect_to_object.getOrigin().getY();

                if (x > 0 && x < 2 && abs(y) < x / 2) {
                    filename = it_filename->second;
                }
            }
        }

        if (filename != "") {
            if (filename != loaded_file_) {
                image_loader_->load(filename);
                loaded_file_ = filename;
            }
            image_loader_->publish();
        } else {



            ros::Time time = ros::Time::now();
            cam_info_.header.stamp = time;
            image_rgb_.header.stamp = time;
            image_depth_.header.stamp = time;

            pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud<pcl::PointXYZ>);
            msg->header.frame_id = image_rgb_.header.frame_id;
            msg->header.stamp = time;
            msg->width  = grid_width_;
            msg->height = grid_height_;


            tf::Transform tf_map_to_kinect = getAbsolutePose();
            tf::Vector3 kinect_origin = tf_map_to_kinect.getOrigin();

            double step_x = (double)640 / grid_width_;
            double step_y = (double)480 / grid_height_;

            int i = 0;
            for(int iy = 0; iy < grid_width_; ++iy) {
                for(int ix = 0; ix < grid_height_; ++ix) {
                    //image_depth_.image.at<float>(y, x) = (double)x / 640;

                    tf::Vector3 dir = tf::Transform(tf_map_to_kinect.getRotation()) * ray_deltas_[i];
                    Ray r(kinect_origin, dir);

                    double distance = 0;
                    if (world.intersect(r, 0, 5, distance)) {
                        tf::Vector3 intersect_pos_kinect = ray_deltas_[i] * distance;
                        msg->points.push_back(pcl::PointXYZ(intersect_pos_kinect.x(), intersect_pos_kinect.y(), intersect_pos_kinect.z()));
                    } else {
                        msg->points.push_back(pcl::PointXYZ(0, 0, 0));
                    }

                    i++;

                    for(int y = iy * step_y; y < min(480.0, (iy + 1) * step_y); ++y) {
                       for(int x = ix * step_x; x < min(640.0, (ix + 1) * step_x); ++x) {
                            image_depth_.image.at<float>(y, x) = distance;
                        }
                    }
                }
            }


            pub_cam_info_.publish(cam_info_);
            pub_rgb_.publish(image_rgb_.toImageMsg());
            pub_depth_.publish(image_depth_.toImageMsg());
            pub_point_cloud_.publish (msg);
        }

   // }

        //cout << "Kinect::publish() took " << ros::Time::now() - t_start << endl;
}
