#include "fast_simulator/Kinect.h"
#include "fast_simulator/World.h"
#include "virtual_cam/Loader.h"

#include "fast_simulator/util.h"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

Kinect::Kinect(const string& rgb_topic, const string& depth_topic, const string& info_topic, const string& point_cloud_topic, const string& frame_id)
    : width_(640), height_(480), x_res_(2), y_res_(2) {
    image_loader_ = new ImageLoader(rgb_topic, depth_topic, info_topic, frame_id);

    // Set camera info

    cam_info_.header.frame_id = frame_id;
    cam_info_.height = height_;
    cam_info_.width = width_;
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

    // init rgb image

    image_rgb_.header.frame_id = frame_id;
    image_rgb_.encoding = "rgb8";
    image_rgb_.image = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(0,0,255));

    // init depth image

    image_depth_.header.frame_id = frame_id;
    image_depth_.encoding = "32FC1";
    image_depth_.image = cv::Mat(height_, width_, CV_32FC1, 0.0);

    // create publishers

    ros::NodeHandle nh;
    pub_rgb_ = nh.advertise<sensor_msgs::Image>(rgb_topic, 10);
    pub_depth_ = nh.advertise<sensor_msgs::Image>(depth_topic, 10);
    pub_cam_info_ = nh.advertise<sensor_msgs::CameraInfo>(info_topic, 10);
    pub_point_cloud_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >(point_cloud_topic, 1);

    // calculate ray directions

    double width = 2.172;
    double height = width * 0.75;

    double dx = width / width_;
    double dy = height / height_;

    ray_deltas_.resize(height_);
    double y = -height / 2;
    for(int iy = 0; iy < height_; ++iy) {
        double x = -width / 2;
        for(int ix = 0; ix < width_; ++ix) {
            ray_deltas_[iy].push_back(tf::Vector3(x, y, 1).normalize());
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

        tf::Transform tf_map_to_kinect = getAbsolutePose();
        tf::Vector3 kinect_origin = tf_map_to_kinect.getOrigin();

        double total_nr_intersects = 0;

        int x_res = 2;
        int y_res = 2;

        for(int y = 0; y < height_; y += y_res) {
            for(int x = 0; x < width_; x += x_res) {
                tf::Vector3 dir_transformed = tf::Transform(tf_map_to_kinect.getRotation()) * ray_deltas_[y][x];
                Ray r_transformed(kinect_origin, dir_transformed);

                double distance = 0;
                if (!world.intersect(r_transformed, 0, 5, distance)) {
                    distance = 0;
                }

                image_depth_.image.at<float>(y, x) = distance;

                //cout << "distance = " << distance << endl;

               // cout << "********" << endl;

                if (x > 0 && y > 0) {
                    float dist00 = image_depth_.image.at<float>(y - y_res , x - x_res);
                    float dist10 = image_depth_.image.at<float>(y - y_res , x);
                    float dist01 = image_depth_.image.at<float>(y, x - x_res);
                    float dist11 = image_depth_.image.at<float>(y, x);

                    // perform interpolation

                    for(int dy = 0; dy < y_res; ++dy) {
                        for(int dx = 0; dx < x_res; ++dx) {
                            if (dx > 0 || dy > 0) {

                                //cout << x - x_res + dx << ", " << y - y_res + dy << endl;
                                double wx = ((double)dx / x_res);
                                double wy = ((double)dy / y_res);

                                double dist = 0;
                                if ((dx == 0 || (abs(dist00 - dist10) < 0.1 && abs(dist01 - dist11) < 0.1))
                                        && (dy == 0 || (abs(dist00 - dist01) < 0.1 && abs(dist10 - dist11) < 0.1))) {
                                    dist = (1 - wy) * ((1 - wx) * dist00 + wx * dist10)
                                            +   wy  * ((1 - wx) * dist01 + wx * dist11);
                                }

                                image_depth_.image.at<float>(y - y_res + dy, x - x_res + dx) = dist;
                            }
                        }
                    }
                }

                total_nr_intersects += r_transformed.nr_intersection_calcs_;
            }
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud<pcl::PointXYZ>);
        msg->header.frame_id = image_rgb_.header.frame_id;
        msg->header.stamp = time;
        msg->width  = height_;
        msg->height = width_;

        for(int iy = 0; iy < height_; ++iy) {
            for(int ix = 0; ix < width_; ++ix) {
                tf::Vector3 intersect_pos_kinect = ray_deltas_[iy][ix] * image_depth_.image.at<float>(iy, ix);
                msg->points.push_back(pcl::PointXYZ(intersect_pos_kinect.x(), intersect_pos_kinect.y(), intersect_pos_kinect.z()));
            }
        }

        //cout << "Avg #intersections per ray = " << total_nr_intersects / i << endl;


        pub_cam_info_.publish(cam_info_);
        pub_rgb_.publish(image_rgb_.toImageMsg());
        pub_depth_.publish(image_depth_.toImageMsg());
        pub_point_cloud_.publish (msg);
    }

    // }

    //cout << "Kinect::publish() took " << ros::Time::now() - t_start << endl;
}
