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

    // initialize pinhole model
    pinhole_model_.fromCameraInfo(cam_info_);

    // init rgb image

    image_rgb_.header.frame_id = frame_id;
    image_rgb_.encoding = "bgr8";
    image_rgb_.image = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(255,255,255));

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

    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(cam_info_);

    // calculate all all origin to pixel directions
    ray_deltas_.resize(cam_info_.width);
    depth_ratios_.resize(cam_info_.width);
    for(unsigned int x = 0; x < cam_info_.width; ++x) {
        ray_deltas_[x].resize(cam_info_.height);
        depth_ratios_[x].resize(cam_info_.height);
        for(unsigned int y = 0; y < cam_info_.height; ++y) {
            // compute direction vector to pixel (x, y)
            cv::Point3d dir_cv = cam_model.projectPixelTo3dRay(cv::Point2d(x, y));

            // convert to tf vector
            tf::Vector3 dir(dir_cv.x, dir_cv.y, dir_cv.z);

            // store the depth ratio for this pixel
            depth_ratios_[x][y] = dir.length();

            // normalize and store direction vector
            ray_deltas_[x][y] = dir.normalize();
        }
    }

}

Kinect::~Kinect() {
}

void Kinect::addModel(const std::string& type, const std::string& filename) {
    type_to_image_[type] = Image::loadFromFile(filename);
}

void Kinect::step(World& world) {

    ros::Time t_start = ros::Time::now();

    //if (pub_cam_info_.getNumSubscribers() > 0 || pub_rgb_.getNumSubscribers() > 0 || pub_depth_.getNumSubscribers() > 0) {

    /*
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
    */

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
            tf::Vector3 dir_transformed = tf::Transform(tf_map_to_kinect.getRotation()) * ray_deltas_[x][y];
            Ray r_transformed(kinect_origin, dir_transformed);

            double distance = 0;
            if (!world.intersect(r_transformed, 0, 5, distance)) {
                distance =  0.0 / 0.0; // create NaN
            }

            image_depth_.image.at<float>(y, x) = distance / depth_ratios_[x][y];

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
    msg->is_dense = true;

    for(int iy = 0; iy < height_; ++iy) {
        for(int ix = 0; ix < width_; ++ix) {
            double distance = image_depth_.image.at<float>(iy, ix) * depth_ratios_[ix][iy];
            if (distance == 0) {
                distance = 0.0 / 0.0; // create NaN
                msg->is_dense = false;
            }

            tf::Vector3 intersect_pos_kinect = ray_deltas_[ix][iy] * distance;
            msg->points.push_back(pcl::PointXYZ(intersect_pos_kinect.x(), intersect_pos_kinect.y(), intersect_pos_kinect.z()));
        }
    }

    //cout << "Avg #intersections per ray = " << total_nr_intersects / i << endl;

    // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

    image_rgb_.image = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(255,255,255));

    map<string, Object*> objects = world.getObjects();

    string filename = "";
    for(map<string, Object*>::const_iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj) {
        Object& obj = *it_obj->second;

        map<string, Image>::iterator it_image = type_to_image_.find(obj.getType());
        if (it_image == type_to_image_.end()) {
            it_image = type_to_image_.find(obj.getID());
        }

        if (it_image != type_to_image_.end()) {
            tf::Transform tf_kinect_to_object = getAbsolutePose().inverseTimes(obj.getAbsolutePose());

            double x = tf_kinect_to_object.getOrigin().getX();
            double y = tf_kinect_to_object.getOrigin().getY();
            double z = tf_kinect_to_object.getOrigin().getZ();

            if (z > 0 && z < 3) {
                cv::Point2d pos2d = pinhole_model_.project3dToPixel(cv::Point3d(x, y, z));

                if (pos2d.x > 0 && pos2d.x < width_ && pos2d.y > 0 && pos2d.y < height_) {

                    Image& image = it_image->second;
                    cv::Mat mask = image.getMaskImage();
                    cv::Mat rgb_img = image.getRGBImage();
                    cv::Mat depth_img = image.getDepthImage();

                    // determine distance to object in prototype image
                    float distance = 1000;
                    for(int y = 0; y < depth_img.rows; ++y) {
                        for(int x = 0; x < depth_img.cols; ++x) {
                            if (depth_img.at<float>(y, x) > 0 && depth_img.at<float>(y, x) < distance) {
                                distance = depth_img.at<float>(y, x);
                            }
                        }
                    }

                    //cout << "Z = " << z << endl;

                    int x_tl = pos2d.x - mask.cols / 2;
                    int y_tl = pos2d.y - mask.rows / 2;

                    for(int y = 0; y < mask.rows; ++y) {
                        for(int x = 0; x < mask.cols; ++x) {

                            int ix = x_tl + x;
                            int iy = y_tl + y;

                            if (ix >=0 && iy >= 0 && ix < width_ && iy < height_) {

                                unsigned char alpha = mask.at<unsigned char>(y, x);
                                if (alpha > 0) {
                                    image_depth_.image.at<float>(iy, ix) = depth_img.at<float>(y, x) - distance + z;
                                    //cout << "   " << image_depth_.image.at<float>(iy, ix);

                                    cv::Vec3b image_clr =  image_rgb_.image.at<cv::Vec3b>(iy, ix);
                                    cv::Vec3b object_clr = rgb_img.at<cv::Vec3b>(y, x);

                                    cv::Vec3b color;
                                    color[0] = (image_clr[0] * (255 - alpha) + object_clr[0] * alpha) / 255;
                                    color[1] = (image_clr[1] * (255 - alpha) + object_clr[1] * alpha) / 255;
                                    color[2] = (image_clr[2] * (255 - alpha) + object_clr[2] * alpha) / 255;

                                    image_rgb_.image.at<cv::Vec3b>(iy, ix) = color;
                                }
                            }
                        }
                    }

                    //cout << endl;

                } // check if in view

            } // check if close enough

        } // check if known object type

    } // loop over objects



    // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *



    pub_cam_info_.publish(cam_info_);
    pub_rgb_.publish(image_rgb_.toImageMsg());
    pub_depth_.publish(image_depth_.toImageMsg());
    pub_point_cloud_.publish (msg);


    // }

    //cout << "Kinect::publish() took " << ros::Time::now() - t_start << endl;
}
