#include "fast_simulator/Kinect.h"
#include "fast_simulator/World.h"
#include "virtual_cam/Loader.h"

#include "fast_simulator/util.h"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

Kinect::Kinect()
    : width_(640), height_(480), x_res_(2), y_res_(2), raytrace_(true) {

    nh_ = new ros::NodeHandle();

    // Set camera info

    cam_info_rgb_.height = height_;
    cam_info_rgb_.width = width_;
    cam_info_rgb_.distortion_model = "plumb_bob";

    // D: [0.0, 0.0, 0.0, 0.0, 0.0]
    cam_info_rgb_.D.push_back(0);
    cam_info_rgb_.D.push_back(0);
    cam_info_rgb_.D.push_back(0);
    cam_info_rgb_.D.push_back(0);
    cam_info_rgb_.D.push_back(0);

    // K: [554.2559327880068, 0.0, 320.5, 0.0, 554.2559327880068, 240.5, 0.0, 0.0, 1.0]
    cam_info_rgb_.K[0] = 554.2559327880068;
    cam_info_rgb_.K[1] = 0;
    cam_info_rgb_.K[2] = 320.5;
    cam_info_rgb_.K[3] = 0;
    cam_info_rgb_.K[4] = 554.2559327880068;
    cam_info_rgb_.K[5] = 240.5;
    cam_info_rgb_.K[6] = 0;
    cam_info_rgb_.K[7] = 0;
    cam_info_rgb_.K[8] = 1;

    // R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    cam_info_rgb_.R[0] = 1;
    cam_info_rgb_.R[1] = 0;
    cam_info_rgb_.R[2] = 0;
    cam_info_rgb_.R[3] = 0;
    cam_info_rgb_.R[4] = 1;
    cam_info_rgb_.R[5] = 0;
    cam_info_rgb_.R[6] = 0;
    cam_info_rgb_.R[7] = 0;
    cam_info_rgb_.R[8] = 1;

    // P: [554.2559327880068, 0.0, 320.5, -0.0, 0.0, 554.2559327880068, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    cam_info_rgb_.P[0] = 554.2559327880068;
    cam_info_rgb_.P[1] = 0;
    cam_info_rgb_.P[2] = 320.5;
    cam_info_rgb_.P[3] = 0;
    cam_info_rgb_.P[4] = 0;
    cam_info_rgb_.P[5] = 554.2559327880068;
    cam_info_rgb_.P[6] = 240.5;
    cam_info_rgb_.P[7] = 0;
    cam_info_rgb_.P[8] = 0;
    cam_info_rgb_.P[9] = 0;
    cam_info_rgb_.P[10] = 1;
    cam_info_rgb_.P[11] = 0;

    cam_info_rgb_.binning_x = 0;
    cam_info_rgb_.binning_y = 0;

    cam_info_rgb_.roi.x_offset = 0;
    cam_info_rgb_.roi.y_offset = 0;
    cam_info_rgb_.roi.height = 0;
    cam_info_rgb_.roi.width = 0;
    cam_info_rgb_.roi.do_rectify = false;


    cam_info_depth_ = cam_info_rgb_;

    // initialize pinhole model
    pinhole_model_.fromCameraInfo(cam_info_rgb_);

    // init rgb image
    image_rgb_.encoding = "bgr8";
    image_rgb_.image = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(255,255,255));

    // init depth image
    image_depth_.encoding = "32FC1";
    image_depth_.image = cv::Mat(height_, width_, CV_32FC1, 0.0);

    // calculate ray directions

    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(cam_info_rgb_);

    // calculate all all origin to pixel directions
    ray_deltas_.resize(cam_info_rgb_.width);
    depth_ratios_.resize(cam_info_rgb_.width);
    for(unsigned int x = 0; x < cam_info_rgb_.width; ++x) {
        ray_deltas_[x].resize(cam_info_rgb_.height);
        depth_ratios_[x].resize(cam_info_rgb_.height);
        for(unsigned int y = 0; y < cam_info_rgb_.height; ++y) {
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
    delete nh_;
}

void Kinect::addRGBTopic(const std::string& topic_name) {
    pubs_rgb_.push_back(nh_->advertise<sensor_msgs::Image>(topic_name, 10));
}

void Kinect::addDepthTopic(const std::string& topic_name) {
    pubs_depth_.push_back(nh_->advertise<sensor_msgs::Image>(topic_name, 10));
}

void Kinect::addPointCloudTopic(const std::string& topic_name) {
    pubs_point_cloud_.push_back(nh_->advertise<pcl::PointCloud<pcl::PointXYZ> >(topic_name, 1));
}

void Kinect::addRGBCameraInfoTopic(const std::string& topic_name) {
    pubs_cam_info_rgb_.push_back(nh_->advertise<sensor_msgs::CameraInfo>(topic_name, 10));
}

void Kinect::addDepthCameraInfoTopic(const std::string& topic_name) {
    pubs_cam_info_depth_.push_back(nh_->advertise<sensor_msgs::CameraInfo>(topic_name, 10));
}

void Kinect::setRGBFrame(const std::string& frame_id) {
    image_rgb_.header.frame_id = frame_id;
    cam_info_rgb_.header.frame_id = frame_id;
}

void Kinect::setDepthFrame(const std::string& frame_id) {
    image_depth_.header.frame_id = frame_id;
    cam_info_depth_.header.frame_id = frame_id;
}


void Kinect::addModel(const std::string& type, const std::string& filename) {
    Image image;
    try {
        image = Image::loadFromFile(filename);
    } catch (rosbag::BagException& e) {
        std::cout << "Could not load Kinect model '" << filename << "'" << std::endl;
        return;
    }
    type_to_image_[type] = image;
}

void Kinect::setRaytracing(bool status) {
    raytrace_ = status;
}

void Kinect::step(World& world) {

    ros::Time time = ros::Time::now();
    cam_info_rgb_.header.stamp = time;
    image_rgb_.header.stamp = time;
    image_depth_.header.stamp = time;

    if (raytrace_) {

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

        for(std::vector<ros::Publisher>::iterator it = pubs_point_cloud_.begin(); it != pubs_point_cloud_.end(); ++it) {
            it->publish(msg);
        }

    } else {
        image_depth_.image = cv::Mat(height_, width_, CV_32FC1, 0.0);
    }

    //cout << "Avg #intersections per ray = " << total_nr_intersects / i << endl;

    // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

    image_rgb_.image = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(255,255,255));

    vector<Object*> objects = world.getObjectsRecursive();

    string filename = "";
    for(vector<Object*>::const_iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj) {

        Object& obj = **it_obj;

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

                    int x_tl = pos2d.x - rgb_img.cols / 2;
                    int y_tl = pos2d.y - rgb_img.rows / 2;

                    for(int y = 0; y < rgb_img.rows; ++y) {
                        for(int x = 0; x < rgb_img.cols; ++x) {

                            int ix = x_tl + x;
                            int iy = y_tl + y;

                            if (ix >=0 && iy >= 0 && ix < width_ && iy < height_) {

                                unsigned char alpha = 255;//mask.at<unsigned char>(y, x);
                                if (alpha > 0) {
                                    image_depth_.image.at<float>(iy, ix) = depth_img.at<float>(y, x) - distance + z;

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

    for(std::vector<ros::Publisher>::iterator it = pubs_cam_info_rgb_.begin(); it != pubs_cam_info_rgb_.end(); ++it) {
        it->publish(cam_info_rgb_);
    }

    for(std::vector<ros::Publisher>::iterator it = pubs_cam_info_depth_.begin(); it != pubs_cam_info_depth_.end(); ++it) {
        it->publish(cam_info_depth_);
    }

    sensor_msgs::Image rgb_image_msg;
    image_rgb_.toImageMsg(rgb_image_msg);
    for(std::vector<ros::Publisher>::iterator it = pubs_rgb_.begin(); it != pubs_rgb_.end(); ++it) {
        it->publish(rgb_image_msg);
    }

    sensor_msgs::Image depth_image_msg;
    image_depth_.toImageMsg(depth_image_msg);
    for(std::vector<ros::Publisher>::iterator it = pubs_depth_.begin(); it != pubs_depth_.end(); ++it) {
        it->publish(depth_image_msg);
    }

    // }

    //cout << "Kinect::publish() took " << ros::Time::now() - t_start << endl;
}
