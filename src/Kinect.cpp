#include "fast_simulator/Kinect.h"
#include "fast_simulator/World.h"
#include "virtual_cam/Loader.h"

#include "fast_simulator/util.h"

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <geolib/Ray.h>
#include <geolib/Box.h>

using namespace std;

// ----------------------------------------------------------------------------------------------------

float randomUniform(float min, float max)
{
    return ((float)rand() / RAND_MAX) * (max - min) + min;
}

Kinect::Kinect()
    : width_(640), height_(480), x_res_(2), y_res_(2) {

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
    cam_info_rgb_.P[0] = 554.2559327880068; // 0, 0
    cam_info_rgb_.P[1] = 0;                 // 0, 1
    cam_info_rgb_.P[2] = 320.5;             // 0, 2
    cam_info_rgb_.P[3] = 0;                 // 0, 3
    cam_info_rgb_.P[4] = 0;                 // 1, 0
    cam_info_rgb_.P[5] = 554.2559327880068; // 1, 1
    cam_info_rgb_.P[6] = 240.5;             // 1, 2
    cam_info_rgb_.P[7] = 0;                 // 1, 3
    cam_info_rgb_.P[8] = 0;                 // 2, 0
    cam_info_rgb_.P[9] = 0;                 // 2, 1
    cam_info_rgb_.P[10] = 1;                // 2, 2
    cam_info_rgb_.P[11] = 0;                // 2, 3

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

    camera_.setFocalLengths(cam_model.fx(), cam_model.fy());
    camera_.setOpticalCenter(cam_model.cx(), cam_model.cy());
    camera_.setOpticalTranslation(cam_model.Tx(), cam_model.Ty());

    // calculate all origin to pixel directions
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

void Kinect::step(World& world) {
    ros::Time time = ros::Time::now();
    cam_info_rgb_.header.stamp = time;
    cam_info_depth_.header.stamp = time;
    image_rgb_.header.stamp = time;
    image_depth_.header.stamp = time;

    geo::Transform tf_map_to_kinect = getAbsolutePose() * geo::Pose3D(0, 0, 0, 3.1415, 0, 0);

    image_depth_.image = cv::Mat(height_, width_, CV_32FC1, 0.0);

    const std::map<std::string, Object*>& objects = world.getObjects();
    for(std::map<std::string, Object*>::const_iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj) {
        Object* obj = it_obj->second;

        geo::ShapePtr shape = obj->getShape();
        if (shape) {
            camera_.rasterize(*shape, obj->getAbsolutePose(), image_depth_.image);
        }

        std::vector<Object*> children;
        obj->getChildrenRecursive(children);

        for(std::vector<Object*>::const_iterator it = children.begin(); it != children.end(); ++it) {
            const Object& child = **it;
            geo::ShapePtr child_shape = child.getShape();
            if (child_shape) {
                geo::Transform t = tf_map_to_kinect.inverse() * child.getAbsolutePose();
                camera_.rasterize(*child_shape, t, image_depth_.image);
            }
        }
    }

    // Add noise to depth image
    for(int y = 0; y < image_depth_.image.rows; ++y)
    {
        for(int x = 0; x < image_depth_.image.cols; ++x)
        {
            float& d = image_depth_.image.at<float>(y, x);
            if (d > 0)
                d += randomUniform(d * -0.005, d * 0.005);
        }
    }


    // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

    image_rgb_.image = cv::Mat(height_, width_, CV_8UC3, cv::Scalar(255,255,255));

    vector<Object*> objects_rec = world.getObjectsRecursive();
    for(vector<Object*>::const_iterator it_obj = objects_rec.begin(); it_obj != objects_rec.end(); ++it_obj) {
        Object& obj = **it_obj;

        map<string, Image>::iterator it_image = type_to_image_.find(obj.getType());
        if (it_image == type_to_image_.end()) {
            it_image = type_to_image_.find(obj.getID());
        }

        if (it_image != type_to_image_.end()) {
            geo::Transform tf_kinect_to_object = getAbsolutePose().inverseTimes(obj.getAbsolutePose());

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
//                                    image_depth_.image.at<float>(iy, ix) = depth_img.at<float>(y, x) - distance + z;
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

    for(int iy = 0; iy < height_; ++iy) {
        for(int ix = 0; ix < width_; ++ix) {
            if (image_depth_.image.at<float>(iy, ix) > 8) {
                image_depth_.image.at<float>(iy, ix) = 0;
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr msg(new pcl::PointCloud<pcl::PointXYZ>);
    msg->header.frame_id = image_rgb_.header.frame_id;
    msg->header.stamp = time;
    msg->width  = height_;
    msg->height = width_;
    msg->is_dense = true;

    int k = 0;
    msg->points.resize(width_ * height_);
    for(int iy = 0; iy < height_; ++iy) {
        for(int ix = 0; ix < width_; ++ix) {
            double distance = image_depth_.image.at<float>(iy, ix) * depth_ratios_[ix][iy];
            //double distance = 5 * depth_ratios_[ix][iy];
            if (distance == 0) {
                distance = 0.0 / 0.0; // create NaN
                msg->is_dense = false;
            }

            tf::Vector3 intersect_pos_kinect = ray_deltas_[ix][iy] * distance;
            msg->points[k] = pcl::PointXYZ(intersect_pos_kinect.x(), intersect_pos_kinect.y(), intersect_pos_kinect.z());
            ++k;
        }
    }

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

    for(std::vector<ros::Publisher>::iterator it = pubs_point_cloud_.begin(); it != pubs_point_cloud_.end(); ++it) {
        it->publish(msg);
    }
}

