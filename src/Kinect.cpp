#include "fast_simulator/Kinect.h"
#include "fast_simulator/World.h"
#include "virtual_cam/Loader.h"

#include "fast_simulator/util.h"

//#include <opencv2/highgui/highgui.hpp>

#include <rgbd/Image.h>

using namespace std;

// ----------------------------------------------------------------------------------------------------

float randomUniform(float min, float max)
{
    return ((float)rand() / RAND_MAX) * (max - min) + min;
}

// ----------------------------------------------------------------------------------------------------

Kinect::Kinect()
{
    // Set cam model
    cam_model_.setFocalLengths(554.2559327880068, 554.2559327880068);
    cam_model_.setOpticalCenter(320.5, 240.5);
    cam_model_.setOpticalTranslation(0, 0);

    // init rgb image
    image_rgb_ = cv::Mat(480, 640, CV_8UC3, cv::Scalar(255, 255, 255));

    // init depth image
    image_depth_ = cv::Mat(480, 640, CV_32FC1, 0.0);
}

// ----------------------------------------------------------------------------------------------------

Kinect::~Kinect()
{
}

// ----------------------------------------------------------------------------------------------------

void Kinect::setRGBDName(const std::string& name)
{
    rgbd_server_.initialize(name);
}

// ----------------------------------------------------------------------------------------------------

void Kinect::setRGBFrame(const std::string& frame_id) {
    rgb_frame_id_ = frame_id;
}

// ----------------------------------------------------------------------------------------------------

void Kinect::setDepthFrame(const std::string& frame_id) {
    depth_frame_id_ = frame_id;
}

// ----------------------------------------------------------------------------------------------------

void Kinect::addModel(const std::string& type, const std::string& filename) {
    Image image;
    try {
        image = Image::loadFromFile(filename);
        if (!image.getMaskImage().data)
        {
            std::cout << "[FSIM] Image file '" << filename << "' does not contain mask." << std::endl;
            return;
        }
    } catch (rosbag::BagException& e) {
        std::cout << "[FSIM] Could not load Kinect model '" << filename << "'" << std::endl;
        return;
    }
    type_to_image_[type] = image;
}

// ----------------------------------------------------------------------------------------------------

void Kinect::step(World& world)
{
    ros::Time time = ros::Time::now();

    geo::Transform tf_map_to_kinect = getAbsolutePose() * geo::Pose3D(0, 0, 0, 3.1415, 0, 0);

    image_depth_.setTo(0.0f);

    const std::map<std::string, Object*>& objects = world.getObjects();
    for(std::map<std::string, Object*>::const_iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj) {
        Object* obj = it_obj->second;

        geo::ShapePtr shape = obj->getShape();
        if (shape) {
            geo::Transform t = tf_map_to_kinect.inverse() * obj->getAbsolutePose();
            cam_model_.rasterize(*shape, t, image_depth_);
        }

        std::vector<Object*> children;
        obj->getChildrenRecursive(children);

        for(std::vector<Object*>::const_iterator it = children.begin(); it != children.end(); ++it) {
            const Object& child = **it;
            geo::ShapePtr child_shape = child.getShape();
            if (child_shape) {
                geo::Transform t = tf_map_to_kinect.inverse() * child.getAbsolutePose();
                cam_model_.rasterize(*child_shape, t, image_depth_);
            }
        }
    }

    // Add noise to depth image and cut off at max depth
    for(int y = 0; y < image_depth_.rows; ++y)
    {
        for(int x = 0; x < image_depth_.cols; ++x)
        {
            float& d = image_depth_.at<float>(y, x);
            if (d > 8)
                d = 0;
            else if (d > 0)
                d += randomUniform(d * -0.005, d * 0.005);
        }
    }


    // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

    for(int i = 0; i < image_rgb_.cols * image_rgb_.rows; ++i)
        image_rgb_.at<cv::Vec3b>(i) = (image_depth_.at<float>(i) / 10) * cv::Vec3b(0, 255, 0);

    vector<Object*> objects_rec = world.getObjectsRecursive();
    for(vector<Object*>::const_iterator it_obj = objects_rec.begin(); it_obj != objects_rec.end(); ++it_obj) {
        Object& obj = **it_obj;

        map<string, Image>::iterator it_image = type_to_image_.find(obj.getType());
        if (it_image == type_to_image_.end()) {
            it_image = type_to_image_.find(obj.getID());
        }

        if (it_image != type_to_image_.end()) {
            geo::Transform tf_kinect_to_object = tf_map_to_kinect.inverse() * obj.getAbsolutePose();

            double z = -tf_kinect_to_object.getOrigin().getZ();

            if (z > 0 && z < 3) {
                cv::Point2d pos2d = cam_model_.project3Dto2D(tf_kinect_to_object.t);

                if (pos2d.x > 0 && pos2d.x < image_rgb_.cols && pos2d.y > 0 && pos2d.y < image_rgb_.rows) {

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

                    // Scale images based on distance
                    float scale = distance / z;
                    cv::Mat mask_scaled, rgb_img_scaled, depth_img_scaled;
                    cv::resize(mask, mask_scaled, cv::Size(), scale, scale);
                    cv::resize(rgb_img, rgb_img_scaled, cv::Size(), scale, scale);
                    cv::resize(depth_img, depth_img_scaled, cv::Size(), scale, scale);

                    int x_tl = pos2d.x - mask_scaled.cols / 2;
                    int y_tl = pos2d.y - mask_scaled.rows / 2;

                    for(int y = 0; y < mask_scaled.rows; ++y) {
                        for(int x = 0; x < mask_scaled.cols; ++x) {

                            int ix = x_tl + x;
                            int iy = y_tl + y;

                            if (ix >=0 && iy >= 0 && ix < image_rgb_.cols && iy < image_rgb_.rows) {

                                unsigned char alpha = mask_scaled.at<unsigned char>(y, x);
                                if (alpha > 0) {
//                                    image_depth_.image.at<float>(iy, ix) = depth_img.at<float>(y, x) - distance + z;
                                    //cout << "   " << image_depth_.image.at<float>(iy, ix);

                                    cv::Vec3b image_clr =  image_rgb_.at<cv::Vec3b>(iy, ix);
                                    cv::Vec3b object_clr = rgb_img_scaled.at<cv::Vec3b>(y, x);

                                    cv::Vec3b color;
                                    color[0] = (image_clr[0] * (255 - alpha) + object_clr[0] * alpha) / 255;
                                    color[1] = (image_clr[1] * (255 - alpha) + object_clr[1] * alpha) / 255;
                                    color[2] = (image_clr[2] * (255 - alpha) + object_clr[2] * alpha) / 255;

                                    image_rgb_.at<cv::Vec3b>(iy, ix) = color;
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

    rgbd::Image image(image_rgb_, image_depth_, cam_model_, rgb_frame_id_, time.toSec());
    rgbd_server_.send(image);
}
