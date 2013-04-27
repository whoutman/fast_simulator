#include "fast_simulator/Kinect.h"
#include "fast_simulator/World.h"
#include "virtual_cam/Loader.h"

#include "fast_simulator/util.h"

using namespace std;

Kinect::Kinect(const string& rgb_topic, const string& depth_topic, const string& info_topic, const string& frame_id) {
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
    image_rgb_.height = 480;
    image_rgb_.width = 640;
    image_rgb_.encoding = "rgb8";
    image_rgb_.is_bigendian = false;
    image_rgb_.step = 1920;

    for (int i = 0; i < 640 * 480 * 3; ++i) {
        image_rgb_.data.push_back(0);
    }

    ros::NodeHandle nh;
    pub_rgb = nh.advertise<sensor_msgs::Image>(rgb_topic, 1000);
    pub_depth = nh.advertise<sensor_msgs::Image>(depth_topic, 1000);
    pub_cam_info = nh.advertise<sensor_msgs::CameraInfo>(info_topic, 1000);
}

Kinect::~Kinect() {
    delete image_loader_;
}

void Kinect::addModel(const std::string& type, const std::string& filename) {
    type_to_filename_[type] = filename;
}

void Kinect::publish() {
    World& world = World::getInstance();
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
        pub_cam_info.publish(cam_info_);
        pub_rgb.publish(image_rgb_);
    }

}

/*
#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_broadcaster.h>

using namespace std;

string FRAME_ID = "/director/camera1";

tf::Vector3 vel_trans_;

tf::Vector3 cam_pos_;

void twistCallback(const geometry_msgs::TwistConstPtr& twist) {
    tf::vector3MsgToTF(twist->linear, vel_trans_);
}

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "");
    ros::NodeHandle nh;

    vel_trans_.setZero();
    cam_pos_.setZero();

    // advertising a topic to send data
    ros::Publisher pub_cam_info = nh.advertise<sensor_msgs::CameraInfo>("/director/camera1/camera_info", 10);
    ros::Publisher pub_cam_image = nh.advertise<sensor_msgs::Image>("/director/camera1/image", 10);

    ros::Subscriber sub_twist = nh.subscribe("/director/camera1/twist", 1, twistCallback);

    sensor_msgs::CameraInfo cam_info;
    cam_info.header.frame_id = FRAME_ID;
    cam_info.height = 480;
    cam_info.width = 640;
    cam_info.distortion_model = "plumb_bob";

    // D: [0.0, 0.0, 0.0, 0.0, 0.0]
    cam_info.D.push_back(0);
    cam_info.D.push_back(0);
    cam_info.D.push_back(0);
    cam_info.D.push_back(0);
    cam_info.D.push_back(0);

    // K: [554.2559327880068, 0.0, 320.5, 0.0, 554.2559327880068, 240.5, 0.0, 0.0, 1.0]
    cam_info.K[0] = 554.2559327880068;
    cam_info.K[1] = 0;
    cam_info.K[2] = 320.5;
    cam_info.K[3] = 0;
    cam_info.K[4] = 554.2559327880068;
    cam_info.K[5] = 240.5;
    cam_info.K[6] = 0;
    cam_info.K[7] = 0;
    cam_info.K[8] = 1;

    // R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    cam_info.R[0] = 1;
    cam_info.R[1] = 0;
    cam_info.R[2] = 0;
    cam_info.R[3] = 0;
    cam_info.R[4] = 1;
    cam_info.R[5] = 0;
    cam_info.R[6] = 0;
    cam_info.R[7] = 0;
    cam_info.R[8] = 1;

    // P: [554.2559327880068, 0.0, 320.5, -0.0, 0.0, 554.2559327880068, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]
    cam_info.P[0] = 554.2559327880068;
    cam_info.P[1] = 0;
    cam_info.P[2] = 320.5;
    cam_info.P[3] = 0;
    cam_info.P[4] = 0;
    cam_info.P[5] = 554.2559327880068;
    cam_info.P[6] = 240.5;
    cam_info.P[7] = 0;
    cam_info.P[8] = 0;
    cam_info.P[9] = 0;
    cam_info.P[10] = 1;
    cam_info.P[11] = 0;

    cam_info.binning_x = 0;
    cam_info.binning_y = 0;

    cam_info.roi.x_offset = 0;
    cam_info.roi.y_offset = 0;
    cam_info.roi.height = 0;
    cam_info.roi.width = 0;
    cam_info.roi.do_rectify = false;

    sensor_msgs::Image image;
    image.header.frame_id = FRAME_ID;
    image.height = 480;
    image.width = 640;
    image.encoding = "rgb8";
    image.is_bigendian = false;
    image.step = 1920;

    for (int i = 0; i < 640 * 480 * 3; ++i) {
        image.data.push_back(0);
    }

    tf::TransformBroadcaster tf_broadcaster;

    tf::StampedTransform t;
    t.child_frame_id_ = FRAME_ID;
    t.frame_id_ = "/map";

    double yaw = 0;

    double rate = 25;
    double dt = 1 / rate;


    ros::Rate r(rate);
    while (ros::ok()) {
        pub_cam_info.publish(cam_info);
        pub_cam_image.publish(image);

        t.stamp_ = ros::Time::now();
        tf::Quaternion q;
        q.setRPY(-1.54 - 0.3, 0.0, yaw);
        t.setData(tf::Transform(q, cam_pos_));

        tf_broadcaster.sendTransform(t);

        r.sleep();

        cam_pos_ += dt * vel_trans_;

        yaw += dt * 0.1;

        ros::spinOnce();
    }


    return 0;
}
*/
