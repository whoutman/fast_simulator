#include "fast_simulator/Pico.h"

using namespace std;

Pico::Pico(ros::NodeHandle& nh, bool publish_localization) : Robot(nh, "pico", publish_localization) {

    setJointPosition("head_yaw_joint", 0.0);
    setJointPosition("head_pitch_joint", 0.0);

    pub_head_pan_ = nh.advertise<std_msgs::Float64>("/pico/head/measurement/yaw", 10);
    pub_head_tilt_ = nh.advertise<std_msgs::Float64>("/pico/head/measurement/pitch", 10);

    // SUBSCRIBERS

    // cmd_vel
    sub_cmd_vel = nh.subscribe("/pico/cmd_vel", 10, &Pico::callbackCmdVel, this);

    sub_init_pose = nh.subscribe("/pico/initialpose", 10, &Pico::callbackInitialPose, this);

    sub_head_pan_ = nh.subscribe("/pico/head/reference/yaw", 10, &Pico::callbackHeadPan, this);
    sub_head_tilt_ = nh.subscribe("/pico/head/reference/pitch", 10, &Pico::callbackHeadTilt, this);

    tf_odom_to_base_link.frame_id_ = "/pico/odom";
    tf_odom_to_base_link.child_frame_id_ = "/pico/base_link";

    event_odom_pub_.scheduleRecurring(50);
}

Pico::~Pico() {
}

void Pico::step(double dt) {
    Robot::step(dt);

    if (ros::Time::now() - t_last_cmd_vel_ > ros::Duration(0.5)) {
        geometry_msgs::Twist& vel = this->velocity_;

        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = 0;

        vel.linear.x = 0;
        vel.linear.y = 0;
        vel.linear.z = 0;
    }

    if (event_odom_pub_.isScheduled()) {
        tf_odom_to_base_link.stamp_ = ros::Time::now();
        tf_odom_to_base_link.setOrigin(getAbsolutePose().getOrigin());
        tf_odom_to_base_link.setRotation(getAbsolutePose().getRotation());
        tf_broadcaster_.sendTransform(tf_odom_to_base_link);
    }

    publishControlRefs();

}

void Pico::callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg) {
    this->velocity_ = *msg;
    t_last_cmd_vel_ = ros::Time::now();
}

void Pico::callbackHeadPan(const std_msgs::Float64::ConstPtr& msg) {
    setJointReference("head_yaw_joint", msg->data);
}

void Pico::callbackHeadTilt(const std_msgs::Float64::ConstPtr& msg) {
    setJointReference("head_pitch_joint", msg->data);
}

void Pico::callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    tf::Transform pose;
    tf::poseMsgToTF(msg->pose.pose, pose);
    setPose(pose.getOrigin(), pose.getRotation());
}

void Pico::publishControlRefs() {
    std_msgs::Float64 f;
    f.data = getJointPosition("head_yaw_joint");
    pub_head_pan_.publish(f);

    f.data = getJointPosition("head_pitch_joint");
    pub_head_tilt_.publish(f);
}
