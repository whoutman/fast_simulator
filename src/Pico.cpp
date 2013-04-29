#include "fast_simulator/Pico.h"

using namespace std;

Pico::Pico(ros::NodeHandle& nh, bool publish_localization) : nh_(nh), publish_localization_(publish_localization) {
    // joint_states
    pub_joint_states = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    joints_["head_yaw_joint"] = new Joint(0);
    joints_["head_pitch_joint"] = new Joint(0);

    pub_head_pan_ = nh.advertise<std_msgs::Float64>("/robot/body/yaw_in", 10);
    pub_head_tilt_ = nh.advertise<std_msgs::Float64>("/robot/body/pitch_in", 10);

    // add laser
    tf::Transform tf_base_link_to_front_laser;
    tf_base_link_to_front_laser.setOrigin(tf::Vector3(0.1, 0, 0.37));
    tf_base_link_to_front_laser.setRotation(tf::Quaternion(0, 0, 0, 1));
    laser_range_finder_ = new LRF("/robot/body/laser", "/laser");
    this->addChild(laser_range_finder_, tf_base_link_to_front_laser);

    // add sonar
    tf::Transform tf_base_link_to_front_sonar;
    tf_base_link_to_front_sonar.setOrigin(tf::Vector3(0.1, 0, 0.37));
    tf_base_link_to_front_sonar.setRotation(tf::Quaternion(0, 0, 0, 1));
    front_sonar_ = new Sonar("/robot/body/sonar_front", "/laser");
    this->addChild(front_sonar_, tf_base_link_to_front_sonar);

    // SUBSCRIBERS

    // cmd_vel
    sub_cmd_vel = nh.subscribe("/robot_in", 10, &Pico::callbackCmdVel, this);

    sub_init_pose = nh.subscribe("/initialpose", 10, &Pico::callbackInitialPose, this);

    sub_head_pan_ = nh.subscribe("/robot/body/yaw_out", 10, &Pico::callbackHeadPan, this);
    sub_head_tilt_ = nh.subscribe("/robot/body/pitch_out", 10, &Pico::callbackHeadTilt, this);

    // TF
    tf_map_to_odom.setOrigin(tf::Vector3(0, 0, 0));
    tf_map_to_odom.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_map_to_odom.frame_id_ = "/map";
    tf_map_to_odom.child_frame_id_ = "/odom";

    tf_odom_to_base_link.frame_id_ = "/odom";
    tf_odom_to_base_link.child_frame_id_ = "/base_link";

    count_ = 0;
}

Pico::~Pico() {
}

void Pico::step(double dt) {
    Object::step(dt);

    for(map<string, Joint*>::iterator it_joint = joints_.begin(); it_joint != joints_.end(); ++it_joint) {
        Joint* joint = it_joint->second;
        joint->step(dt);
    }

    if (ros::Time::now() - t_last_cmd_vel_ > ros::Duration(0.5)) {
        geometry_msgs::Twist& vel = this->description_->velocity_;

        vel.angular.x = 0;
        vel.angular.y = 0;
        vel.angular.z = 0;

        vel.linear.x = 0;
        vel.linear.y = 0;
        vel.linear.z = 0;
    }

    if (count_ % 4 == 0) {
        tf_odom_to_base_link.stamp_ = ros::Time::now();
        tf_odom_to_base_link.setOrigin(this->pose_.getOrigin());
        tf_odom_to_base_link.setRotation(this->pose_.getRotation());
        tf_broadcaster_.sendTransform(tf_odom_to_base_link);

        if (publish_localization_) {
            tf_map_to_odom.stamp_ = ros::Time::now();
            tf_broadcaster_.sendTransform(tf_map_to_odom);
        }
    }

    if (count_ % 4 == 0) {
        sensor_msgs::JointState joint_states = getJointStates();
        pub_joint_states.publish(joint_states);
    }

    publishControlRefs();

    if (count_ % 10 == 0) {
        //laser_range_finder_->publish();
        front_sonar_->publishScan();
    }

    count_++;
}

void Pico::setJointReference(const string& joint_name, double position) {
    joints_[joint_name]->reference_ = position;
}

double Pico::getJointPosition(const string& joint_name) {
    return joints_[joint_name]->position_;
}

void Pico::callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg) {
    this->description_->velocity_ = *msg;
    t_last_cmd_vel_ = ros::Time::now();
}

void Pico::callbackHeadPan(const std_msgs::Float64::ConstPtr& msg) {
    setJointReference("head_yaw_joint", msg->data);
}

void Pico::callbackHeadTilt(const std_msgs::Float64::ConstPtr& msg) {
    setJointReference("head_pitch_joint", msg->data);
}

void Pico::callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    tf::poseMsgToTF(msg->pose.pose, this->pose_);
}

void Pico::publishControlRefs() {
    std_msgs::Float64 f;
    f.data = getJointPosition("head_yaw_joint");
    pub_head_pan_.publish(f);

    f.data = getJointPosition("head_pitch_joint");
    pub_head_tilt_.publish(f);
}

sensor_msgs::JointState Pico::getJointStates() {
    sensor_msgs::JointState joint_states;
    joint_states.header.stamp = ros::Time::now();

    for(map<string, Joint*>::iterator it_joint = joints_.begin(); it_joint != joints_.end(); ++it_joint) {
        Joint* joint = it_joint->second;
        joint_states.name.push_back(it_joint->first);
        joint_states.position.push_back(joint->position_);
    }

    return joint_states;
}
