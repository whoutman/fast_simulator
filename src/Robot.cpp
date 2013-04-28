#include "fast_simulator/Robot.h"

using namespace std;

Robot::Robot(ros::NodeHandle& nh, bool publish_localization) : nh_(nh), publish_localization_(publish_localization) {
    // joint_states
    pub_joint_states = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    // TF
    tf_map_to_odom.setOrigin(tf::Vector3(0, 0, 0));
    tf_map_to_odom.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_map_to_odom.frame_id_ = "/map";
    tf_map_to_odom.child_frame_id_ = "/odom";

    event_loc_pub_.scheduleRecurring(100);
    event_joint_states_pub_.scheduleRecurring(100);
    event_sensors_pub_.scheduleRecurring(10);
}

Robot::~Robot() {
    for(vector<Sensor*>::iterator it_sensor = sensors_.begin(); it_sensor != sensors_.end(); ++it_sensor) {
        delete *it_sensor;
    }
}

void Robot::step(double dt) {
    Object::step(dt);

    for(map<string, Joint*>::iterator it_joint = joints_.begin(); it_joint != joints_.end(); ++it_joint) {
        Joint* joint = it_joint->second;
        joint->step(dt);
    }

    if (event_loc_pub_.isScheduled()) {
        if (publish_localization_) {
            tf_map_to_odom.stamp_ = ros::Time::now();
            tf_broadcaster_.sendTransform(tf_map_to_odom);
        }
    }

    if (event_joint_states_pub_.isScheduled()) {
        sensor_msgs::JointState joint_states = getJointStates();
        pub_joint_states.publish(joint_states);
    }

    if (event_sensors_pub_.isScheduled()) {
        for(vector<Sensor*>::iterator it_sensor = sensors_.begin(); it_sensor != sensors_.end(); ++it_sensor) {
            Sensor* sensor = *it_sensor;
            sensor->publish();
        }
    }
}

void Robot::addSensor(Sensor* sensor, const tf::Transform& rel_pose) {
    this->addChild(sensor, rel_pose);
    sensors_.push_back(sensor);
}

void Robot::setJointPosition(const string& joint_name, double position) {
    map<std::string, Joint*>::iterator it_jnt = joints_.find(joint_name);
    if (it_jnt == joints_.end()) {
        joints_[joint_name] = new Joint(position);
    } else {
        it_jnt->second->position_ = position;
    }
}

void Robot::setJointReference(const string& joint_name, double position) {
    joints_[joint_name]->reference_ = position;
}

double Robot::getJointPosition(const string& joint_name) {
    return joints_[joint_name]->position_;
}

sensor_msgs::JointState Robot::getJointStates() {
    sensor_msgs::JointState joint_states;
    joint_states.header.stamp = ros::Time::now();

    for(map<string, Joint*>::iterator it_joint = joints_.begin(); it_joint != joints_.end(); ++it_joint) {
        Joint* joint = it_joint->second;
        joint_states.name.push_back(it_joint->first);
        joint_states.position.push_back(joint->position_);
    }

    return joint_states;
}
