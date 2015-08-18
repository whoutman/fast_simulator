#include "fast_simulator/Pera.h"

#include <geolib/ros/msg_conversions.h>
#include <geolib/ros/tf_conversions.h>

using namespace std;

Pera::Pera(ros::NodeHandle& nh) : Robot(nh, "pera") {

    setJointPosition("shoulder_yaw_joint_left", -0.010038043598955326);
    setJointPosition("shoulder_pitch_joint_left", -0.39997462399515005);
    setJointPosition("shoulder_roll_joint_left", 2.0889754646091774e-06);
    setJointPosition("elbow_pitch_joint_left", 1.1999600775244508);
    setJointPosition("elbow_roll_joint_left", 4.330400908969523e-07);
    setJointPosition("wrist_yaw_joint_left", 1.7639288287796262e-06);
    setJointPosition("wrist_pitch_joint_left", 0.7999636309384188);
    setJointPosition("finger1_joint_left", 0.6000396498469334);
    setJointPosition("finger2_joint_left", 0.6000510112333535);
    setJointPosition("finger1_tip_joint_left", -0.20000170842812892);
    setJointPosition("finger2_tip_joint_left", -0.1999984035736233);

    left_arm_joint_names.push_back("shoulder_yaw_joint_left");
    left_arm_joint_names.push_back("shoulder_pitch_joint_left");
    left_arm_joint_names.push_back("shoulder_roll_joint_left");
    left_arm_joint_names.push_back("elbow_pitch_joint_left");
    left_arm_joint_names.push_back("elbow_roll_joint_left");
    left_arm_joint_names.push_back("wrist_pitch_joint_left");
    left_arm_joint_names.push_back("wrist_yaw_joint_left");

    pub_left_arm_ = nh.advertise<sensor_msgs::JointState>("/pera/measurements", 10);
    pub_left_gripper_ = nh.advertise<tue_msgs::GripperMeasurement>("/pera/gripper_measurement", 10);

    // SUBSCRIBERS

    sub_left_arm = nh.subscribe("/pera/references", 10, &Pera::callbackLeftArm, this);
    sub_left_gripper = nh.subscribe("/pera/gripper_command", 10, &Pera::callbackLeftGripper, this);

    left_gripper_direction_ = tue_msgs::GripperMeasurement::OPEN;

    event_refs_pub_.scheduleRecurring(100);

    tf_location_.frame_id_ = "/map";
    tf_location_.child_frame_id_ = "/pera/shoulder_mount_left";
    event_loc_pub_.scheduleRecurring(50);
}

Pera::~Pera() {

}

void Pera::step(double dt) {
    Robot::step(dt);
    publishControlRefs();

    if (event_loc_pub_.isScheduled()) {
        tf_location_.stamp_ = ros::Time::now();
        geo::convert(getAbsolutePose(), tf_location_);
        tf_broadcaster_.sendTransform(tf_location_);
    }

}

void Pera::callbackLeftGripper(const tue_msgs::GripperCommand::ConstPtr& msg) {
    if (msg->direction == tue_msgs::GripperCommand::CLOSE) {
        setJointReference("finger1_joint_left", 0.20);
        setJointReference("finger2_joint_left", 0.20);
        setJointReference("finger1_tip_joint_left", -0.60);
        setJointReference("finger2_tip_joint_left", -0.60);
        left_gripper_direction_ = tue_msgs::GripperMeasurement::CLOSE;
    } else if  (msg->direction == tue_msgs::GripperCommand::OPEN) {
        setJointReference("finger1_joint_left", 0.60);
        setJointReference("finger2_joint_left", 0.60);
        setJointReference("finger1_tip_joint_left", -0.18);
        setJointReference("finger2_tip_joint_left", -0.18);
        left_gripper_direction_ = tue_msgs::GripperMeasurement::OPEN;
    }
}

void Pera::callbackLeftArm(const sensor_msgs::JointState::ConstPtr& msg) {
    for(unsigned int i = 0; i < msg->name.size(); ++i) {
        setJointReference(msg->name[i], msg->position[i]);
    }
}

void Pera::publishControlRefs() {
   sensor_msgs::JointState left_arm_joints;
    for(unsigned int j = 0; j < left_arm_joint_names.size(); ++j) {
        left_arm_joints.name.push_back(left_arm_joint_names[j]);
        left_arm_joints.position.push_back(getJointPosition(left_arm_joint_names[j]));
    }
    pub_left_arm_.publish(left_arm_joints);

    tue_msgs::GripperMeasurement left_gripper;
    left_gripper.direction = left_gripper_direction_;
    if ((left_gripper.direction == tue_msgs::GripperMeasurement::CLOSE && getJointPosition("finger1_tip_joint_left") < -0.58)
            || (left_gripper.direction == tue_msgs::GripperMeasurement::OPEN && getJointPosition("finger1_joint_left") > 0.58)) {
        left_gripper.end_position_reached = true;
    }
    pub_left_gripper_.publish(left_gripper);
}

