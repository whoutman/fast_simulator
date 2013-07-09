#include "fast_simulator/Pera.h"

using namespace std;

Pera::Pera(ros::NodeHandle& nh, bool publish_localization) : Robot(nh, publish_localization) {

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

    pub_left_arm_ = nh.advertise<amigo_msgs::arm_joints>("/arm_left_controller/joint_measurements", 10);
    pub_left_gripper_ = nh.advertise<amigo_msgs::AmigoGripperMeasurement>("/arm_left_controller/gripper_measurement", 10);

    // SUBSCRIBERS

    sub_left_arm = nh.subscribe("/arm_left_controller/joint_references", 10, &Pera::callbackLeftArm, this);
    sub_left_gripper = nh.subscribe("/arm_left_controller/gripper_command", 10, &Pera::callbackLeftGripper, this);

    left_gripper_direction_ = amigo_msgs::AmigoGripperMeasurement::OPEN;

    event_refs_pub_.scheduleRecurring(100);
}

Pera::~Pera() {

}

void Pera::step(double dt) {
    Robot::step(dt);
    publishControlRefs();

}

void Pera::callbackLeftGripper(const amigo_msgs::AmigoGripperCommand::ConstPtr& msg) {
    if (msg->direction == amigo_msgs::AmigoGripperCommand::CLOSE) {
        setJointReference("finger1_joint_left", 0.20);
        setJointReference("finger2_joint_left", 0.20);
        setJointReference("finger1_tip_joint_left", -0.60);
        setJointReference("finger2_tip_joint_left", -0.60);
        left_gripper_direction_ = amigo_msgs::AmigoGripperMeasurement::CLOSE;
    } else if  (msg->direction == amigo_msgs::AmigoGripperCommand::OPEN) {
        setJointReference("finger1_joint_left", 0.60);
        setJointReference("finger2_joint_left", 0.60);
        setJointReference("finger1_tip_joint_left", -0.18);
        setJointReference("finger2_tip_joint_left", -0.18);
        left_gripper_direction_ = amigo_msgs::AmigoGripperMeasurement::OPEN;
    }
}

void Pera::callbackLeftArm(const amigo_msgs::arm_joints::ConstPtr& msg) {
    for(unsigned int i = 0; i < msg->pos.size(); ++i) {
        setJointReference(left_arm_joint_names[i], msg->pos[i].data);
    }
}

void Pera::publishControlRefs() {
    amigo_msgs::arm_joints left_arm_joints;
    for(unsigned int j = 0; j < left_arm_joint_names.size(); ++j) {
        left_arm_joints.pos[j].data = getJointPosition(left_arm_joint_names[j]);
    }
    pub_left_arm_.publish(left_arm_joints);

    amigo_msgs::AmigoGripperMeasurement left_gripper;
    left_gripper.direction = left_gripper_direction_;
    if ((left_gripper.direction == amigo_msgs::AmigoGripperMeasurement::CLOSE && getJointPosition("finger1_tip_joint_left") < -0.58)
            || (left_gripper.direction == amigo_msgs::AmigoGripperMeasurement::OPEN && getJointPosition("finger1_joint_left") > 0.58)) {
        left_gripper.end_position_reached = true;
    }
    pub_left_gripper_.publish(left_gripper);
}

