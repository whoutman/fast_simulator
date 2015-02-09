#include "fast_simulator/Sergio.h"

#include <geolib/ros/msg_conversions.h>
#include <geolib/ros/tf_conversions.h>

#include <nav_msgs/Odometry.h>

using namespace std;

Sergio::Sergio(ros::NodeHandle& nh, bool publish_localization) : Robot(nh, "sergio", publish_localization) {

    setJointPosition("ankle_joint", 0.87);
    setJointPosition("knee_joint", 2.0);
    setJointPosition("hip_joint", 2.36);
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
    setJointPosition("shoulder_yaw_joint_right", -0.01004552338080078);
    setJointPosition("shoulder_pitch_joint_right", -0.39998108009563715);
    setJointPosition("shoulder_roll_joint_right", 9.422008346859911e-06);
    setJointPosition("elbow_pitch_joint_right", 1.1999679974909059);
    setJointPosition("elbow_roll_joint_right", 2.5800741013881634e-05);
    setJointPosition("wrist_yaw_joint_right", 4.025142829355843e-05);
    setJointPosition("wrist_pitch_joint_right", 0.7999828748945985);
    setJointPosition("finger1_joint_right", 0.6000464445187825);
    setJointPosition("finger2_joint_right", 0.6000300525013822);
    setJointPosition("finger1_tip_joint_right", -0.1999953219398023);
    setJointPosition("finger2_tip_joint_right", -0.20000480019727807);
    setJointPosition("neck_pan_joint", -3.033573445776483e-07);
    setJointPosition("neck_tilt_joint", 0.00029286782768789266);
    setJointPosition("laser_tilt_joint", 0.00029286782768789266);

    left_arm_joint_names.push_back("shoulder_yaw_joint_left");
    left_arm_joint_names.push_back("shoulder_pitch_joint_left");
    left_arm_joint_names.push_back("shoulder_roll_joint_left");
    left_arm_joint_names.push_back("elbow_pitch_joint_left");
    left_arm_joint_names.push_back("elbow_roll_joint_left");
    left_arm_joint_names.push_back("wrist_pitch_joint_left");
    left_arm_joint_names.push_back("wrist_yaw_joint_left");

    right_arm_joint_names.push_back("shoulder_yaw_joint_right");
    right_arm_joint_names.push_back("shoulder_pitch_joint_right");
    right_arm_joint_names.push_back("shoulder_roll_joint_right");
    right_arm_joint_names.push_back("elbow_pitch_joint_right");
    right_arm_joint_names.push_back("elbow_roll_joint_right");
    right_arm_joint_names.push_back("wrist_pitch_joint_right");
    right_arm_joint_names.push_back("wrist_yaw_joint_right");

    torso_joint_names.push_back("ankle_joint");
    torso_joint_names.push_back("knee_joint");
    torso_joint_names.push_back("hip_joint");

    pub_head_ = nh.advertise<sensor_msgs::JointState>("/sergio/neck/measurements", 10);
    pub_dynamixel_ = nh.advertise<sensor_msgs::JointState>("/sergio/dynamixel/measurements", 10);
    pub_arms_ = nh.advertise<sensor_msgs::JointState>("/sergio/joint_states", 10);
    pub_torso_ = nh.advertise<sensor_msgs::JointState>("/sergio/torso/measurements", 10);
    pub_left_gripper_ = nh.advertise<tue_msgs::GripperMeasurement>("/sergio/left_gripper/measurements", 10);
    pub_right_gripper_ = nh.advertise<tue_msgs::GripperMeasurement>("/sergio/right_gripper/measurements", 10);
    pub_odom_ = nh.advertise<nav_msgs::Odometry>("/sergio/base/measurements", 10);

    // SUBSCRIBERS

    sub_init_pose = nh.subscribe("/sergio/initialpose", 10, &Sergio::callbackInitialPose, this);
    sub_cmd_vel = nh.subscribe("/sergio/base/references", 10, &Sergio::callbackCmdVel, this);
    sub_head = nh.subscribe("/sergio/neck/references", 10, &Sergio::callbackJointReference, this);
    sub_dynamixel = nh.subscribe("/sergio/dynamixel/references", 10, &Sergio::callbackJointReference, this);
    sub_spindle = nh.subscribe("/sergio/torso/references", 10, &Sergio::callbackJointReference, this);
    sub_left_arm = nh.subscribe("/sergio/left_arm/references", 10, &Sergio::callbackJointReference, this);
    sub_right_arm = nh.subscribe("/sergio/right_arm/references", 10, &Sergio::callbackJointReference, this);

    sub_spindle_traj_ = nh.subscribe("/sergio/torso/ref_trajectory", 10, &Sergio::callbackJointTrajectory, this);
    sub_left_arm_traj_ = nh.subscribe("/sergio/left_arm/ref_trajectory", 10, &Sergio::callbackJointTrajectory, this);
    sub_right_arm_traj_ = nh.subscribe("/sergio/right_arm/ref_trajectory", 10, &Sergio::callbackJointTrajectory, this);

    left_gripper_direction_ = tue_msgs::GripperMeasurement::OPEN;
    right_gripper_direction_ = tue_msgs::GripperMeasurement::OPEN;

    sub_left_gripper = nh.subscribe("/sergio/left_gripper/references", 10, &Sergio::callbackLeftGripper, this);
    sub_right_gripper = nh.subscribe("/sergio/right_gripper/references", 10, &Sergio::callbackRightGripper, this);

    tf_odom_to_base_link.frame_id_ = "/sergio/odom";
    tf_odom_to_base_link.child_frame_id_ = "/sergio/base_link";

    event_odom_pub_.scheduleRecurring(50);
    event_refs_pub_.scheduleRecurring(100);
}

Sergio::~Sergio() {

}

void Sergio::step(double dt) {
    for (std::map<std::string, Trajectory>::iterator it = joint_trajectories_.begin(); it != joint_trajectories_.end(); ++it) {
        Trajectory& t = it->second;
        if (!t.set_points.empty()) {
            int num_steps = std::max(1, (int)(dt / t.dt));

            double ref = t.set_points.back();
            for (int i = 0; i < num_steps && !t.set_points.empty(); ++i) {
                ref = t.set_points.back();
                t.set_points.pop_back();
            }

            setJointReference(it->first, ref);
        }
    }

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
        /// Send tf
        tf_odom_to_base_link.stamp_ = ros::Time::now();
        geo::convert(getAbsolutePose(), tf_odom_to_base_link);
        tf_broadcaster_.sendTransform(tf_odom_to_base_link);

        /// Send odom message
        // Fill pose
        nav_msgs::Odometry odom_msg;
        odom_msg.header.frame_id = "/sergio/odom";
        odom_msg.header.stamp    = ros::Time::now();
        odom_msg.child_frame_id  = "/sergio/base_link";
        tf::Vector3 position = tf_odom_to_base_link.getOrigin();
        odom_msg.pose.pose.position.x = position.getX();
        odom_msg.pose.pose.position.y = position.getY();
        odom_msg.pose.pose.position.z = position.getZ();
        tf::Quaternion orientation = tf_odom_to_base_link.getRotation();
        odom_msg.pose.pose.orientation.x = orientation.getX();
        odom_msg.pose.pose.orientation.y = orientation.getY();
        odom_msg.pose.pose.orientation.z = orientation.getZ();
        odom_msg.pose.pose.orientation.w = orientation.getW();
        //ROS_INFO("Position = [%f, %f, %f]",position.getX(),position.getY(),position.getZ());
        // ToDo: fill covariance

        // Fill twist (assume base controller can follow this->velocity_)
        geometry_msgs::Twist base_vel = this->velocity_;
        odom_msg.twist.twist.linear.x  = base_vel.linear.x;
        odom_msg.twist.twist.linear.y  = base_vel.linear.y;
        odom_msg.twist.twist.angular.z = base_vel.angular.z;
        //ROS_INFO("Twist: [%f, %f, %f]", odom_msg.twist.twist.linear.x, odom_msg.twist.twist.linear.y, odom_msg.twist.twist.angular.z);
        // ToDo: fill covariance

        pub_odom_.publish(odom_msg);

    }

    publishControlRefs();

}

void Sergio::callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg) {
    this->velocity_ = *msg;
    t_last_cmd_vel_ = ros::Time::now();
}

void Sergio::callbackLeftGripper(const tue_msgs::GripperCommand::ConstPtr& msg) {
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

void Sergio::callbackRightGripper(const tue_msgs::GripperCommand::ConstPtr& msg) {
    if (msg->direction == tue_msgs::GripperCommand::CLOSE) {
        setJointReference("finger1_joint_right", 0.20);
        setJointReference("finger2_joint_right", 0.20);
        setJointReference("finger1_tip_joint_right", -0.60);
        setJointReference("finger2_tip_joint_right", -0.60);
        right_gripper_direction_ = tue_msgs::GripperMeasurement::CLOSE;
    } else if  (msg->direction == tue_msgs::GripperCommand::OPEN) {
        setJointReference("finger1_joint_right", 0.60);
        setJointReference("finger2_joint_right", 0.60);
        setJointReference("finger1_tip_joint_right", -0.18);
        setJointReference("finger2_tip_joint_right", -0.18);
        right_gripper_direction_ = tue_msgs::GripperMeasurement::OPEN;
    }
}

void Sergio::callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    geo::Transform pose;
    geo::convert(msg->pose.pose, pose);
    setPose(pose);
}

void Sergio::callbackJointReference(const sensor_msgs::JointState::ConstPtr msg) {
    for(unsigned int i = 0; i < msg->name.size(); ++i) {
        setJointReference(msg->name[i], msg->position[i]);
    }
}

void Sergio::callbackJointTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr msg) {
    for(unsigned int i = 0; i < msg->joint_names.size(); ++i) {
        Trajectory& trajectory = joint_trajectories_[msg->joint_names[i]];
        trajectory.set_points.resize(msg->points.size());

        if (msg->points.size() >= 2) {
            trajectory.dt = (msg->points[1].time_from_start - msg->points[0].time_from_start).toSec();
        } else {
            trajectory.dt = 0;
        }

        for(unsigned int j = 0; j < msg->points.size(); ++j) {
            trajectory.set_points[msg->points.size() - j - 1] = msg->points[j].positions[i];
        }
    }
}

void Sergio::publishControlRefs() {
    std_msgs::Header header;
    header.stamp = ros::Time::now();

    sensor_msgs::JointState head_meas_msg;
    head_meas_msg.header = header;
    head_meas_msg.name.push_back("neck_pan_joint");
    head_meas_msg.name.push_back("neck_tilt_joint");
    head_meas_msg.position.push_back(getJointPosition("neck_pan_joint"));
    head_meas_msg.position.push_back(getJointPosition("neck_tilt_joint"));
    pub_head_.publish(head_meas_msg);

    sensor_msgs::JointState dynamixel_msg;
    dynamixel_msg.header = header;
    dynamixel_msg.name.push_back("laser_tilt_joint");
    dynamixel_msg.position.push_back(getJointPosition("laser_tilt_joint"));
    pub_dynamixel_.publish(dynamixel_msg);

    sensor_msgs::JointState torso_meas_msg;
    torso_meas_msg.header = header;
    for (unsigned int j = 0; j < torso_joint_names.size(); ++j) {
        torso_meas_msg.name.push_back(torso_joint_names[j]);
        torso_meas_msg.position.push_back(getJointPosition(torso_joint_names[j]));
    }
    /// Apply constraint
    // ToDo: don't hardcode!!!
    ROS_WARN_ONCE("Knee/ankle constraint hardcoded!!!");
    double q_ankle = torso_meas_msg.position[0];
    double q_knee  = 1.3365*q_ankle*q_ankle*q_ankle -1.9862*q_ankle*q_ankle + 2.6392*q_ankle + 0.001;
    torso_meas_msg.position[1] = q_knee;
    pub_torso_.publish(torso_meas_msg);

    sensor_msgs::JointState arm_joints;
    arm_joints.header = header;
    for(unsigned int j = 0; j < left_arm_joint_names.size(); ++j) {
        arm_joints.name.push_back(left_arm_joint_names[j]);
        arm_joints.position.push_back(getJointPosition(left_arm_joint_names[j]));
    }
    for(unsigned int j = 0; j < right_arm_joint_names.size(); ++j) {
        arm_joints.name.push_back(right_arm_joint_names[j]);
        arm_joints.position.push_back(getJointPosition(right_arm_joint_names[j]));
    }
    pub_arms_.publish(arm_joints);

    tue_msgs::GripperMeasurement left_gripper;
    left_gripper.direction = left_gripper_direction_;
    if ((left_gripper.direction == tue_msgs::GripperMeasurement::CLOSE && getJointPosition("finger1_tip_joint_left") < -0.58)
            || (left_gripper.direction == tue_msgs::GripperMeasurement::OPEN && getJointPosition("finger1_joint_left") > 0.58)) {
        left_gripper.end_position_reached = true;
    }
    pub_left_gripper_.publish(left_gripper);

    tue_msgs::GripperMeasurement right_gripper;
    right_gripper.direction = right_gripper_direction_;
    if ((right_gripper.direction == tue_msgs::GripperMeasurement::CLOSE && getJointPosition("finger1_tip_joint_right") < -0.58)
            || (right_gripper.direction == tue_msgs::GripperMeasurement::OPEN && getJointPosition("finger1_joint_right") > 0.58)) {
        right_gripper.end_position_reached = true;
    }
    pub_right_gripper_.publish(right_gripper);
}

