//#include "fast_simulator/Sergio.h"
#include "../include/fast_simulator/Sergio.h"

#include <geolib/ros/msg_conversions.h>
#include <geolib/ros/tf_conversions.h>
#include <urdf/model.h>

#include <nav_msgs/Odometry.h>

using namespace std;

// ----------------------------------------------------------------------------------------------------

Sergio::Sergio(ros::NodeHandle& nh) : Robot(nh, "sergio")
{
    initJoint("ankle_joint", 0.87);
    initJoint("knee_joint", 2.0);
    initJoint("hip_joint", 2.36);

    initJoint("shoulder_yaw_joint_left", -0.01);
    initJoint("shoulder_pitch_joint_left", -0.4);
    initJoint("shoulder_roll_joint_left", 0);
    initJoint("elbow_pitch_joint_left", 1.2);
    initJoint("elbow_roll_joint_left", 0);
    initJoint("wrist_yaw_joint_left", 0);
    initJoint("wrist_pitch_joint_left", 0.8);
    initJoint("finger1_joint_left", 0.6);
    initJoint("finger2_joint_left", 0.6);
    initJoint("finger1_tip_joint_left", -0.2);
    initJoint("finger2_tip_joint_left", -0.2);

    initJoint("shoulder_yaw_joint_right", -0.01);
    initJoint("shoulder_pitch_joint_right", -0.4);
    initJoint("shoulder_roll_joint_right", 0);
    initJoint("elbow_pitch_joint_right", 1.2);
    initJoint("elbow_roll_joint_right", 0);
    initJoint("wrist_yaw_joint_right", 0);
    initJoint("wrist_pitch_joint_right", 0.8);
    initJoint("finger1_joint_right", 0.6);
    initJoint("finger2_joint_right", 0.6);
    initJoint("finger1_tip_joint_right", -0.2);
    initJoint("finger2_tip_joint_right", -0.2);

    initJoint("neck_pan_joint", 0);
    initJoint("neck_tilt_joint", 0);
    initJoint("laser_tilt_joint", 0);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    urdf::Model model;
    model.initParam("/sergio/robot_description");

    const std::vector<std::string>& joint_names = reference_generator_.joint_names();
    for(unsigned int i = 0 ; i < joint_names.size(); ++i)
    {
        boost::shared_ptr<const urdf::Joint> joint = model.getJoint(joint_names[i]);
        reference_generator_.setPositionLimits(i, joint->limits->lower, joint->limits->upper);
        reference_generator_.setMaxVelocity(i, joint->limits->velocity);
        reference_generator_.setMaxAcceleration(i, joint->limits->effort);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    as_ = new TrajectoryActionServer(nh, "body/joint_trajectory_action", false);
    as_->registerGoalCallback(boost::bind(&Sergio::goalCallback, this, _1));
    as_->registerCancelCallback(boost::bind(&Sergio::cancelCallback, this, _1));
    as_->start();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    pub_body_ = nh.advertise<sensor_msgs::JointState>("/sergio/body/measurements", 10);
    pub_head_ = nh.advertise<sensor_msgs::JointState>("/sergio/neck/measurements", 10);
    pub_dynamixel_ = nh.advertise<sensor_msgs::JointState>("/sergio/neck/measurements", 10);
    pub_left_arm_ = nh.advertise<sensor_msgs::JointState>("/sergio/left_arm/measurements", 10);
    pub_right_arm_ = nh.advertise<sensor_msgs::JointState>("/sergio/right_arm/measurements", 10);
    pub_torso_ = nh.advertise<sensor_msgs::JointState>("/sergio/torso/measurements", 10);
    pub_left_gripper_ = nh.advertise<tue_msgs::GripperMeasurement>("/sergio/left_arm/gripper/measurements", 10);
    pub_right_gripper_ = nh.advertise<tue_msgs::GripperMeasurement>("/sergio/right_arm/gripper/measurements", 10);
    pub_odom_ = nh.advertise<nav_msgs::Odometry>("/sergio/base/measurements", 10);

    // SUBSCRIBERS

    sub_init_pose = nh.subscribe("/sergio/initialpose", 10, &Sergio::callbackInitialPose, this);
    sub_cmd_vel = nh.subscribe("/sergio/base/references", 10, &Sergio::callbackCmdVel, this);

    left_gripper_direction_ = tue_msgs::GripperMeasurement::OPEN;
    right_gripper_direction_ = tue_msgs::GripperMeasurement::OPEN;

    sub_left_gripper = nh.subscribe("/sergio/left_arm/gripper/references", 10, &Sergio::callbackLeftGripper, this);
    sub_right_gripper = nh.subscribe("/sergio/right_arm/gripper/references", 10, &Sergio::callbackRightGripper, this);

    tf_odom_to_base_link.frame_id_ = "/sergio/odom";
    tf_odom_to_base_link.child_frame_id_ = "/sergio/base_link";

    event_odom_pub_.scheduleRecurring(50);
    event_refs_pub_.scheduleRecurring(100);
}

// ----------------------------------------------------------------------------------------------------

Sergio::~Sergio()
{
    delete as_;
}

// ----------------------------------------------------------------------------------------------------

void Sergio::initJoint(const std::string& name, double pos)
{
    reference_generator_.initJoint(name, 0, 0, 0, 0);
    reference_generator_.setJointState(name, pos, 0);
    setJointPosition(name, pos);
}

// ----------------------------------------------------------------------------------------------------

void Sergio::step(double dt)
{
    const std::vector<std::string>& joint_names = reference_generator_.joint_names();

    std::vector<double> references;
    if (!reference_generator_.calculatePositionReferences(dt, references))
        return;

    for(unsigned int i = 0 ; i < joint_names.size(); ++i)
        setJointPosition(joint_names[i], references[i]);

    for(std::map<std::string, TrajectoryActionServer::GoalHandle>::iterator it = goal_handles_.begin(); it != goal_handles_.end();)
    {
        TrajectoryActionServer::GoalHandle& gh = it->second;
        tue::manipulation::JointGoalStatus status = reference_generator_.getGoalStatus(gh.getGoalID().id);

        if (status == tue::manipulation::JOINT_GOAL_SUCCEEDED)
        {
            gh.setSucceeded();
            goal_handles_.erase(it++);
        }
        else if (status == tue::manipulation::JOINT_GOAL_CANCELED)
        {
            gh.setCanceled();
            goal_handles_.erase(it++);
        }
        else
        {
            ++it;
        }
    }

    Robot::step(dt);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if (ros::Time::now() - t_last_cmd_vel_ > ros::Duration(0.5))
    {
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

// ----------------------------------------------------------------------------------------------------

void Sergio::callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
{
    this->velocity_ = *msg;
    t_last_cmd_vel_ = ros::Time::now();
}

// ----------------------------------------------------------------------------------------------------

void Sergio::callbackLeftGripper(const tue_msgs::GripperCommand::ConstPtr& msg)
{
    if (msg->direction == tue_msgs::GripperCommand::CLOSE)
    {
        setJointReference("finger1_joint_left", 0.20);
        setJointReference("finger2_joint_left", 0.20);
        setJointReference("finger1_tip_joint_left", -0.60);
        setJointReference("finger2_tip_joint_left", -0.60);
        left_gripper_direction_ = tue_msgs::GripperMeasurement::CLOSE;
    }
    else if  (msg->direction == tue_msgs::GripperCommand::OPEN)
    {
        setJointReference("finger1_joint_left", 0.60);
        setJointReference("finger2_joint_left", 0.60);
        setJointReference("finger1_tip_joint_left", -0.18);
        setJointReference("finger2_tip_joint_left", -0.18);
        left_gripper_direction_ = tue_msgs::GripperMeasurement::OPEN;
    }
}

// ----------------------------------------------------------------------------------------------------

void Sergio::callbackRightGripper(const tue_msgs::GripperCommand::ConstPtr& msg)
{
    if (msg->direction == tue_msgs::GripperCommand::CLOSE)
    {
        setJointReference("finger1_joint_right", 0.20);
        setJointReference("finger2_joint_right", 0.20);
        setJointReference("finger1_tip_joint_right", -0.60);
        setJointReference("finger2_tip_joint_right", -0.60);
        right_gripper_direction_ = tue_msgs::GripperMeasurement::CLOSE;
    }
    else if  (msg->direction == tue_msgs::GripperCommand::OPEN)
    {
        setJointReference("finger1_joint_right", 0.60);
        setJointReference("finger2_joint_right", 0.60);
        setJointReference("finger1_tip_joint_right", -0.18);
        setJointReference("finger2_tip_joint_right", -0.18);
        right_gripper_direction_ = tue_msgs::GripperMeasurement::OPEN;
    }
}

// ----------------------------------------------------------------------------------------------------

void Sergio::callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    geo::Transform pose;
    geo::convert(msg->pose.pose, pose);
    setPose(pose);
}

// ----------------------------------------------------------------------------------------------------

void Sergio::goalCallback(TrajectoryActionServer::GoalHandle gh)
{
    std::string id = gh.getGoalID().id;

    std::stringstream error;
    if (!reference_generator_.setGoal(*gh.getGoal(), id, error))
    {
        gh.setRejected(control_msgs::FollowJointTrajectoryResult(), error.str());
        ROS_ERROR("%s", error.str().c_str());
        return;
    }

    // Accept the goal
    gh.setAccepted();
    goal_handles_[id] = gh;
}

// ----------------------------------------------------------------------------------------------------

void Sergio::cancelCallback(TrajectoryActionServer::GoalHandle gh)
{
    gh.setCanceled();
    reference_generator_.cancelGoal(gh.getGoalID().id);
    goal_handles_.erase(gh.getGoalID().id);
}

// ----------------------------------------------------------------------------------------------------

void Sergio::publishControlRefs()
{
    std_msgs::Header header;
    header.stamp = ros::Time::now();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    sensor_msgs::JointState body_meas_msg;
    body_meas_msg.header = header;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    for(unsigned int j = 0; j < reference_generator_.joint_names().size(); ++j)
    {
        const std::string& joint_name = reference_generator_.joint_names()[j];
        body_meas_msg.name.push_back(joint_name);
        body_meas_msg.position.push_back(getJointPosition(joint_name));
    }

    pub_body_.publish(body_meas_msg);
    pub_left_arm_.publish(body_meas_msg);
    pub_right_arm_.publish(body_meas_msg);
    pub_torso_.publish(body_meas_msg);

//    std_msgs::Header header;
//    header.stamp = ros::Time::now();

//    sensor_msgs::JointState head_meas_msg;
//    head_meas_msg.header = header;
//    head_meas_msg.name.push_back("neck_pan_joint");
//    head_meas_msg.name.push_back("neck_tilt_joint");
//    head_meas_msg.position.push_back(getJointPosition("neck_pan_joint"));
//    head_meas_msg.position.push_back(getJointPosition("neck_tilt_joint"));
//    pub_head_.publish(head_meas_msg);

//    sensor_msgs::JointState dynamixel_msg;
//    dynamixel_msg.header = header;
//    dynamixel_msg.name.push_back("laser_tilt_joint");
//    dynamixel_msg.position.push_back(getJointPosition("laser_tilt_joint"));
//    pub_dynamixel_.publish(dynamixel_msg);

//    sensor_msgs::JointState torso_meas_msg;
//    torso_meas_msg.header = header;
//    for (unsigned int j = 0; j < torso_joint_names.size(); ++j) {
//        torso_meas_msg.name.push_back(torso_joint_names[j]);
//        torso_meas_msg.position.push_back(getJointPosition(torso_joint_names[j]));
//    }
//    /// Apply constraint
//    // ToDo: don't hardcode!!!
//    ROS_WARN_ONCE("Knee/ankle constraint hardcoded!!!");
//    double q_ankle = torso_meas_msg.position[0];
//    double q_knee  = 1.3365*q_ankle*q_ankle*q_ankle -1.9862*q_ankle*q_ankle + 2.6392*q_ankle + 0.001;
//    torso_meas_msg.position[1] = q_knee;
//    pub_torso_.publish(torso_meas_msg);

//    sensor_msgs::JointState left_arm_joints;
//    left_arm_joints.header = header;
//    for(unsigned int j = 0; j < left_arm_joint_names.size(); ++j) {
//        left_arm_joints.name.push_back(left_arm_joint_names[j]);
//        left_arm_joints.position.push_back(getJointPosition(left_arm_joint_names[j]));
//    }
//    pub_left_arm_.publish(left_arm_joints);

//    sensor_msgs::JointState right_arm_joints;
//    right_arm_joints.header = header;
//    for(unsigned int j = 0; j < right_arm_joint_names.size(); ++j) {
//        right_arm_joints.name.push_back(right_arm_joint_names[j]);
//        right_arm_joints.position.push_back(getJointPosition(right_arm_joint_names[j]));
//    }
//    pub_right_arm_.publish(right_arm_joints);

//    tue_msgs::GripperMeasurement left_gripper;
//    left_gripper.direction = left_gripper_direction_;
//    if ((left_gripper.direction == tue_msgs::GripperMeasurement::CLOSE && getJointPosition("finger1_tip_joint_left") < -0.58)
//            || (left_gripper.direction == tue_msgs::GripperMeasurement::OPEN && getJointPosition("finger1_joint_left") > 0.58)) {
//        left_gripper.end_position_reached = true;
//    }
//    pub_left_gripper_.publish(left_gripper);

//    tue_msgs::GripperMeasurement right_gripper;
//    right_gripper.direction = right_gripper_direction_;
//    if ((right_gripper.direction == tue_msgs::GripperMeasurement::CLOSE && getJointPosition("finger1_tip_joint_right") < -0.58)
//            || (right_gripper.direction == tue_msgs::GripperMeasurement::OPEN && getJointPosition("finger1_joint_right") > 0.58)) {
//        right_gripper.end_position_reached = true;
//    }
//    pub_right_gripper_.publish(right_gripper);
}

// ----------------------------------------------------------------------------------------------------
