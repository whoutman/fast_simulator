#include "../include/fast_simulator/Amigo.h"

#include <geolib/ros/msg_conversions.h>
#include <geolib/ros/tf_conversions.h>

using namespace std;

// ----------------------------------------------------------------------------------------------------

BodyPart::BodyPart() : robot_(NULL)
{
}

// ----------------------------------------------------------------------------------------------------

BodyPart::~BodyPart()
{
    for(std::vector<TrajectoryActionServer*>::iterator it = action_servers_.begin(); it != action_servers_.end(); ++it)
        delete *it;
}

// ----------------------------------------------------------------------------------------------------

void BodyPart::addActionServer(ros::NodeHandle& nh, const std::string& name)
{
    TrajectoryActionServer* as = new TrajectoryActionServer(nh, name, false);
    as->registerGoalCallback(boost::bind(&BodyPart::goalCallback, this, _1));
    as->registerCancelCallback(boost::bind(&BodyPart::cancelCallback, this, _1));
    as->start();

    action_servers_.push_back(as);
}

// ----------------------------------------------------------------------------------------------------

void BodyPart::initJoint(const std::string& name, double pos)
{
    reference_generator_.initJoint(name, 0, 0, 0, 0);
    reference_generator_.setJointState(name, pos, 0);
    robot_->setJointPosition(name, pos);
}

// ----------------------------------------------------------------------------------------------------

void BodyPart::readJointInfoFromModel(const urdf::Model& Model)
{
    const std::vector<std::string>& joint_names = reference_generator_.joint_names();
    for(unsigned int i = 0 ; i < joint_names.size(); ++i)
    {
        boost::shared_ptr<const urdf::Joint> joint = Model.getJoint(joint_names[i]);
        reference_generator_.setPositionLimits(i, joint->limits->lower, joint->limits->upper);
        reference_generator_.setMaxVelocity(i, joint->limits->velocity);
        reference_generator_.setMaxAcceleration(i, joint->limits->effort);
    }
}

// ----------------------------------------------------------------------------------------------------

void BodyPart::step(double dt)
{
    const std::vector<std::string>& joint_names = reference_generator_.joint_names();

    std::vector<double> references;
    if (!reference_generator_.calculatePositionReferences(dt, references))
        return;

    for(unsigned int i = 0 ; i < joint_names.size(); ++i)
        robot_->setJointPosition(joint_names[i], references[i]);

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
}

// ----------------------------------------------------------------------------------------------------

void BodyPart::goalCallback(TrajectoryActionServer::GoalHandle gh)
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

void BodyPart::cancelCallback(TrajectoryActionServer::GoalHandle gh)
{
    gh.setCanceled();
    reference_generator_.cancelGoal(gh.getGoalID().id);
    goal_handles_.erase(gh.getGoalID().id);
}

// ----------------------------------------------------------------------------------------------------

Amigo::Amigo(ros::NodeHandle& nh) : Robot(nh, "amigo")
{
    urdf::Model model;
    model.initParam("/amigo/robot_description");

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    body_.setRobot(this);
//    body_.addActionServer(nh, "body/joint_trajectory");
//    body_.addActionServer(nh, "torso/joint_trajectory");
//    body_.addActionServer(nh, "left_arm/joint_trajectory");
//    body_.addActionServer(nh, "right_arm/joint_trajectory");

    body_.addActionServer(nh, "body/joint_trajectory_action");
//    body_.addActionServer(nh, "torso/joint_trajectory_action");
//    body_.addActionServer(nh, "left_arm/joint_trajectory_action");
//    body_.addActionServer(nh, "right_arm/joint_trajectory_action");

    //body_.initJoint("manipulation_joint", 0.0); // As it should 
    body_.initJoint("manipulation_joint", 1.57); // For testing... ToDo: replace by previous line 
    body_.initJoint("torso_joint", 0.35);
    body_.initJoint("shoulder_yaw_joint_left", -0.01);
    body_.initJoint("shoulder_pitch_joint_left", -0.4);
    body_.initJoint("shoulder_roll_joint_left", 0);
    body_.initJoint("elbow_pitch_joint_left", 1.2);
    body_.initJoint("elbow_roll_joint_left", 0);
    body_.initJoint("wrist_yaw_joint_left", 0);
    body_.initJoint("wrist_pitch_joint_left", 0.8);
    body_.initJoint("shoulder_yaw_joint_right", -0.01);
    body_.initJoint("shoulder_pitch_joint_right", -0.4);
    body_.initJoint("shoulder_roll_joint_right", 0);
    body_.initJoint("elbow_pitch_joint_right", 1.2);
    body_.initJoint("elbow_roll_joint_right", 0);
    body_.initJoint("wrist_yaw_joint_right", 0);
    body_.initJoint("wrist_pitch_joint_right", 0.8);
    body_.readJointInfoFromModel(model);

    setJointPosition("finger1_joint_left", 0.6);
    setJointPosition("finger2_joint_left", 0.6);
    setJointPosition("finger1_tip_joint_left", -0.2);
    setJointPosition("finger2_tip_joint_left", -0.2);

    setJointPosition("finger1_joint_right", 0.6);
    setJointPosition("finger2_joint_right", 0.6);
    setJointPosition("finger1_tip_joint_right", -0.2);
    setJointPosition("finger2_tip_joint_right", -0.2);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    setJointPosition("neck_pan_joint", 0);
    setJointPosition("neck_tilt_joint", 0);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    pub_body_ = nh.advertise<sensor_msgs::JointState>("/amigo/body/measurements", 10);
    pub_head_ = nh.advertise<sensor_msgs::JointState>("/amigo/neck/measurements", 10);
    pub_left_arm_ = nh.advertise<sensor_msgs::JointState>("/amigo/left_arm/measurements", 10);
    pub_right_arm_ = nh.advertise<sensor_msgs::JointState>("/amigo/right_arm/measurements", 10);
    pub_torso_ = nh.advertise<sensor_msgs::JointState>("/amigo/torso/measurements", 10);
    pub_left_gripper_ = nh.advertise<tue_msgs::GripperMeasurement>("/amigo/left_arm/gripper/measurements", 10);
    pub_right_gripper_ = nh.advertise<tue_msgs::GripperMeasurement>("/amigo/right_arm/gripper/measurements", 10);
    pub_odom_ = nh.advertise<nav_msgs::Odometry>("/amigo/base/measurements", 10);

    // SUBSCRIBERS

    sub_init_pose = nh.subscribe("/amigo/initialpose", 10, &Amigo::callbackInitialPose, this);
    sub_cmd_vel = nh.subscribe("/amigo/base/references", 10, &Amigo::callbackCmdVel, this);
    sub_head = nh.subscribe("/amigo/neck/references", 10, &Amigo::callbackJointReference, this);
    sub_spindle = nh.subscribe("/amigo/torso/references", 10, &Amigo::callbackJointReference, this);
    sub_left_arm = nh.subscribe("/amigo/left_arm/references", 10, &Amigo::callbackJointReference, this);
    sub_right_arm = nh.subscribe("/amigo/right_arm/references", 10, &Amigo::callbackJointReference, this);

    left_gripper_direction_ = tue_msgs::GripperMeasurement::OPEN;
    right_gripper_direction_ = tue_msgs::GripperMeasurement::OPEN;

    sub_left_gripper = nh.subscribe("/amigo/left_arm/gripper/references", 10, &Amigo::callbackLeftGripper, this);
    sub_right_gripper = nh.subscribe("/amigo/right_arm/gripper/references", 10, &Amigo::callbackRightGripper, this);

    tf_odom_to_base_link.frame_id_ = "/amigo/odom";
    tf_odom_to_base_link.child_frame_id_ = "/amigo/manipulation_root";

    event_odom_pub_.scheduleRecurring(50);
    event_refs_pub_.scheduleRecurring(100);
}

// ----------------------------------------------------------------------------------------------------

Amigo::~Amigo() {

}

// ----------------------------------------------------------------------------------------------------

void Amigo::step(double dt)
{
    body_.step(dt);

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
        geo::convert(getAbsolutePose(), tf_odom_to_base_link);
        tf_broadcaster_.sendTransform(tf_odom_to_base_link);

        /// Send odom message
        // Fill pose
        nav_msgs::Odometry odom_msg;
        odom_msg.header.frame_id = "/amigo/odom";
        odom_msg.header.stamp    = ros::Time::now();
        odom_msg.child_frame_id  = "/amigo/manipulation_root";
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

void Amigo::callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg) {
    this->velocity_ = *msg;
    t_last_cmd_vel_ = ros::Time::now();
}

// ----------------------------------------------------------------------------------------------------

void Amigo::callbackLeftGripper(const tue_msgs::GripperCommand::ConstPtr& msg) {
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

// ----------------------------------------------------------------------------------------------------

void Amigo::callbackRightGripper(const tue_msgs::GripperCommand::ConstPtr& msg) {
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

// ----------------------------------------------------------------------------------------------------

void Amigo::callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    geo::Transform pose;
    geo::convert(msg->pose.pose, pose);
    setPose(pose);
}

// ----------------------------------------------------------------------------------------------------

void Amigo::callbackJointReference(const sensor_msgs::JointState::ConstPtr msg) {
    for(unsigned int i = 0; i < msg->name.size(); ++i) {
        setJointReference(msg->name[i], msg->position[i]);
    }
}

// ----------------------------------------------------------------------------------------------------

void Amigo::publishControlRefs()
{
    std_msgs::Header header;
    header.stamp = ros::Time::now();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    sensor_msgs::JointState body_meas_msg;
    body_meas_msg.header = header;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    for(unsigned int j = 0; j < body_.joint_names().size(); ++j)
    {
        const std::string& joint_name = body_.joint_names()[j];
        body_meas_msg.name.push_back(joint_name);
        body_meas_msg.position.push_back(getJointPosition(joint_name));
    }

    pub_body_.publish(body_meas_msg);
    pub_left_arm_.publish(body_meas_msg);
    pub_right_arm_.publish(body_meas_msg);
    pub_torso_.publish(body_meas_msg);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    sensor_msgs::JointState head_meas_msg;
    head_meas_msg.header = header;
    head_meas_msg.name.push_back("neck_pan_joint");
    head_meas_msg.name.push_back("neck_tilt_joint");
    head_meas_msg.position.push_back(getJointPosition("neck_pan_joint"));
    head_meas_msg.position.push_back(getJointPosition("neck_tilt_joint"));
    pub_head_.publish(head_meas_msg);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    sensor_msgs::JointState torso_meas_msg;
    torso_meas_msg.header = header;
    torso_meas_msg.name.push_back("torso_joint");
    torso_meas_msg.position.push_back(getJointPosition("torso_joint"));
    pub_torso_.publish(torso_meas_msg);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    tue_msgs::GripperMeasurement left_gripper;
    left_gripper.direction = left_gripper_direction_;
    if ((left_gripper.direction == tue_msgs::GripperMeasurement::CLOSE && getJointPosition("finger1_tip_joint_left") < -0.58)
            || (left_gripper.direction == tue_msgs::GripperMeasurement::OPEN && getJointPosition("finger1_joint_left") > 0.58)) {
        left_gripper.end_position_reached = true;
    }
    pub_left_gripper_.publish(left_gripper);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    tue_msgs::GripperMeasurement right_gripper;
    right_gripper.direction = right_gripper_direction_;
    if ((right_gripper.direction == tue_msgs::GripperMeasurement::CLOSE && getJointPosition("finger1_tip_joint_right") < -0.58)
            || (right_gripper.direction == tue_msgs::GripperMeasurement::OPEN && getJointPosition("finger1_joint_right") > 0.58)) {
        right_gripper.end_position_reached = true;
    }
    pub_right_gripper_.publish(right_gripper);
}

// ----------------------------------------------------------------------------------------------------
