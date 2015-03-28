#include "fast_simulator/Amigo.h"

#include <geolib/ros/msg_conversions.h>
#include <geolib/ros/tf_conversions.h>

using namespace std;

Amigo::Amigo(ros::NodeHandle& nh, bool publish_localization) : Robot(nh, "amigo", publish_localization) {

    setJointPosition("torso_joint", 0.351846521684684);
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

    joint_names.push_back("torso_joint");

    joint_names.push_back("shoulder_yaw_joint_left");
    joint_names.push_back("shoulder_pitch_joint_left");
    joint_names.push_back("shoulder_roll_joint_left");
    joint_names.push_back("elbow_pitch_joint_left");
    joint_names.push_back("elbow_roll_joint_left");
    joint_names.push_back("wrist_pitch_joint_left");
    joint_names.push_back("wrist_yaw_joint_left");

    joint_names.push_back("shoulder_yaw_joint_right");
    joint_names.push_back("shoulder_pitch_joint_right");
    joint_names.push_back("shoulder_roll_joint_right");
    joint_names.push_back("elbow_pitch_joint_right");
    joint_names.push_back("elbow_roll_joint_right");
    joint_names.push_back("wrist_pitch_joint_right");
    joint_names.push_back("wrist_yaw_joint_right");

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
    tf_odom_to_base_link.child_frame_id_ = "/amigo/base_link";

    event_odom_pub_.scheduleRecurring(50);
    event_refs_pub_.scheduleRecurring(100);

    // Get the constraints for each joint from the URDF model
    urdf::Model Model;
    Model.initParam("/amigo/robot_description");

    for (size_t i = 0; i < joint_names.size(); ++i)
    {
        boost::shared_ptr<const urdf::Joint> Joint = Model.getJoint(joint_names[i]);
        joint_min_constraints[joint_names[i]] = Joint->limits->lower;
        joint_max_constraints[joint_names[i]] = Joint->limits->upper;
    }

    // Setup action server
    as_ = new TrajectoryActionServer(nh,"/amigo/joint_trajectory_action",false);
    as_->registerGoalCallback(boost::bind(&Amigo::goalCallback, this, _1));
    as_->registerCancelCallback(boost::bind(&Amigo::cancelCallback, this, _1));

    // Start action server
    as_->start();
}

Amigo::~Amigo() {

}

void Amigo::step(double dt)
{
    if (!goal_handles_.empty())
    {       
        TrajectoryInfo& t_info = goal_handles_.front();

        TrajectoryActionServer::GoalHandle& gh = t_info.goal_handle;

        double t_end = t_info.time + dt;

        const std::vector<trajectory_msgs::JointTrajectoryPoint>& points = gh.getGoal()->trajectory.points;

        //std::cout << "Trajectory with " << points.size() << " points" << std::endl;

        int new_index = -1;
        for(int i = t_info.index + 1; i < (int)points.size(); ++i)
        {
            //std::cout << "    checking index " << i << std::endl;

            const trajectory_msgs::JointTrajectoryPoint& p = points[i];

            //std::cout << "        " << p.time_from_start.toSec() << " / " << t_end << std::endl;

            if (p.time_from_start.toSec() > t_end)
                break;

            new_index = i;
        }

        if (new_index > t_info.index)
        {
            //std::cout << "    new point!" << std::endl;

            const trajectory_msgs::JointTrajectoryPoint& p = points[new_index];

            // Send point to 'controller'
            const std::vector<std::string>& joint_names = gh.getGoal()->trajectory.joint_names;

            //std::cout << joint_names.size() << " " << p.positions.size() << std::endl;

            for(unsigned int j = 0; j < joint_names.size(); ++j)
            {
                //std::cout << "Joint " << joint_names[j] << " to " << p.positions[j] << std::endl;
                this->setJointPosition(joint_names[j], p.positions[j]);
            }

            // Progress index
            t_info.index = new_index;
        }

        // Progress time
        t_info.time += dt;

        // Check if this was the last point. If so, remove the goal handle
        if (new_index + 1 == points.size())
        {
            gh.setSucceeded();
            goal_handles_.erase(goal_handles_.begin());
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
        tf_odom_to_base_link.stamp_ = ros::Time::now();
        geo::convert(getAbsolutePose(), tf_odom_to_base_link);
        tf_broadcaster_.sendTransform(tf_odom_to_base_link);

        /// Send odom message
        // Fill pose
        nav_msgs::Odometry odom_msg;
        odom_msg.header.frame_id = "/amigo/odom";
        odom_msg.header.stamp    = ros::Time::now();
        odom_msg.child_frame_id  = "/amigo/base_link";
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

void Amigo::callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg) {
    this->velocity_ = *msg;
    t_last_cmd_vel_ = ros::Time::now();
}

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

void Amigo::callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    geo::Transform pose;
    geo::convert(msg->pose.pose, pose);
    setPose(pose);
}

void Amigo::callbackJointReference(const sensor_msgs::JointState::ConstPtr msg) {
    for(unsigned int i = 0; i < msg->name.size(); ++i) {
        setJointReference(msg->name[i], msg->position[i]);
    }
}

void Amigo::goalCallback(TrajectoryActionServer::GoalHandle gh)
{
    bool valid_goal = true;
    int number_of_goal_joints = gh.getGoal()->trajectory.joint_names.size();

    // Check feasibility of arm joint goals
    for (uint i = 0; i < number_of_goal_joints; i++) {
        std::string joint_name = gh.getGoal()->trajectory.joint_names[i];
        if (std::find(joint_names.begin(), joint_names.end(), joint_name) == joint_names.end()) {
            ROS_ERROR("Unknown joint %s",joint_name.c_str());
            valid_goal = false;
        }
        else {
            for (uint j = 0; j < gh.getGoal()->trajectory.points.size(); j++) {
                double ref = gh.getGoal()->trajectory.points[j].positions[i];
                if (ref < joint_min_constraints[joint_name] || ref > joint_max_constraints[joint_name]) {
                    ROS_ERROR("Reference for joint %s is %f but should be between %f and %f.",joint_name.c_str(),ref,joint_min_constraints[joint_name],joint_max_constraints[joint_name]);
                    valid_goal = false;
                }
            }
        }
    }
    if (!valid_goal) {
        gh.setRejected();
        return;
    }

    // Accept the goal
    gh.setAccepted();

    TrajectoryInfo t_info;
    t_info.goal_handle = gh;

    // Push back goal handle
    goal_handles_.push_back(t_info);
}

void Amigo::cancelCallback(TrajectoryActionServer::GoalHandle gh)
{
    // Find the goalhandle in the goal_handles_ vector
    for(std::vector<TrajectoryInfo>::iterator it = goal_handles_.begin(); it != goal_handles_.end(); ++it)
    {
        if (gh.getGoalID().id == it->goal_handle.getGoalID().id)
        {
            it->goal_handle.setCanceled();
            it = goal_handles_.erase(it);
            return;
        }
    }
}

void Amigo::publishControlRefs() {
    std_msgs::Header header;
    header.stamp = ros::Time::now();

    sensor_msgs::JointState body_meas_msg;
    body_meas_msg.header = header;
    body_meas_msg.name.push_back("torso_joint");
    body_meas_msg.position.push_back(getJointPosition("torso_joint"));
    for(unsigned int j = 0; j < left_arm_joint_names.size(); ++j) {
        body_meas_msg.name.push_back(left_arm_joint_names[j]);
        body_meas_msg.position.push_back(getJointPosition(left_arm_joint_names[j]));
    }
    for(unsigned int j = 0; j < right_arm_joint_names.size(); ++j) {
        body_meas_msg.name.push_back(right_arm_joint_names[j]);
        body_meas_msg.position.push_back(getJointPosition(left_arm_joint_names[j]));
    }
    pub_head_.publish(body_meas_msg);

    sensor_msgs::JointState head_meas_msg;
    head_meas_msg.header = header;
    head_meas_msg.name.push_back("neck_pan_joint");
    head_meas_msg.name.push_back("neck_tilt_joint");
    head_meas_msg.position.push_back(getJointPosition("neck_pan_joint"));
    head_meas_msg.position.push_back(getJointPosition("neck_tilt_joint"));
    pub_head_.publish(head_meas_msg);

    sensor_msgs::JointState torso_meas_msg;
    torso_meas_msg.header = header;
    torso_meas_msg.name.push_back("torso_joint");
    torso_meas_msg.position.push_back(getJointPosition("torso_joint"));
    pub_torso_.publish(torso_meas_msg);

    sensor_msgs::JointState left_arm_joints;
    left_arm_joints.header = header;
    for(unsigned int j = 0; j < left_arm_joint_names.size(); ++j) {
        left_arm_joints.name.push_back(left_arm_joint_names[j]);
        left_arm_joints.position.push_back(getJointPosition(left_arm_joint_names[j]));
    }
    pub_left_arm_.publish(left_arm_joints);

    sensor_msgs::JointState right_arm_joints;
    right_arm_joints.header = header;
    for(unsigned int j = 0; j < right_arm_joint_names.size(); ++j) {
        right_arm_joints.name.push_back(right_arm_joint_names[j]);
        right_arm_joints.position.push_back(getJointPosition(right_arm_joint_names[j]));
    }
    pub_right_arm_.publish(right_arm_joints);

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
