#ifndef _FAST_SIMULATOR_SERGIO_H_
#define _FAST_SIMULATOR_SERGIO_H_

#include "tue_msgs/GripperCommand.h"
#include "tue_msgs/GripperMeasurement.h"
#include <trajectory_msgs/JointTrajectory.h>

#include "fast_simulator/Robot.h"

class Sergio : public Robot {

public:

    Sergio(ros::NodeHandle& nh);

    virtual ~Sergio();

    void step(double dt);

protected:

    tf::StampedTransform tf_odom_to_base_link;

    ros::Time t_last_cmd_vel_;

    ros::Publisher pub_head_;
    ros::Publisher pub_dynamixel_;
    ros::Publisher pub_left_arm_;
    ros::Publisher pub_right_arm_;
    ros::Publisher pub_torso_;
    ros::Publisher pub_left_gripper_;
    ros::Publisher pub_right_gripper_;
    ros::Publisher pub_odom_;

    ros::Subscriber sub_cmd_vel;
    ros::Subscriber sub_init_pose;
    ros::Subscriber sub_head;
    ros::Subscriber sub_dynamixel;
    ros::Subscriber sub_spindle, sub_spindle_traj_;
    ros::Subscriber sub_left_arm, sub_left_arm_traj_;
    ros::Subscriber sub_right_arm, sub_right_arm_traj_;

    ros::Subscriber sub_left_gripper;
    ros::Subscriber sub_right_gripper;

    int left_gripper_direction_;
    int right_gripper_direction_;

    std::vector<std::string> left_arm_joint_names;
    std::vector<std::string> right_arm_joint_names;
    std::vector<std::string> torso_joint_names;

    std::map<std::string, Trajectory> joint_trajectories_;

    void callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg);

    void callbackLeftGripper(const tue_msgs::GripperCommand::ConstPtr& msg);

    void callbackRightGripper(const tue_msgs::GripperCommand::ConstPtr& msg);    

    void callbackJointReference(const sensor_msgs::JointState::ConstPtr msg);

    void callbackJointTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr msg);

    void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    void publishControlRefs();

    Event event_odom_pub_;
    Event event_refs_pub_;

};

#endif
