#ifndef _FAST_SIMULATOR_AMIGO_H_
#define _FAST_SIMULATOR_AMIGO_H_

#include "tue_msgs/GripperCommand.h"
#include "tue_msgs/GripperMeasurement.h"
#include <trajectory_msgs/JointTrajectory.h> // Delete
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "fast_simulator/Robot.h"

typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> TrajectoryActionServer;

class Amigo : public Robot {    

public:

    Amigo(ros::NodeHandle& nh, bool publish_localization = true);

    virtual ~Amigo();

    void step(double dt);

protected:

    tf::StampedTransform tf_odom_to_base_link;

    ros::Time t_last_cmd_vel_;

    ros::Publisher pub_head_;
    ros::Publisher pub_left_arm_;
    ros::Publisher pub_right_arm_;
    ros::Publisher pub_torso_;
    ros::Publisher pub_left_gripper_;
    ros::Publisher pub_right_gripper_;
    ros::Publisher pub_odom_;

    ros::Subscriber sub_cmd_vel;
    ros::Subscriber sub_init_pose;
    ros::Subscriber sub_head;
    ros::Subscriber sub_spindle, sub_spindle_traj_;
    ros::Subscriber sub_right_arm, sub_right_arm_traj_;

    ros::Subscriber sub_left_gripper;
    ros::Subscriber sub_right_gripper;

    int left_gripper_direction_;
    int right_gripper_direction_;

    std::vector<std::string> left_arm_joint_names;
    std::vector<std::string> right_arm_joint_names;
    std::vector<std::string> joint_names;

    std::map<std::string,double> intermediate_goal_constraints;
    std::map<std::string,double> final_goal_constraints;
    std::map<std::string,double> trajectory_constraints;
    std::map<std::string,double> joint_min_constraints;
    std::map<std::string,double> joint_max_constraints;


    std::map<std::string, Trajectory> joint_trajectories_;

    void callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg);

    void callbackLeftGripper(const tue_msgs::GripperCommand::ConstPtr& msg);

    void callbackRightGripper(const tue_msgs::GripperCommand::ConstPtr& msg);    

    void callbackJointReference(const sensor_msgs::JointState::ConstPtr msg);

    void callbackJointTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr msg);

    void goalCallback(TrajectoryActionServer::GoalHandle gh);
    void cancelCallback(TrajectoryActionServer::GoalHandle gh);
    TrajectoryActionServer* as_;
    std::vector < TrajectoryActionServer::GoalHandle > goal_handles_;

    void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    void publishControlRefs();

    Event event_odom_pub_;
    Event event_refs_pub_;

};

#endif
