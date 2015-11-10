#ifndef _FAST_SIMULATOR_AMIGO_H_
#define _FAST_SIMULATOR_AMIGO_H_

#include "tue_msgs/GripperCommand.h"
#include "tue_msgs/GripperMeasurement.h"
#include <trajectory_msgs/JointTrajectory.h> // Delete
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <urdf/model.h>

#include "../../include/fast_simulator/Robot.h"

#include <tue/manipulation/reference_generator.h>

typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> TrajectoryActionServer;

// ----------------------------------------------------------------------------------------------------

class Amigo;

class BodyPart
{

public:

    BodyPart();

    ~BodyPart();

    void initialize(ros::NodeHandle& nh, Amigo* robot, const std::string& action_name);

    void initJoint(const std::string& name, double pos, double max_vel, double max_acc);

    void readJointInfoFromModel(const urdf::Model& Model);

    void step(double dt);

    const std::vector<std::string>& joint_names() const { return reference_generator_.joint_names(); }

private:

    Amigo* robot_;

    tue::manipulation::ReferenceGenerator reference_generator_;

    void goalCallback(TrajectoryActionServer::GoalHandle gh);

    void cancelCallback(TrajectoryActionServer::GoalHandle gh);

    TrajectoryActionServer* as_;

    bool has_goal_;

    TrajectoryActionServer::GoalHandle goal_handle_;

};

// ----------------------------------------------------------------------------------------------------

struct TrajectoryInfo
{
    TrajectoryInfo() : time(0), index(-1) {}

    TrajectoryActionServer::GoalHandle goal_handle;
    double time;
    int index;
};

// ----------------------------------------------------------------------------------------------------

class Amigo : public Robot {    

public:

    Amigo(ros::NodeHandle& nh);

    virtual ~Amigo();

    void step(double dt);

protected:

    tf::StampedTransform tf_odom_to_base_link;

    ros::Time t_last_cmd_vel_;

    ros::Publisher pub_body_;
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
    ros::Subscriber sub_spindle;
    ros::Subscriber sub_left_arm;
    ros::Subscriber sub_right_arm;

    ros::Subscriber sub_left_gripper;
    ros::Subscriber sub_right_gripper;

    int left_gripper_direction_;
    int right_gripper_direction_;

    void callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg);

    void callbackLeftGripper(const tue_msgs::GripperCommand::ConstPtr& msg);

    void callbackRightGripper(const tue_msgs::GripperCommand::ConstPtr& msg);    

    void callbackJointReference(const sensor_msgs::JointState::ConstPtr msg);

    BodyPart left_arm_;
    BodyPart right_arm_;
    BodyPart torso_;

    void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    void publishControlRefs();

    Event event_odom_pub_;
    Event event_refs_pub_;

};

#endif
