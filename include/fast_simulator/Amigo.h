#ifndef _FAST_SIMULATOR_AMIGO_H_
#define _FAST_SIMULATOR_AMIGO_H_

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "amigo_msgs/spindle_setpoint.h"
#include "amigo_msgs/head_ref.h"
#include "amigo_msgs/arm_joints.h"
#include "amigo_msgs/AmigoGripperCommand.h"
#include "amigo_msgs/AmigoGripperMeasurement.h"
#include "std_msgs/Float64.h"

#include "fast_simulator/Joint.h"
#include "fast_simulator/Object.h"
#include "fast_simulator/World.h"
#include "fast_simulator/LRF.h"
#include "fast_simulator/Kinect.h"

class Amigo : public Object {

public:

    Amigo(ros::NodeHandle& nh, bool publish_localization = true);

    virtual ~Amigo();

    void step(double dt);

    void addSensor(Sensor *sensor, const tf::Transform& rel_pose);

protected:

    ros::NodeHandle& nh_;

    bool publish_localization_;

    tf::StampedTransform tf_map_to_odom;
    tf::StampedTransform tf_odom_to_base_link;

    ros::Time t_last_cmd_vel_;

    ros::Publisher pub_head_pan_;
    ros::Publisher pub_head_tilt_;

    ros::Publisher pub_left_arm_;
    ros::Publisher pub_right_arm_;

    ros::Publisher pub_spindle_;

    ros::Publisher pub_left_gripper_;
    ros::Publisher pub_right_gripper_;

    ros::Publisher pub_joint_states;

    ros::Subscriber sub_cmd_vel;

    ros::Subscriber sub_init_pose;

    ros::Subscriber sub_spindle;

    ros::Subscriber sub_head;

    ros::Subscriber sub_left_arm;
    ros::Subscriber sub_right_arm;

    ros::Subscriber sub_left_gripper;
    ros::Subscriber sub_right_gripper;

    int left_gripper_direction_;
    int right_gripper_direction_;

    tf::TransformBroadcaster tf_broadcaster_;

    //sensor_msgs::JointState joint_states;

    std::map<std::string, Joint*> joints_;

    std::vector<std::string> left_arm_joint_names;
    std::vector<std::string> right_arm_joint_names;

    std::vector<Sensor*> sensors_;

    void setJointReference(const std::string& joint_name, double position);

    double getJointPosition(const std::string& joint_name);

    void callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg);

    void callbackLeftGripper(const amigo_msgs::AmigoGripperCommand::ConstPtr& msg);

    void callbackRightGripper(const amigo_msgs::AmigoGripperCommand::ConstPtr& msg);

    void callbackSpindleSetpoint(const amigo_msgs::spindle_setpoint::ConstPtr& msg);

    void callbackHeadPanTilt(const amigo_msgs::head_ref::ConstPtr& msg);

    void callbackLeftArm(const amigo_msgs::arm_joints::ConstPtr& msg);

    void callbackRightArm(const amigo_msgs::arm_joints::ConstPtr& msg);

    void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    void publishControlRefs();

    sensor_msgs::JointState getJointStates();

    long count_;


};

#endif
