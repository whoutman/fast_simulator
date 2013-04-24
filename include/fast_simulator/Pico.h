#ifndef _FAST_SIMULATOR_PICO_H_
#define _FAST_SIMULATOR_PICO_H_

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "std_msgs/Float64.h"

#include "fast_simulator/Joint.h"
#include "fast_simulator/Object.h"
#include "fast_simulator/World.h"
#include "fast_simulator/LRF.h"
#include "fast_simulator/Sonar.h"

class Pico : public Object {

public:

    Pico(ros::NodeHandle& nh, bool publish_localization = true);

    virtual ~Pico();

    void step(double dt);

protected:

    ros::NodeHandle& nh_;

    bool publish_localization_;

    tf::StampedTransform tf_map_to_odom;
    tf::StampedTransform tf_odom_to_base_link;

    ros::Time t_last_cmd_vel_;

    ros::Publisher pub_head_pan_;
    ros::Publisher pub_head_tilt_;

    ros::Publisher pub_joint_states;

    ros::Subscriber sub_cmd_vel;

    ros::Subscriber sub_init_pose;

    ros::Subscriber sub_head_pan_;
    ros::Subscriber sub_head_tilt_;

    tf::TransformBroadcaster tf_broadcaster_;

    std::map<std::string, Joint*> joints_;

    LRF* laser_range_finder_;

    Sonar* front_sonar_;

    void setJointReference(const std::string& joint_name, double position);

    double getJointPosition(const std::string& joint_name);

    void callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg);

    void callbackHeadPan(const std_msgs::Float64::ConstPtr& msg);

    void callbackHeadTilt(const std_msgs::Float64::ConstPtr& msg);

    void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    void publishControlRefs();

    sensor_msgs::JointState getJointStates();

    long count_;

};

#endif
