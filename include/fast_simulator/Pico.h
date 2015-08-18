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

#include "fast_simulator/Robot.h"

class Pico : public Robot {

public:

    Pico(ros::NodeHandle& nh);

    virtual ~Pico();

    void step(double dt);

protected:

    tf::StampedTransform tf_localization_;
    tf::StampedTransform tf_odom_to_base_link;

    ros::Time t_last_cmd_vel_;

    ros::Publisher pub_head_pan_;
    ros::Publisher pub_head_tilt_;

    ros::Subscriber sub_cmd_vel;

    ros::Subscriber sub_init_pose;

    ros::Subscriber sub_head_pan_;
    ros::Subscriber sub_head_tilt_;

    void callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg);

    void callbackHeadPan(const std_msgs::Float64::ConstPtr& msg);

    void callbackHeadTilt(const std_msgs::Float64::ConstPtr& msg);

    void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    void publishControlRefs();

    Event event_odom_pub_;

};

#endif
