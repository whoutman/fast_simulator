#ifndef _FAST_SIMULATOR_PERA_H_
#define _FAST_SIMULATOR_PERA_H_

#include "tue_msgs/GripperCommand.h"
#include "tue_msgs/GripperMeasurement.h"

#include "sensor_msgs/JointState.h"

#include "fast_simulator/Robot.h"

class Pera : public Robot {

public:

    Pera(ros::NodeHandle& nh);

    virtual ~Pera();

    void step(double dt);

protected:

    ros::Publisher pub_left_arm_;
    ros::Publisher pub_left_gripper_;
    ros::Subscriber sub_left_arm;
    ros::Subscriber sub_left_gripper;

    int left_gripper_direction_;

    Event event_loc_pub_;
    tf::StampedTransform tf_location_;

    std::vector<std::string> left_arm_joint_names;

    void callbackLeftGripper(const tue_msgs::GripperCommand::ConstPtr& msg);

    void callbackLeftArm(const sensor_msgs::JointState::ConstPtr& msg);

    void publishControlRefs();

    Event event_refs_pub_;

};

#endif
