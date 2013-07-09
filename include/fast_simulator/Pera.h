#ifndef _FAST_SIMULATOR_PERA_H_
#define _FAST_SIMULATOR_PERA_H_

#include "amigo_msgs/arm_joints.h"
#include "amigo_msgs/AmigoGripperCommand.h"
#include "amigo_msgs/AmigoGripperMeasurement.h"
#include "std_msgs/Float64.h"

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

    void callbackLeftGripper(const amigo_msgs::AmigoGripperCommand::ConstPtr& msg);

    void callbackLeftArm(const amigo_msgs::arm_joints::ConstPtr& msg);

    void publishControlRefs();

    Event event_refs_pub_;

};

#endif
