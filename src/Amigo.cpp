#include "fast_simulator/Amigo.h"

using namespace std;

Amigo::Amigo(ros::NodeHandle& nh, bool publish_localization) : nh_(nh), publish_localization_(publish_localization) {
    // joint_states
    pub_joint_states = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    joints_["spindle_joint"] = new Joint(0.351846521684684, 0.1);
    joints_["shoulder_yaw_joint_left"] = new Joint(-0.010038043598955326);
    joints_["shoulder_pitch_joint_left"] = new Joint(-0.39997462399515005);
    joints_["shoulder_roll_joint_left"] = new Joint(2.0889754646091774e-06);
    joints_["elbow_pitch_joint_left"] = new Joint(1.1999600775244508);
    joints_["elbow_roll_joint_left"] = new Joint(4.330400908969523e-07);
    joints_["wrist_yaw_joint_left"] = new Joint(1.7639288287796262e-06);
    joints_["wrist_pitch_joint_left"] = new Joint(0.7999636309384188);
    joints_["finger1_joint_left"] = new Joint(0.6000396498469334);
    joints_["finger2_joint_left"] = new Joint(0.6000510112333535);
    joints_["finger1_tip_joint_left"] = new Joint(-0.20000170842812892);
    joints_["finger2_tip_joint_left"] = new Joint(-0.1999984035736233);
    joints_["shoulder_yaw_joint_right"] = new Joint(-0.01004552338080078);
    joints_["shoulder_pitch_joint_right"] = new Joint(-0.39998108009563715);
    joints_["shoulder_roll_joint_right"] = new Joint(9.422008346859911e-06);
    joints_["elbow_pitch_joint_right"] = new Joint(1.1999679974909059);
    joints_["elbow_roll_joint_right"] = new Joint(2.5800741013881634e-05);
    joints_["wrist_yaw_joint_right"] = new Joint(4.025142829355843e-05);
    joints_["wrist_pitch_joint_right"] = new Joint(0.7999828748945985);
    joints_["finger1_joint_right"] = new Joint(0.6000464445187825);
    joints_["finger2_joint_right"] = new Joint(0.6000300525013822);
    joints_["finger1_tip_joint_right"] = new Joint(-0.1999953219398023);
    joints_["finger2_tip_joint_right"] = new Joint(-0.20000480019727807);
    joints_["base_phi_joint"] = new Joint(-1.9961446717786657e-07);
    joints_["base_x_joint"] = new Joint(-3.771623482909497e-05);
    joints_["base_y_joint"] = new Joint(-7.877193603413587e-08);
    joints_["neck_pan_joint"] = new Joint(-3.033573445776483e-07);
    joints_["neck_tilt_joint"] = new Joint(0.00029286782768789266);

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

    pub_head_pan_ = nh.advertise<std_msgs::Float64>("/head_pan_angle", 10);
    pub_head_tilt_ = nh.advertise<std_msgs::Float64>("/head_tilt_angle", 10);

    pub_left_arm_ = nh.advertise<amigo_msgs::arm_joints>("/arm_left_controller/joint_measurements", 10);
    pub_right_arm_ = nh.advertise<amigo_msgs::arm_joints>("/arm_right_controller/joint_measurements", 10);

    pub_spindle_ = nh.advertise<std_msgs::Float64>("/spindle_position", 10);

    pub_left_gripper_ = nh.advertise<amigo_msgs::AmigoGripperMeasurement>("/arm_left_controller/gripper_measurement", 10);
    pub_right_gripper_ = nh.advertise<amigo_msgs::AmigoGripperMeasurement>("/arm_right_controller/gripper_measurement", 10);

    tf::Transform tf_base_link_to_front_laser;
    tf_base_link_to_front_laser.setOrigin(tf::Vector3(0.31, 0, 0.3));
    tf_base_link_to_front_laser.setRotation(tf::Quaternion(0, 0, 0, 1));
    laser_range_finder_ = new LRF("/base_scan", "/front_laser");
    this->addChild(laser_range_finder_, tf_base_link_to_front_laser);

    // SUBSCRIBERS

    // cmd_vel
    sub_cmd_vel = nh.subscribe("/cmd_vel", 10, &Amigo::callbackCmdVel, this);

    sub_init_pose = nh.subscribe("/initialpose", 10, &Amigo::callbackInitialPose, this);

    sub_spindle = nh.subscribe("/spindle_controller/spindle_coordinates", 10, &Amigo::callbackSpindleSetpoint, this);

    sub_head = nh.subscribe("/head_controller/set_Head", 10, &Amigo::callbackHeadPanTilt, this);

    sub_left_arm = nh.subscribe("/arm_left_controller/joint_references", 10, &Amigo::callbackLeftArm, this);

    sub_right_arm = nh.subscribe("/arm_right_controller/joint_references", 10, &Amigo::callbackRightArm, this);

    left_gripper_direction_ = amigo_msgs::AmigoGripperMeasurement::OPEN;
    right_gripper_direction_ = amigo_msgs::AmigoGripperMeasurement::OPEN;


    sub_left_gripper = nh.subscribe("/arm_left_controller/gripper_command", 10, &Amigo::callbackLeftGripper, this);

    sub_right_gripper = nh.subscribe("/arm_right_controller/gripper_command", 10, &Amigo::callbackRightGripper, this);

    // TF
    tf_map_to_odom.setOrigin(tf::Vector3(0, 0, 0));
    tf_map_to_odom.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_map_to_odom.frame_id_ = "/map";
    tf_map_to_odom.child_frame_id_ = "/odom";

    tf_odom_to_base_link.frame_id_ = "/odom";
    tf_odom_to_base_link.child_frame_id_ = "/base_link";

    count_ = 0;
}

Amigo::~Amigo() {
}

void Amigo::step(double dt) {
    Object::step(dt);

    for(map<string, Joint*>::iterator it_joint = joints_.begin(); it_joint != joints_.end(); ++it_joint) {
        Joint* joint = it_joint->second;
        joint->step(dt);
    }

    if (ros::Time::now() - t_last_cmd_vel_ > ros::Duration(0.5)) {
        this->velocity_.angular.x = 0;
        this->velocity_.angular.y = 0;
        this->velocity_.angular.z = 0;

        this->velocity_.linear.x = 0;
        this->velocity_.linear.y = 0;
        this->velocity_.linear.z = 0;
    }

    if (count_ % 4 == 0) {
        tf_odom_to_base_link.stamp_ = ros::Time::now();
        tf_odom_to_base_link.setOrigin(this->pose_.getOrigin());
        tf_odom_to_base_link.setRotation(this->pose_.getRotation());
        tf_broadcaster_.sendTransform(tf_odom_to_base_link);

        if (publish_localization_) {
            tf_map_to_odom.stamp_ = ros::Time::now();
            tf_broadcaster_.sendTransform(tf_map_to_odom);
        }
    }

    if (count_ % 4 == 0) {
        sensor_msgs::JointState joint_states = getJointStates();
        pub_joint_states.publish(joint_states);
    }

    publishControlRefs();

    if (count_ % 10 == 0) {
        laser_range_finder_->publishScan();
    }

    count_++;
}

void Amigo::setJointReference(const string& joint_name, double position) {
    joints_[joint_name]->reference_ = position;
}

double Amigo::getJointPosition(const string& joint_name) {
    return joints_[joint_name]->position_;
}

void Amigo::callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg) {
    this->velocity_ = *msg;
    t_last_cmd_vel_ = ros::Time::now();
}

void Amigo::callbackLeftGripper(const amigo_msgs::AmigoGripperCommand::ConstPtr& msg) {
    if (msg->direction == amigo_msgs::AmigoGripperCommand::CLOSE) {
        setJointReference("finger1_joint_left", 0.20);
        setJointReference("finger2_joint_left", 0.20);
        setJointReference("finger1_tip_joint_left", -0.60);
        setJointReference("finger2_tip_joint_left", -0.60);
        left_gripper_direction_ = amigo_msgs::AmigoGripperMeasurement::CLOSE;
    } else if  (msg->direction == amigo_msgs::AmigoGripperCommand::OPEN) {
        setJointReference("finger1_joint_left", 0.60);
        setJointReference("finger2_joint_left", 0.60);
        setJointReference("finger1_tip_joint_left", -0.18);
        setJointReference("finger2_tip_joint_left", -0.18);
        left_gripper_direction_ = amigo_msgs::AmigoGripperMeasurement::OPEN;
    }
}

void Amigo::callbackRightGripper(const amigo_msgs::AmigoGripperCommand::ConstPtr& msg) {
    if (msg->direction == amigo_msgs::AmigoGripperCommand::CLOSE) {
        setJointReference("finger1_joint_right", 0.20);
        setJointReference("finger2_joint_right", 0.20);
        setJointReference("finger1_tip_joint_right", -0.60);
        setJointReference("finger2_tip_joint_right", -0.60);
        right_gripper_direction_ = amigo_msgs::AmigoGripperMeasurement::CLOSE;
    } else if  (msg->direction == amigo_msgs::AmigoGripperCommand::OPEN) {
        setJointReference("finger1_joint_right", 0.60);
        setJointReference("finger2_joint_right", 0.60);
        setJointReference("finger1_tip_joint_right", -0.18);
        setJointReference("finger2_tip_joint_right", -0.18);
        right_gripper_direction_ = amigo_msgs::AmigoGripperMeasurement::OPEN;
    }
}

void Amigo::callbackSpindleSetpoint(const amigo_msgs::spindle_setpoint::ConstPtr& msg) {
    setJointReference("spindle_joint", msg->pos);
}

void Amigo::callbackHeadPanTilt(const amigo_msgs::head_ref::ConstPtr& msg) {
    setJointReference("neck_pan_joint", msg->head_pan);
    setJointReference("neck_tilt_joint", msg->head_tilt);
}

void Amigo::callbackLeftArm(const amigo_msgs::arm_joints::ConstPtr& msg) {
    for(unsigned int i = 0; i < msg->pos.size(); ++i) {
        setJointReference(left_arm_joint_names[i], msg->pos[i].data);
    }
}

void Amigo::callbackRightArm(const amigo_msgs::arm_joints::ConstPtr& msg) {
    for(unsigned int i = 0; i < msg->pos.size(); ++i) {
        setJointReference(right_arm_joint_names[i], msg->pos[i].data);
    }
}

void Amigo::callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    tf::poseMsgToTF(msg->pose.pose, this->pose_);
}

void Amigo::publishControlRefs() {
    std_msgs::Float64 f;
    f.data = getJointPosition("neck_pan_joint");
    pub_head_pan_.publish(f);

    f.data = getJointPosition("neck_tilt_joint");
    pub_head_tilt_.publish(f);

    f.data = getJointPosition("spindle_joint");
    pub_spindle_.publish(f);

    amigo_msgs::arm_joints left_arm_joints;
    for(unsigned int j = 0; j < left_arm_joint_names.size(); ++j) {
        left_arm_joints.pos[j].data = getJointPosition(left_arm_joint_names[j]);
    }
    pub_left_arm_.publish(left_arm_joints);

    amigo_msgs::arm_joints right_arm_joints;
    for(unsigned int j = 0; j < right_arm_joint_names.size(); ++j) {
        right_arm_joints.pos[j].data = getJointPosition(right_arm_joint_names[j]);
    }
    pub_right_arm_.publish(right_arm_joints);

    amigo_msgs::AmigoGripperMeasurement left_gripper;
    left_gripper.direction = left_gripper_direction_;
    if ((left_gripper.direction == amigo_msgs::AmigoGripperMeasurement::CLOSE && getJointPosition("finger1_tip_joint_left") < -0.58)
            || (left_gripper.direction == amigo_msgs::AmigoGripperMeasurement::OPEN && getJointPosition("finger1_joint_left") > 0.58)) {
        left_gripper.end_position_reached = true;
    }
    pub_left_gripper_.publish(left_gripper);

    amigo_msgs::AmigoGripperMeasurement right_gripper;
    right_gripper.direction = right_gripper_direction_;
    if ((right_gripper.direction == amigo_msgs::AmigoGripperMeasurement::CLOSE && getJointPosition("finger1_tip_joint_right") < -0.58)
            || (right_gripper.direction == amigo_msgs::AmigoGripperMeasurement::OPEN && getJointPosition("finger1_joint_right") > 0.58)) {
        right_gripper.end_position_reached = true;
    }
    pub_right_gripper_.publish(right_gripper);
}

sensor_msgs::JointState Amigo::getJointStates() {
    sensor_msgs::JointState joint_states;
    joint_states.header.stamp = ros::Time::now();

    for(map<string, Joint*>::iterator it_joint = joints_.begin(); it_joint != joints_.end(); ++it_joint) {
        Joint* joint = it_joint->second;
        joint_states.name.push_back(it_joint->first);
        joint_states.position.push_back(joint->position_);
    }

    return joint_states;
}
