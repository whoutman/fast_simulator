#include <ros/ros.h>

#include "fast_simulator/Object.h"
#include "fast_simulator/Joint.h"

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>



#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "amigo_msgs/spindle_setpoint.h"
#include "amigo_msgs/head_ref.h"
#include "amigo_msgs/arm_joints.h"
#include "amigo_msgs/AmigoGripperCommand.h"
#include "amigo_msgs/AmigoGripperMeasurement.h"
#include "std_msgs/Float64.h"

// TODO:
// pubish on  * /head_tilt_angle [unknown type] and * /head_pan_angle [unknown type]


using namespace std;

Object* robot_ = 0;

vector<Object*> objects_;

nav_msgs::OccupancyGrid world_map_;
tf::Transform map_transform_;

sensor_msgs::LaserScan scan;
vector<tf::Vector3> laser_ray_deltas_;

tf::StampedTransform tf_base_link_to_front_laser;
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
int left_gripper_direction_;
int right_gripper_direction_;

double laser_resolution = 0.01;

//sensor_msgs::JointState joint_states;

map<string, Joint*> joints_;

vector<string> left_arm_joint_names;
vector<string> right_arm_joint_names;

void setJointReference(const string& joint_name, double position) {
    joints_[joint_name]->reference_ = position;
}

double getJointPosition(const string& joint_name) {
    return joints_[joint_name]->position_;
}

void callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg) {
    if (robot_) {
        robot_->velocity_ = *msg;
    }
    t_last_cmd_vel_ = ros::Time::now();
}

void callbackLeftGripper(const amigo_msgs::AmigoGripperCommand::ConstPtr& msg) {
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

void callbackRightGripper(const amigo_msgs::AmigoGripperCommand::ConstPtr& msg) {
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

void callbackSpindleSetpoint(const amigo_msgs::spindle_setpoint::ConstPtr& msg) {
    setJointReference("spindle_joint", msg->pos);
}

void callbackHeadPanTilt(const amigo_msgs::head_ref::ConstPtr& msg) {
    setJointReference("neck_pan_joint", msg->head_pan);
    setJointReference("neck_tilt_joint", msg->head_tilt);
}

void callbackLeftArm(const amigo_msgs::arm_joints::ConstPtr& msg) {
    for(unsigned int i = 0; i < msg->pos.size(); ++i) {
        setJointReference(left_arm_joint_names[i], msg->pos[i].data);
    }
}

void callbackRightArm(const amigo_msgs::arm_joints::ConstPtr& msg) {
    for(unsigned int i = 0; i < msg->pos.size(); ++i) {
        setJointReference(right_arm_joint_names[i], msg->pos[i].data);
    }
}

void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (robot_) {
        tf::poseMsgToTF(msg->pose.pose, robot_->pose_);
    }
}

void callbackMap(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    world_map_ = *msg;
    tf::poseMsgToTF(world_map_.info.origin, map_transform_);
}

void publishControlRefs() {
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

void step(double dt) {
    for(vector<Object*>::iterator it_obj = objects_.begin(); it_obj != objects_.end(); ++it_obj) {
        Object* obj = *it_obj;
        obj->step(dt);
    }
    for(map<string, Joint*>::iterator it_joint = joints_.begin(); it_joint != joints_.end(); ++it_joint) {
        Joint* joint = it_joint->second;
        joint->step(dt);
    }
}

#include <sstream>
string toString(const tf::Vector3& v) {
    stringstream s;
    s << "(" << v.getX() << ", " << v.getY() << ", " << v.getZ() << ")";
    return s.str();
}

string toString(const tf::Transform& tf) {
    stringstream s;
    s << toString(tf.getOrigin()) << ", "
      << "(" << tf.getRotation().getX() << ", " << tf.getRotation().getY() << ", " << tf.getRotation().getZ() << ", " << tf.getRotation().getW() << ")";
    return s.str();
}

bool isOccupied(const tf::Vector3& pos) {

    tf::Vector3 pos_map = map_transform_.inverse() * pos;

    int mx = (int)(pos_map.getX() / world_map_.info.resolution);
    int my = (int)(pos_map.getY() / world_map_.info.resolution);

    if (mx < 0 || mx >= (int)world_map_.info.width || my < 0 || my >= (int)world_map_.info.height) {
        return false;
    }

    if (world_map_.data[world_map_.info.width * my + mx] > 10 ) {
        return true;
    }

    return false;
}

void initLaserScan() {
    scan.header.frame_id = "/front_laser";
    scan.angle_min = -2.09439492226;
    scan.angle_max = 2.09439492226;
    scan.angle_increment = 0.00614192103967;
    scan.time_increment = 0;
    scan.scan_time = 0;
    scan.range_min = 0.5;
    scan.range_max = 10.0;

    laser_ray_deltas_.clear();
    for(double angle = scan.angle_min; angle <= scan.angle_max; angle += scan.angle_increment) {
        tf::Quaternion q;
        q.setRPY(0, 0, angle);

        tf::Transform t;
        t.setOrigin(tf::Vector3(0, 0, 0));
        t.setRotation(q);

        laser_ray_deltas_.push_back(t * tf::Vector3(laser_resolution, 0, 0));

        //cout << laser_ray_deltas_.back().getX() << ", " << laser_ray_deltas_.back().getY() << ", " << laser_ray_deltas_.back().getZ() << endl;
    }

    //cout << laser_ray_deltas_.size() << endl;
}

 bool getLaserScan(sensor_msgs::LaserScan& ret_scan) {

    scan.header.stamp = ros::Time::now();

    scan.ranges.clear();
    scan.intensities.clear();

    tf::Transform tf_map_to_front_laser = tf_map_to_odom *  tf_odom_to_base_link * tf_base_link_to_front_laser;

    tf::Vector3 laser_origin = tf_map_to_front_laser.getOrigin();

    for(unsigned int i = 0; i < laser_ray_deltas_.size(); ++i) {

        tf::Vector3 delta = tf::Transform(tf_map_to_front_laser.getRotation()) * laser_ray_deltas_[i];

        tf::Vector3 v = laser_origin;
        double distance = 0;
        for(; distance < 9; distance += laser_resolution) {
            if (isOccupied(v)) {
                break;
            }
            v += delta;
        }

        scan.ranges.push_back(distance);
        scan.intensities.push_back(101);
    }

    ret_scan = scan;

    return true;
}

 sensor_msgs::JointState getJointStates() {
     sensor_msgs::JointState joint_states;
     joint_states.header.stamp = ros::Time::now();

     for(map<string, Joint*>::iterator it_joint = joints_.begin(); it_joint != joints_.end(); ++it_joint) {
         Joint* joint = it_joint->second;
         joint_states.name.push_back(it_joint->first);
         joint_states.position.push_back(joint->position_);
     }

     return joint_states;
 }

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "fast_simulator");
    ros::NodeHandle nh;

    bool publish_localization = true;

    ros::Subscriber sub_map = nh.subscribe("/fast_simulator/map", 10, &callbackMap);
    while(ros::ok() && world_map_.data.empty()) {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        ROS_INFO("Waiting for map at topic %s", sub_map.getTopic().c_str());
    }
    ROS_INFO("Map found at topic %s", sub_map.getTopic().c_str());
    sub_map.shutdown();

    // PUBLISHERS

    // joint_states
    ros::Publisher pub_joint_states = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    joints_["spindle_joint"] = new Joint(0.351846521684684);
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

    ros::Publisher pub_laser_scan = nh.advertise<sensor_msgs::LaserScan>("/base_scan", 10);

    tf_base_link_to_front_laser.setOrigin(tf::Vector3(0.31, 0, 0.3));
    tf_base_link_to_front_laser.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_base_link_to_front_laser.frame_id_ = "/base_link";
    tf_base_link_to_front_laser.child_frame_id_ = "/front_laser";

    initLaserScan();

    // SUBSCRIBERS

    // cmd_vel
    ros::Subscriber sub_cmd_vel = nh.subscribe("/cmd_vel", 10, &callbackCmdVel);

    ros::Subscriber sub_init_pose = nh.subscribe("/initialpose", 10, &callbackInitialPose);

    ros::Subscriber sub_spindle = nh.subscribe("/spindle_controller/spindle_coordinates", 10, &callbackSpindleSetpoint);

    ros::Subscriber sub_head = nh.subscribe("/head_controller/set_Head", 10, &callbackHeadPanTilt);

    ros::Subscriber sub_left_arm = nh.subscribe("/arm_left_controller/joint_references", 10, &callbackLeftArm);

    ros::Subscriber sub_right_arm = nh.subscribe("/arm_right_controller/joint_references", 10, &callbackRightArm);

    left_gripper_direction_ = amigo_msgs::AmigoGripperMeasurement::OPEN;
    right_gripper_direction_ = amigo_msgs::AmigoGripperMeasurement::OPEN;


    ros::Subscriber sub_left_gripper = nh.subscribe("/arm_left_controller/gripper_command", 10, &callbackLeftGripper);

    ros::Subscriber sub_right_gripper = nh.subscribe("/arm_right_controller/gripper_command", 10, &callbackRightGripper);

    // TF

    tf::TransformBroadcaster tf_broadcaster;

    tf_map_to_odom.setOrigin(tf::Vector3(0, 0, 0));
    tf_map_to_odom.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_map_to_odom.frame_id_ = "/map";
    tf_map_to_odom.child_frame_id_ = "/odom";

    tf_odom_to_base_link.frame_id_ = "/odom";
    tf_odom_to_base_link.child_frame_id_ = "/base_link";

    // AMIGO

    Object* amigo = new Object();
    robot_ = amigo;
    amigo->pose_.setOrigin(tf::Vector3(0, 0, 0));
    amigo->pose_.setRotation(tf::Quaternion(0, 0, 0, 1));
    objects_.push_back(amigo);

    double freq = 100;
    ros::Rate r(freq);

    long count = 0;
    while(ros::ok()) {
        ros::spinOnce();

        if (ros::Time::now() - t_last_cmd_vel_ > ros::Duration(0.5) && robot_) {
            robot_->velocity_.angular.x = 0;
            robot_->velocity_.angular.y = 0;
            robot_->velocity_.angular.z = 0;

            robot_->velocity_.linear.x = 0;
            robot_->velocity_.linear.y = 0;
            robot_->velocity_.linear.z = 0;
        }

        if (count % 4 == 0) {
            tf_odom_to_base_link.stamp_ = ros::Time::now();
            tf_odom_to_base_link.setOrigin(amigo->pose_.getOrigin());
            tf_odom_to_base_link.setRotation(amigo->pose_.getRotation());
            tf_broadcaster.sendTransform(tf_odom_to_base_link);

            if (publish_localization) {
                tf_map_to_odom.stamp_ = ros::Time::now();
                tf_broadcaster.sendTransform(tf_map_to_odom);
            }
        }

        if (count % 4 == 0) {
            sensor_msgs::JointState joint_states = getJointStates();
            pub_joint_states.publish(joint_states);
        }

        publishControlRefs();

        if (count % 10 == 0) {
            sensor_msgs::LaserScan scan ;
            if (getLaserScan(scan)) {
                pub_laser_scan.publish(scan);
            }
        }

        step(1 / freq);

        ++count;
        r.sleep();
    }

    return 0;
}
