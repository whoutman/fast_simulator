#include "fast_simulator/Robot.h"

#include <tf_conversions/tf_kdl.h>

using namespace std;

Robot::Robot(ros::NodeHandle& nh, bool publish_localization) : nh_(nh), publish_localization_(publish_localization) {
    // joint_states
    pub_joint_states = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    // TF
    tf_map_to_odom.setOrigin(tf::Vector3(0, 0, 0));
    tf_map_to_odom.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_map_to_odom.frame_id_ = "/map";
    tf_map_to_odom.child_frame_id_ = "/odom";

    event_loc_pub_.scheduleRecurring(100);
    event_joint_states_pub_.scheduleRecurring(100);
    event_sensors_pub_.scheduleRecurring(10);

    // gets the location of the robot description on the parameter server
    if (!kdl_parser::treeFromParam("/robot_description", tree)){
      ROS_ERROR("Failed to extract kdl tree from xml robot description");
    }
    addChildren(*this, tree.getRootSegment());

}

Robot::~Robot() {
    for(vector<Sensor*>::iterator it_sensor = sensors_.begin(); it_sensor != sensors_.end(); ++it_sensor) {
        delete *it_sensor;
    }
}

void Robot::addChildren(Object& obj, const KDL::SegmentMap::const_iterator segment) {
    //const std::string& root = segment->second.segment.getName();

    const std::vector<KDL::SegmentMap::const_iterator>& children = segment->second.children;
    for (unsigned int i=0; i<children.size(); i++){
        const KDL::Segment& child_kdl = children[i]->second.segment;

        tf::Transform rel_pose;
        tf::TransformKDLToTF(child_kdl.pose(0), rel_pose);

        Object* child = new Object("robot_link");
        obj.addChild(child, rel_pose);

        links_[child_kdl.getName()] = child;

        joint_to_link_[child_kdl.getJoint().getName()] = child;

        // ALMOST THERE!
        // Just update the pose of a link whenever a joints position is set

        /*
        SegmentPair s(children[i]->second.segment, root, child.getName());
        if (child.getJoint().getType() == KDL::Joint::None){
            segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
            ROS_DEBUG("Adding fixed segment from %s to %s", root.c_str(), child.getName().c_str());
        }
        else{
            segments_.insert(make_pair(child.getJoint().getName(), s));
            ROS_DEBUG("Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
        }*/

        cout << child_kdl.getName() << endl;


        addChildren(*child, children[i]);
    }
}

void Robot::step(double dt) {
    Object::step(dt);

    for(map<string, Joint*>::iterator it_joint = joints_.begin(); it_joint != joints_.end(); ++it_joint) {
        Joint* joint = it_joint->second;
        joint->step(dt);
    }

    if (event_loc_pub_.isScheduled()) {
        if (publish_localization_) {
            tf_map_to_odom.stamp_ = ros::Time::now();
            tf_broadcaster_.sendTransform(tf_map_to_odom);
        }
    }

    if (event_joint_states_pub_.isScheduled()) {
        sensor_msgs::JointState joint_states = getJointStates();
        pub_joint_states.publish(joint_states);
    }

    if (event_sensors_pub_.isScheduled()) {
        for(vector<Sensor*>::iterator it_sensor = sensors_.begin(); it_sensor != sensors_.end(); ++it_sensor) {
            Sensor* sensor = *it_sensor;
            sensor->publish();
        }
    }
}

void Robot::addSensor(Sensor* sensor, const tf::Transform& rel_pose) {
    this->addChild(sensor, rel_pose);
    sensors_.push_back(sensor);
}

void Robot::setJointPosition(const string& joint_name, double position) {
    map<std::string, Joint*>::iterator it_jnt = joints_.find(joint_name);
    if (it_jnt == joints_.end()) {
        joints_[joint_name] = new Joint(position);
    } else {
        it_jnt->second->position_ = position;
    }
}

void Robot::setJointReference(const string& joint_name, double position) {
    joints_[joint_name]->reference_ = position;
}

double Robot::getJointPosition(const string& joint_name) {
    return joints_[joint_name]->position_;
}

sensor_msgs::JointState Robot::getJointStates() {
    sensor_msgs::JointState joint_states;
    joint_states.header.stamp = ros::Time::now();

    for(map<string, Joint*>::iterator it_joint = joints_.begin(); it_joint != joints_.end(); ++it_joint) {
        Joint* joint = it_joint->second;
        joint_states.name.push_back(it_joint->first);
        joint_states.position.push_back(joint->position_);
    }

    return joint_states;
}
