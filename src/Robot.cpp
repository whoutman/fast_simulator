#include "fast_simulator/Robot.h"

#include <tf_conversions/tf_kdl.h>

using namespace std;

// ----------------------------------------------------------------------------------------------------

Robot::Robot(ros::NodeHandle& nh) : nh_(nh)
{

}

// ----------------------------------------------------------------------------------------------------

Robot::Robot(ros::NodeHandle& nh, const std::string& robot_type) : nh_(nh)
{
    // joint_states
    pub_joint_states = nh.advertise<sensor_msgs::JointState>("/" + robot_type + "/joint_states", 10);

    event_joint_states_pub_.scheduleRecurring(100);
    event_sensors_pub_.scheduleRecurring(10);

    // gets the location of the robot description on the parameter server
    if (!kdl_parser::treeFromParam("/" + robot_type + "/robot_description", tree)){
      ROS_ERROR("Failed to extract kdl tree from xml robot description");
    } else {
        addChildren(*this, tree.getRootSegment());
    }
}

// ----------------------------------------------------------------------------------------------------

Robot::~Robot()
{
    for(vector<Sensor*>::iterator it_sensor = sensors_.begin(); it_sensor != sensors_.end(); ++it_sensor) {
        delete *it_sensor;
    }
}

// ----------------------------------------------------------------------------------------------------

void Robot::addChildren(Object& obj, const KDL::SegmentMap::const_iterator segment)
{
    //const std::string& root = segment->second.segment.getName();

    const std::vector<KDL::SegmentMap::const_iterator>& children = segment->second.children;
    for (unsigned int i=0; i<children.size(); i++){
        const KDL::Segment& child_kdl = children[i]->second.segment;

        Joint* joint = new Joint(0);
        joints_[child_kdl.getJoint().getName()] = joint;

        KDL::Frame rel_pose_kdl = child_kdl.pose(joint->position_);
        geo::Transform rel_pose(geo::Matrix3(rel_pose_kdl.M.data), geo::Vector3(rel_pose_kdl.p.data));

        Object* child = new Object("robot_link", child_kdl.getName());
        addChildren(*child, children[i]);

        obj.addChild(child, rel_pose);

        links_[child_kdl.getName()] = child;
        joint->link_ = child;
        joint->kdl_segment_ = child_kdl;

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

        //cout << child_kdl.getName() << endl;



    }
}

void Robot::step(double dt) {
    Object::step(dt);

    for(map<string, Joint*>::iterator it_joint = joints_.begin(); it_joint != joints_.end(); ++it_joint) {
        Joint* joint = it_joint->second;
        joint->step(dt);

        // TODO: make this much nicer (see Joint::kdl_segment_)
        KDL::Frame rel_pose_kdl = joint->kdl_segment_.pose(joint->position_);
        geo::Transform rel_pose(geo::Matrix3(rel_pose_kdl.M.data), geo::Vector3(rel_pose_kdl.p.data));
        joint->link_->setPose(rel_pose);
    }

    if (event_joint_states_pub_.isScheduled()) {
        sensor_msgs::JointState joint_states = getJointStates();
        pub_joint_states.publish(joint_states);
    }

    /*
    if (event_sensors_pub_.isScheduled()) {
        for(vector<Sensor*>::iterator it_sensor = sensors_.begin(); it_sensor != sensors_.end(); ++it_sensor) {
            Sensor* sensor = *it_sensor;
            sensor->publish();
        }
    }
    */
}

void Robot::registerSensor(Sensor* sensor) {
    //this->addChild(sensor, rel_pose);
    sensors_.push_back(sensor);
    sensor->start();
}

void Robot::setJointPosition(const string& joint_name, double position) {
    map<std::string, Joint*>::iterator it_jnt = joints_.find(joint_name);
    if (it_jnt == joints_.end()) {
        ROS_ERROR("Joint %s does not exist", joint_name.c_str());
    } else {
        it_jnt->second->position_ = position;
        it_jnt->second->reference_ = position;
        //cout << "Setting " << joint_name << " to " << it_jnt->second->position_ << endl;
    }
}

void Robot::setJointReference(const string& joint_name, double position) {
    joints_[joint_name]->reference_ = position;
}

double Robot::getJointPosition(const string& joint_name) const {
    map<std::string, Joint*>::const_iterator it_jnt = joints_.find(joint_name);
    if (it_jnt == joints_.end()) {
        ROS_ERROR("Joint %s does not exist", joint_name.c_str());
        return 0;
    }
    return it_jnt->second->position_;
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

Object* Robot::getLink(const std::string& name) const {
    std::map<std::string, Object*>::const_iterator it = links_.find(name);
    if (it != links_.end()) {
        return it->second;
    } else {
        ROS_ERROR("Link %s does not exist", name.c_str());
        return 0;
    }
}

void Robot::setParameter(const std::string& param_name, const std::string& value) {

}
