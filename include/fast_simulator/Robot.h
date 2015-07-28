#ifndef _FAST_SIMULATOR_ROBOT_H_
#define _FAST_SIMULATOR_ROBOT_H_

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "fast_simulator/Joint.h"
#include "fast_simulator/Object.h"
#include "fast_simulator/World.h"
#include "fast_simulator/LRF.h"
#include "fast_simulator/Kinect.h"

struct Trajectory {
    std::vector<double> set_points;
    double dt;
};

class Robot : public Object {

public:

    Robot(ros::NodeHandle& nh);

    Robot(ros::NodeHandle& nh, const std::string& robot_type);

    virtual ~Robot();

    void addChildren(Object& obj, const KDL::SegmentMap::const_iterator segment);

    //virtual Robot* clone() const = 0;

    void step(double dt);

    void registerSensor(Sensor *sensor);

    void setJointPosition(const std::string& joint_name, double position);

    void setJointReference(const std::string& joint_name, double position);

    double getJointPosition(const std::string& joint_name) const;

    sensor_msgs::JointState getJointStates();

    Object* getLink(const std::string& name) const;

    void setParameter(const std::string& param_name, const std::string& value);

protected:

    ros::NodeHandle& nh_;

    tf::TransformBroadcaster tf_broadcaster_;

private:

    KDL::Tree tree;

    std::map<std::string, Joint*> joints_;

    std::vector<Sensor*> sensors_;

    ros::Publisher pub_joint_states;

    std::map<std::string, Object*> links_;

    Event event_joint_states_pub_;
    Event event_sensors_pub_;


};


#endif
