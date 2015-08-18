#ifndef FAST_SIMULATOR_ROS_H_
#define FAST_SIMULATOR_ROS_H_

#include <ros/ros.h>

#include "fast_simulator/Simulator.h"

#include <visualization_msgs/MarkerArray.h>
#include "fast_simulator/SetObject.h"
#include "ModelParser.h"

#include <tue/config/configuration.h>

class SimulatorROS
{

public:

    SimulatorROS(ros::NodeHandle& nh, const std::string& model_file, const std::string& model_dir);

    ~SimulatorROS();

    void configure(tue::Configuration& config);

    void parseModelFile(const std::string& filename, const std::string& model_dir);

    Object* getObjectFromModel(const std::string& model_name, const std::string& id = "");

    void addObject(const std::string& id, Object* obj);

    bool setObject(fast_simulator::SetObject::Request& req, fast_simulator::SetObject::Response& res);

    void start();

protected:

    ros::NodeHandle nh_;

    Simulator simulator_;

    std::set<std::string> faces_;

    ros::Publisher pub_marker_;

    ros::ServiceServer srv_set_object_;

    ModelParser* model_parser_;

    std::string model_dir_;


    visualization_msgs::MarkerArray getROSVisualizationMessage();

};

#endif
