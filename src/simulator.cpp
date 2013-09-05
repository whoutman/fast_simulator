#include "fast_simulator/SimulatorROS.h"

#include <ros/package.h>
#include <boost/program_options.hpp>

using namespace std;

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//
//                                            MAIN
//
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

namespace po = boost::program_options;

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "fast_simulator");
    ros::NodeHandle nh;

    string model_dir = ros::package::getPath("fast_simulator_data");
    if (model_dir == "") {
        ROS_ERROR("Could not find package 'fast_simulator_data' for object models. Exiting..");
        exit(-1);
    }

    tf::Vector3 robot_pos(0, 0, 0);
    double robot_ori_x = 0;
    double robot_ori_y = 0;
    double robot_ori_z = 0;

    // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    // PARSE COMMAND-LINE ARGUMENTS

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Show this")
        ("robot", po::value<string>(), "Type of robot: pico or amigo")
        ("world", po::value<string>(), "Name of the model that should be used as world")
        ("x", po::value<double>(), "X-value of robot intial pose")
        ("y", po::value<double>(), "Y-value of robot intial pose")
        ("z", po::value<double>(), "Z-value of robot intial pose")
        ("rx", po::value<double>(), "X-value of robot intial rotation")
        ("ry", po::value<double>(), "Y-value of robot intial rotation")
        ("rz", po::value<double>(), "Z-value of robot intial rotation")
        ("no-localization", "Set if no transformation from /map to /odom should be published")
        ("kinect-raytracing", po::value<bool>(), "If set to false, Kinect raytracing is disabled (default: true)")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        cout << desc << "\n";
        return 1;
    }

    if (vm.count("x")) { robot_pos.setX(vm["x"].as<double>()); }
    if (vm.count("y")) { robot_pos.setY(vm["y"].as<double>()); }
    if (vm.count("z")) { robot_pos.setZ(vm["z"].as<double>()); }

    if (vm.count("rx")) { robot_ori_x = vm["rx"].as<double>(); }
    if (vm.count("ry")) { robot_ori_y = vm["ry"].as<double>(); }
    if (vm.count("rz")) { robot_ori_z = vm["rz"].as<double>(); }

    bool publish_localization = !(vm.count("no-localization"));

    bool raytrace = true;
    if (vm.count("kinect-raytracing")) { raytrace = vm["kinect-raytracing"].as<bool>(); }

    string robot_name = "";
    if (vm.count("robot")) {
        robot_name = vm["robot"].as<string>();
    }

    string world_name = "";
    if (vm.count("world")) {
        world_name = vm["world"].as<string>();
    }

    tf::Quaternion robot_ori;
    robot_ori.setEuler(robot_ori_x, robot_ori_y, robot_ori_z);

    SimulatorROS SIM(nh, model_dir + "/models/models.xml", model_dir);
    SIM.parseModelFile(model_dir + "/models/models.xml", model_dir);

    SIM.start();

    return 0;
}
