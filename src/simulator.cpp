#include "fast_simulator/SimulatorROS.h"

#include <tue/config/loaders/yaml.h>
#include <tue/config/configuration.h>

// Models
#include "fast_simulator/StandaloneKinect.h"

#include <ros/package.h>

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    // - - - - - - - - - - - - - - - - - - - - - -
    // Initialize ROS node

    ros::init(argc, argv, "fast_simulator");
    ros::NodeHandle nh;

    // - - - - - - - - - - - - - - - - - - - - - -
    // Create simulator

    std::string model_dir = ros::package::getPath("fast_simulator_data");

    SimulatorROS sim(nh, model_dir + "/models/models.xml", model_dir);
    sim.parseModelFile(model_dir + "/models/models.xml", model_dir);

    // - - - - - - - - - - - - - - - - - - - - - -
    // Configure simulator

    if (argc >= 2)
    {
        tue::Configuration config;
        std::string yaml_filename = argv[1];
        config.loadFromYAMLFile(yaml_filename);

        if (config.hasError())
        {
            std::cout << std::endl << "Could not load configuration file:" << std::endl << std::endl << config.error() << std::endl;
            return 1;
        }

        sim.configure(config);

        if (config.hasError())
        {
            std::cout << std::endl << "Error during configuration:" << std::endl << std::endl << config.error() << std::endl;
            return 1;
        }
    }

    // - - - - - - - - - - - - - - - - - - - - - -
    // Start simulator

    sim.start();

    return 0;
}
