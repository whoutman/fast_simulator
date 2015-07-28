#include "fast_simulator/SimulatorROS.h"

#include <tue/config/loaders/yaml.h>
#include <tue/config/configuration.h>

// Models
#include "fast_simulator/StandaloneKinect.h"

#include <ros/package.h>

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//
//                                            MAIN
//
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

void configure(SimulatorROS& sim, tue::Configuration config, ros::NodeHandle& nh, const std::string& model_dir)
{
    if (config.readArray("objects"))
    {
        while (config.nextArrayItem())
        {
            // Check for the 'enabled' field. If it exists and the value is 0, omit this object. This allows
            // the user to easily enable and disable certain objects with one single flag.
            int enabled;
            if (config.value("enabled", enabled, tue::OPTIONAL) && !enabled)
                continue;

            std::string id;
            if (!config.value("id", id))
                continue;

            std::string type;
            config.value("type", type, tue::OPTIONAL);

            geo::Pose3D pose = geo::Pose3D::identity();
            if (config.readGroup("pose", tue::REQUIRED))
            {
                config.value("x", pose.t.x);
                config.value("y", pose.t.y);
                config.value("z", pose.t.z);

                double roll = 0, pitch = 0, yaw = 0;
                config.value("roll", roll, tue::OPTIONAL);
                config.value("pitch", pitch, tue::OPTIONAL);
                config.value("yaw", yaw, tue::OPTIONAL);
                pose.R.setRPY(roll, pitch, yaw);

                config.endGroup();
            }
            else
                continue;

            Object* obj = 0;
            if (type == "kinect")
            {
                std::string topic, frame_id;
                if (config.value("topic", topic) && config.value("frame", frame_id))
                {
                    StandaloneKinect* kinect = new StandaloneKinect(nh, topic, frame_id, model_dir);
                    obj = kinect;
                }
            }
            else if (!type.empty())
            {
                obj = sim.getObjectFromModel(type, id);
            }

            if (obj)
            {
                std::cout << "Added object: id = '" << id << "', type = '" << type << "', pose = " << pose << std::endl;

                obj->setPose(pose);
                sim.addObject(id, obj);
            }
        }

        config.endArray();
    }
}

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

        configure(sim, config, nh, model_dir);

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
