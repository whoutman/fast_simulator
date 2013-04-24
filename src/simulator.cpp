#include <ros/ros.h>

#include "fast_simulator/Object.h"
#include "fast_simulator/Joint.h"
#include "fast_simulator/World.h"
#include "fast_simulator/Amigo.h"
#include "fast_simulator/Pico.h"

using namespace std;

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "fast_simulator");
    ros::NodeHandle nh;

    set<string> args;
    for(int i = 1; i < argc; ++i) {
        args.insert(argv[i]);
    }

    bool publish_localization = (args.find("no-loc") == args.end());

    World& world = World::getInstance();
    world.initFromTopic("/fast_simulator/map");

    // PUBLISHERS    

    // AMIGO

    if (args.find("pico") != args.end()) {
        Pico* pico = new Pico(nh, publish_localization);
        pico->setPose(tf::Vector3(0, 0, 0), tf::Quaternion(0, 0, 0, 1));
        world.addObject(pico);
    } else {
        Amigo* amigo = new Amigo(nh, publish_localization);
        amigo->setPose(tf::Vector3(0, 0, 0), tf::Quaternion(0, 0, 0, 1));
        world.addObject(amigo);
    }

    double freq = 100;
    ros::Rate r(freq);

    long count = 0;
    while(ros::ok()) {
        ros::spinOnce();

        world.step(1 / freq);

        ++count;
        r.sleep();
    }

    return 0;
}
