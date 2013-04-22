#include <ros/ros.h>

#include "fast_simulator/Object.h"
#include "fast_simulator/Joint.h"
#include "fast_simulator/World.h"
#include "fast_simulator/Amigo.h"

using namespace std;

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "fast_simulator");
    ros::NodeHandle nh;

    bool publish_localization = true;

    World& world = World::getInstance();
    world.initFromTopic("/fast_simulator/map");

    // PUBLISHERS    

    // AMIGO

    Amigo* amigo = new Amigo(nh, publish_localization);
    amigo->pose_.setOrigin(tf::Vector3(0, 0, 0));
    amigo->pose_.setRotation(tf::Quaternion(0, 0, 0, 1));
    world.addObject(amigo);

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
