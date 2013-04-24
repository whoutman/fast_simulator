#include <ros/ros.h>

#include "fast_simulator/Object.h"
#include "fast_simulator/Joint.h"
#include "fast_simulator/World.h"
#include "fast_simulator/Amigo.h"
#include "fast_simulator/Pico.h"

#include "fast_simulator/SetObject.h"

using namespace std;

World* WORLD;

bool setObject(fast_simulator::SetObject::Request& req, fast_simulator::SetObject::Response& res) {
    /*
    WORLD->


    map<string, Object*>::iterator it_obj = id_to_object_.find(req.id);

    if (req.action == gazebo_life::Set::Request::DELETE) {
        if (it_obj != id_to_object_.end()) {
            delete it_obj->second;
            id_to_object_.erase(it_obj);
        } else {
            res.result_msg = "Could not find object for deletion";
        }
    } else if (req.action == gazebo_life::Set::Request::SET_PARAMS) {
        if (it_obj == id_to_object_.end()) {
            res.result_msg = "Could not find object for setting path";
            return true;
        }

        Object* obj = it_obj->second;

        for(unsigned int i = 0; i < req.param_names.size(); ++i) {
            if (req.param_names[i] == "velocity") {
                obj->velocity_ = req.param_values[i];
            } else {
                res.result_msg = "Unknown parameter: " + req.param_names[i];
                return true;
            }
        }
    } else if (req.action == gazebo_life::Set::Request::SET_POSE) {
        Object* obj = 0;
        if (it_obj == id_to_object_.end()) {
            obj = new Object();
            obj->id_ = req.id;
            id_to_object_[req.id] = obj;

            if (req.type == "person") {
                obj->addChild("face", 0, 0, 0.8);
                obj->addChild("leg", 0,  0.1, -0.8);
                obj->addChild("leg", 0, -0.1, -0.8);
            }

        } else {
            obj = it_obj->second;
        }

        obj->type_ = req.type;
        obj->current_goal_ = -1;
        tf::poseStampedMsgToTF(req.pose, obj->pose_);
    } else if (req.action == gazebo_life::Set::Request::SET_PATH) {
        if (it_obj == id_to_object_.end()) {
            res.result_msg = "Could not find object for setting path";
            return true;
        }

        Object* obj = it_obj->second;

        obj->current_goal_ = 0;
        for(unsigned int i = 0; i < req.path.size(); ++i) {
            tf::Stamped<tf::Pose> waypoint;
            tf::poseStampedMsgToTF(req.path[i], waypoint);
            obj->path_.push_back(waypoint);
        }
    }

    res.result_msg = "";

    return true;
    */
}

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "fast_simulator");
    ros::NodeHandle nh;

    set<string> args;
    for(int i = 1; i < argc; ++i) {
        args.insert(argv[i]);
    }

    bool publish_localization = (args.find("no-loc") == args.end());

    WORLD = &World::getInstance();
    WORLD->initFromTopic("/fast_simulator/map");

    // PUBLISHERS    

    // AMIGO

    if (args.find("pico") != args.end()) {
        Pico* pico = new Pico(nh, publish_localization);
        pico->setPose(tf::Vector3(0, 0, 0), tf::Quaternion(0, 0, 0, 1));
        WORLD->addObject("pico", pico);
    } else {
        Amigo* amigo = new Amigo(nh, publish_localization);
        amigo->setPose(tf::Vector3(-0.6, 0, 0), tf::Quaternion(0, 0, 0, 1));
        WORLD->addObject("amigo", amigo);
    }

    double freq = 100;
    ros::Rate r(freq);

    long count = 0;
    while(ros::ok()) {
        ros::spinOnce();

        WORLD->step(1 / freq);

        ++count;
        r.sleep();
    }

    return 0;
}
