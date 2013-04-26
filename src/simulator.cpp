#include <ros/ros.h>
#include <ros/package.h>

#include "fast_simulator/Object.h"
#include "fast_simulator/Joint.h"
#include "fast_simulator/World.h"
#include "fast_simulator/Sprite.h"
#include "fast_simulator/Amigo.h"
#include "fast_simulator/Pico.h"

#include "fast_simulator/SetObject.h"

#include <visualization_msgs/MarkerArray.h>

using namespace std;

World* WORLD;
string MODEL_DIR;
int UNIQUE_VIS_ID = 0;
ros::Publisher PUB_MARKER;

map<string, int> object_id_to_vis_id;

bool setObject(fast_simulator::SetObject::Request& req, fast_simulator::SetObject::Response& res) {

    Object* obj = WORLD->getObject(req.id);

    if (req.action == fast_simulator::SetObject::Request::SET_POSE) {
        if (!obj) {
            obj = new Object(req.type);
            if (req.type == "person") {
                obj->setBoundingBox(Box(tf::Vector3(-0.4, -0.4, 0.5), tf::Vector3(0.4, 0.4, 1.5)));
                obj->setShape(Sprite(MODEL_DIR + "/laser/body.pgm", 0.025, 0.5, 1.5));
            }
            WORLD->addObject(req.id, obj);
        }

        tf::Point pos;
        tf::pointMsgToTF(req.pose.position, pos);
        tf::Quaternion rot;
        tf::quaternionMsgToTF(req.pose.orientation, rot);
        obj->setPose(pos, rot);

        cout << "Set " << req.id << " (type: " << req.type << ") at position (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")" << endl;

    } else {
        if (!obj) {
            res.result_msg = "Object with id " + req.id + " does not exist";
            return true;
        }

        if (req.action == fast_simulator::SetObject::Request::DELETE) {
            WORLD->removeObject(req.id);
        } else if (req.action == fast_simulator::SetObject::Request::SET_PARAMS) {

        }
    }

    return true;



    /*
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

void visualizeObjects() {
    World& world = World::getInstance();
    map<string, Object*> objects = world.getObjects();

    visualization_msgs::MarkerArray marker_array;
    for(map<string, Object*>::const_iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj) {
        Object& obj = *it_obj->second;

        visualization_msgs::Marker m;

        m.action = visualization_msgs::Marker::ADD;

        map<string, int>::iterator it_vis_id = object_id_to_vis_id.find(obj.getID());
        if (it_vis_id == object_id_to_vis_id.end()) {
            m.id = UNIQUE_VIS_ID;
            object_id_to_vis_id[obj.getID()] = m.id;
            UNIQUE_VIS_ID++;
        } else {
            m.id = it_vis_id->second;
        }

        tf::Transform pose = obj.getAbsolutePose();
        m.header.frame_id = "/map";
        tf::poseTFToMsg(pose, m.pose);

        m.color.a = 1;
        m.color.r = 1;
        m.color.g = 1;
        m.color.b = 1;

        m.type = visualization_msgs::Marker::SPHERE;
        m.scale.x = 0.1;
        m.scale.y = 0.1;
        m.scale.z = 0.1;

        m.lifetime = ros::Duration(1.0);

        // add text
        visualization_msgs::Marker m_text;

        m_text.id = m.id + 10000;
        m_text.text = obj.getID();
        m_text.action = visualization_msgs::Marker::ADD;
        m_text.header.frame_id = "/map";
        tf::poseTFToMsg(pose, m_text.pose);
        m_text.pose.position.z += 0.2;
        m_text.color.a = 1;
        m_text.color.r = 1;
        m_text.color.g = 1;
        m_text.color.b = 1;

        m_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m_text.scale.x = 0.1;
        m_text.scale.y = 0.1;
        m_text.scale.z = 0.1;

        m_text.lifetime = ros::Duration(1.0);

        marker_array.markers.push_back(m);
        marker_array.markers.push_back(m_text);
    }

    PUB_MARKER.publish(marker_array);
}

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "fast_simulator");
    ros::NodeHandle nh;

    MODEL_DIR = ros::package::getPath("fast_simulator_data");
    if (MODEL_DIR == "") {
        ROS_ERROR("Could not find package 'fast_simulator_data' for object models. Exiting..");
        exit(-1);
    }

    set<string> args;
    for(int i = 1; i < argc; ++i) {
        args.insert(argv[i]);
    }

    bool publish_localization = (args.find("no-loc") == args.end());

    WORLD = &World::getInstance();
    WORLD->initFromTopic("/fast_simulator/map");

    // PUBLISHERS    

    PUB_MARKER = nh.advertise<visualization_msgs::MarkerArray>("/fast_simulator/visualization", 10);

    if (args.find("pico") != args.end()) {
        Pico* pico = new Pico(nh, publish_localization);
        pico->setPose(tf::Vector3(0, 0, 0), tf::Quaternion(0, 0, 0, 1));
        WORLD->addObject("pico", pico);
    } else {
        Amigo* amigo = new Amigo(nh, publish_localization);

        // add kinect
        tf::Transform tf_base_link_to_kinect;
        tf_base_link_to_kinect.setOrigin(tf::Vector3(0.31, 0, 1.0)); // exact pose doesnt matter for now
        tf_base_link_to_kinect.setRotation(tf::Quaternion(0, 0, 0, 1));
        Kinect* top_kinect = new Kinect("/camera/rgb/image_rect_color", "/camera/depth_registered/image", "/camera/rgb/camera_info", "/openni_rgb_optical_frame");
        //top_kinect->addModel("loy", MODEL_DIR + "/kinect/loy");
        top_kinect->addModel("coke", MODEL_DIR + "/kinect/coke");

        amigo->addSensor(top_kinect, tf_base_link_to_kinect);

        amigo->setPose(tf::Vector3(-0.6, 0, 0), tf::Quaternion(0, 0, 0, 1));
        WORLD->addObject("amigo", amigo);
    }

    ros::ServiceServer srv_set_object_ = nh.advertiseService("/fast_simulator/set_object", &setObject);

    double freq = 100;
    ros::Rate r(freq);

    long count = 0;
    while(ros::ok()) {
        ros::spinOnce();

        WORLD->step(1 / freq);

        if (count % 10 == 0) {
            visualizeObjects();
        }

        ++count;
        r.sleep();
    }

    srv_set_object_.shutdown();

    return 0;
}
