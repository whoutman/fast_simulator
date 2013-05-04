/*

    By Sjoerd van den Dries (2013)

    TODO:
       - Get rid of counting and replace with event queue
       - Parallelize controllers from sensors



*/

#include <ros/ros.h>
#include <ros/package.h>

#include "fast_simulator/Object.h"
#include "fast_simulator/Joint.h"
#include "fast_simulator/World.h"
#include "fast_simulator/Sprite.h"
#include "fast_simulator/Amigo.h"
#include "fast_simulator/Pico.h"
#include "fast_simulator/ModelParser.h"

#include "fast_simulator/SetObject.h"

#include <visualization_msgs/MarkerArray.h>

#include <boost/program_options.hpp>

using namespace std;

World* WORLD;
string MODEL_DIR;
int UNIQUE_VIS_ID = 0;
ros::Publisher PUB_MARKER;

map<string, int> object_id_to_vis_id;
map<string, Object> MODELS;

bool setObject(fast_simulator::SetObject::Request& req, fast_simulator::SetObject::Response& res) {

    Object* obj = WORLD->getObject(req.id);

    if (req.action == fast_simulator::SetObject::Request::SET_POSE) {

        if (!obj) {
            if (req.type == "box") {
                obj = new Object(req.type);
                obj->setShape(Box(tf::Vector3(-0.4, -0.4, 0), tf::Vector3(0.4, 0.4, 1)));
                WORLD->addObject(req.id, obj);
            } else {
                map<string, Object>::iterator it_model = MODELS.find(req.type);
                if (it_model == MODELS.end()) {
                    cout << "Unknown model type: '" << req.type << "'" << endl;
                    return true;
                }
                obj = new Object(it_model->second);
                WORLD->addObject(req.id, obj);
            }
        }

        /*
        if (!obj) {
            obj = new Object(req.type);
            if (req.type == "person") {
                obj->setBoundingBox(Box(tf::Vector3(-0.4, -0.4, 0.5), tf::Vector3(0.4, 0.4, 1.5)));
                obj->setShape(Sprite(MODEL_DIR + "/laser/body.pgm", 0.025, 0.5, 1.5));
            } else if (req.type == "box") {
                obj->setShape(Box(tf::Vector3(-0.4, -0.4, 0), tf::Vector3(0.4, 0.4, 1)));
            }

            WORLD->addObject(req.id, obj);
        }
        */

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

namespace po = boost::program_options;

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "fast_simulator");
    ros::NodeHandle nh;

    MODEL_DIR = ros::package::getPath("fast_simulator_data");
    if (MODEL_DIR == "") {
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
        ("x", po::value<double>(), "X-value of robot intial pose")
        ("y", po::value<double>(), "Y-value of robot intial pose")
        ("z", po::value<double>(), "Z-value of robot intial pose")
        ("rx", po::value<double>(), "X-value of robot intial rotation")
        ("ry", po::value<double>(), "Y-value of robot intial rotation")
        ("rz", po::value<double>(), "Z-value of robot intial rotation")
        ("no-localization", "Set if no transformation from /map to /odom should be published")
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

    string robot_name = "";
    if (vm.count("robot")) {
        robot_name = vm["robot"].as<string>();
    }

    tf::Quaternion robot_ori;
    robot_ori.setEuler(robot_ori_x, robot_ori_y, robot_ori_z);

    // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

    // parse models

    MODELS.clear();
    ModelParser model_parser(MODEL_DIR + "/models/models.xml");
    if (!model_parser.parse(MODELS)) {
        ROS_ERROR("Could not parse models: %s", model_parser.getError().c_str());
    }

    // parse world

    map<string, Object> world;
    ModelParser world_parser(MODEL_DIR + "/models/world.xml");
    if (!world_parser.parse(world)) {
        ROS_ERROR("Could not parse world: %s", model_parser.getError().c_str());
    }

    WORLD = &World::getInstance();
    WORLD->addObject("world", new Object(world["world"]));

    //WORLD->initFromTopic("/fast_simulator/map");

    // PUBLISHERS

    PUB_MARKER = nh.advertise<visualization_msgs::MarkerArray>("/fast_simulator/visualization", 10);

    if (robot_name == "pico") {
        Pico* pico = new Pico(nh, publish_localization);

        // add laser
        LRF* laser_range_finder_ = new LRF("/robot/body/laser", "/laser");
        pico->registerSensor(laser_range_finder_);
        pico->getLink("laser")->addChild(laser_range_finder_);

        Sonar* front_sonar_ = new Sonar("/robot/body/sonar_front", "/sonar_front");
        pico->registerSensor(front_sonar_);
        pico->getLink("sonar_front")->addChild(front_sonar_);

        pico->setPose(robot_pos, robot_ori);
        WORLD->addObject("pico", pico);
    } else {
        Amigo* amigo = new Amigo(nh, publish_localization);

        // add kinect
        Kinect* top_kinect = new Kinect("/camera/rgb/image_rect_color", "/camera/depth_registered/image", "/camera/rgb/camera_info", "/camera/rgb/points", "/openni_rgb_optical_frame");
        //top_kinect->addModel("loy", MODEL_DIR + "/kinect/loy");
        top_kinect->addModel("coke", MODEL_DIR + "/kinect/coke");

        amigo->registerSensor(top_kinect);
        amigo->getLink("openni_rgb_optical_frame")->addChild(top_kinect);


        LRF* laser_range_finder_ = new LRF("/base_scan", "/front_laser");
        amigo->registerSensor(laser_range_finder_);
        amigo->getLink("front_laser")->addChild(laser_range_finder_);

        //tf::Transform tf_base_link_to_top_laser;
        //tf_base_link_to_top_laser.setOrigin(tf::Vector3(0.31, 0, 1.0));
        //tf_base_link_to_top_laser.setRotation(tf::Quaternion(0, 0, 0, 1));
        //LRF* laser_range_finder_top_ = new LRF("/top_scan", "/front_laser");
        //this->registerSensor(laser_range_finder_top_, tf_base_link_to_top_laser);

        amigo->setPose(robot_pos, robot_ori);
        WORLD->addObject("amigo", amigo);
    }

    ros::ServiceServer srv_set_object_ = nh.advertiseService("/fast_simulator/set_object", &setObject);

    double real_time_factor = 1;

    double freq = 100;
    ros::Rate r(freq);

    ros::Duration max_cycle_time(0);

    long count = 0;

    ros::Time t = ros::Time(0);

    while(ros::ok()) {
        double dt = 0;
        if (t > ros::Time(0)) {
            dt = (ros::Time::now() - t).toSec() * real_time_factor;
        }
        t = ros::Time::now();

        ros::Time t_start = ros::Time::now();

        ros::spinOnce();

        WORLD->step(dt);

        if (count % 10 == 0) {
            visualizeObjects();
        }

        ros::Duration cycle_time = ros::Time::now() - t_start;
        if (cycle_time > max_cycle_time) {
            max_cycle_time = cycle_time;
        }

        //cout << "Max main cycle duration: " << max_cycle_time << " seconds" << endl;

        ++count;
        r.sleep();
    }

    srv_set_object_.shutdown();

    return 0;
}
