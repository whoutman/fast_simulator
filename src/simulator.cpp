/*

    By Sjoerd van den Dries (2013)

    TODO:
       - Get rid of counting and replace with event queue
       - Parallelize controllers from sensors



*/


#include <ros/package.h>


#include "fast_simulator/Amigo.h"
#include "fast_simulator/Pico.h"

#include <boost/program_options.hpp>

#include "fast_simulator/simulator.h"

#include "fast_simulator/ModelParser.h"

#include "fast_simulator/Octree.h"

using namespace std;


Simulator::Simulator() : world_(World::getInstance()), UNIQUE_VIS_ID(0) {

}

Simulator::~Simulator() {

}

void Simulator::step(double dt) {
    world_.step(dt);
}

void Simulator::addObject(const std::string& id, Object* obj) {
    world_.addObject(id, obj);
}

Object* Simulator::getObject(const std::string& id) const {
    return world_.getObject(id);
}

void Simulator::removeObject(const std::string& id) {
    world_.removeObject(id);
}

void Simulator::addModel(const std::string& name, const Object& obj) {
    MODELS[name] = obj;
}

const Object* Simulator::getModel(const std::string& name) const {
    map<string, Object>::const_iterator it = MODELS.find(name);
    if (it == MODELS.end()) {
        return 0;
    }
    return &it->second;
}

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//
//                                      ROS VISUALIZATION
//
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

visualization_msgs::MarkerArray Simulator::getROSVisualizationMessage() {
    //vector<Object*> objects = world_.getObjectsRecursive();
    map<string, Object*> objects = world_.getObjects();

    visualization_msgs::MarkerArray marker_array;
    //for(vector<Object*>::const_iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj) {
    for(map<string, Object*>::const_iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj) {       

        //Object& obj = **it_obj;
        Object& obj = *it_obj->second;

        visualization_msgs::Marker m;

        m.action = visualization_msgs::Marker::ADD;

        /*
        map<string, int>::iterator it_vis_id = object_id_to_vis_id.find(obj.getID());
        if (it_vis_id == object_id_to_vis_id.end()) {
            m.id = UNIQUE_VIS_ID;
            object_id_to_vis_id[obj.getID()] = m.id;
            UNIQUE_VIS_ID++;
        } else {
            m.id = it_vis_id->second;
        }
        */

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

        //m_text.id = m.id + 10000;

        if (obj.getID() != "") {
            m_text.text = obj.getID();
            m_text.color.r = 1;
            m_text.color.g = 1;
            m_text.color.b = 1;
        } else {
            m_text.text = obj.getType();
            m_text.color.r = 1;
            m_text.color.g = 0;
            m_text.color.b = 0;
        }

        m_text.action = visualization_msgs::Marker::ADD;
        m_text.header.frame_id = "/map";
        tf::poseTFToMsg(pose, m_text.pose);
        m_text.pose.position.z += 0.2;
        m_text.color.a = 1;

        m_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        m_text.scale.x = 0.1;
        m_text.scale.y = 0.1;
        m_text.scale.z = 0.1;

        m_text.lifetime = ros::Duration(1.0);

        marker_array.markers.push_back(m);
        marker_array.markers.push_back(m_text);

    }

    for(unsigned int i = 0; i < marker_array.markers.size(); ++i) {
        marker_array.markers[i].id = i;
    }

    return marker_array;
}


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//
//                                            MAIN
//
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

Simulator* SIM;

string MODEL_DIR;

bool setObject(fast_simulator::SetObject::Request& req, fast_simulator::SetObject::Response& res) {

    Object* obj = SIM->getObject(req.id);

    if (req.action == fast_simulator::SetObject::Request::SET_POSE) {

        if (!obj) {
            if (req.type == "box") {
                obj = new Object(req.type);
                obj->setShape(Box(tf::Vector3(-0.4, -0.4, 0), tf::Vector3(0.4, 0.4, 1)));
            } else if (req.type == "person") {
                obj = new Object(req.type);

                Object* body = new Object("body", req.id + "-body");
                body->setShape(Octree::fromHeightImage(MODEL_DIR + "/laser/body.pgm", 1, 0.025));
                obj->addChild(body, tf::Vector3(0, 0, 0.5), tf::Quaternion(0, 0, 0, 1));
                //body->setPose(tf::Vector3(0, 0, 1), tf::Quaternion(0, 0, 0, 1));
                //obj->addChild(body, tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.5, 0.5, 0.5)));

                Object* face = new Object("face", req.id + "-face");
                obj->addChild(face, tf::Vector3(0, 0, 1.6), tf::Quaternion(0, 0, 0, 1));

            } else {
                const Object* model = SIM->getModel(req.type);
                if (model) {
                    obj = Object::fromModel(*model);
                } else {
                    obj = new Object(req.type);
                    //cout << "Unknown model type: '" << req.type << "'" << endl;
                    //return true;
                }

            }
            SIM->addObject(req.id, obj);
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
            SIM->removeObject(req.id);
        } else if (req.action == fast_simulator::SetObject::Request::SET_PARAMS) {

        }
    }

    return true;


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

    // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

    // parse models

    std::map<std::string, Object> MODELS;
    ModelParser model_parser(MODEL_DIR + "/models/models.xml", MODEL_DIR);
    if (!model_parser.parse(MODELS)) {
        ROS_ERROR("Could not parse models: %s", model_parser.getError().c_str());
    }    

    // * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

    SIM = new Simulator();

    for(map<string, Object>::iterator it = MODELS.begin(); it != MODELS.end(); ++it) {
        SIM->addModel(it->first, it->second);
    }

    if (world_name != "") {
        map<string, Object>::iterator it_world = MODELS.find(world_name);
        if (it_world != MODELS.end()) {
            SIM->addObject(world_name, new Object(it_world->second));
        } else {
            ROS_ERROR("While loading world: could not find model: '%s'", world_name.c_str());
        }
    }

    // PUBLISHERS

    ros::Publisher PUB_MARKER = nh.advertise<visualization_msgs::MarkerArray>("/fast_simulator/visualization", 10);

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
        SIM->addObject("pico", pico);
    } else {
        Amigo* amigo = new Amigo(nh, publish_localization);

        // add kinect
        Kinect* top_kinect = new Kinect("/camera/rgb/image_rect_color", "/camera/depth_registered/image", "/camera/rgb/camera_info", "/camera/rgb/points", "/openni_rgb_optical_frame");
        //top_kinect->addModel("loy", MODEL_DIR + "/kinect/loy");
        top_kinect->addModel("coke", MODEL_DIR + "/kinect/coke_cropped");
        top_kinect->addModel("cif", MODEL_DIR + "/kinect/cif_cropped");
        top_kinect->addModel("tea_pack", MODEL_DIR + "/kinect/tea_pack_cropped");
        top_kinect->addModel("face", MODEL_DIR + "/kinect/loy_cropped");

        top_kinect->addModel("drops", MODEL_DIR + "/kinect/drops_cropped");
        top_kinect->addModel("marmalade", MODEL_DIR + "/kinect/marmalade_cropped");
        top_kinect->addModel("tomato_soup", MODEL_DIR + "/kinect/tomato_soup_cropped");
        top_kinect->addModel("cleaner", MODEL_DIR + "/kinect/cleaner_cropped");

        top_kinect->addModel("energy_drink", MODEL_DIR + "/kinect/energy_drink_cropped");
        top_kinect->addModel("sponge", MODEL_DIR + "/kinect/sponge_cropped");
        top_kinect->addModel("veggie_noodles", MODEL_DIR + "/kinect/veggie_noodles_cropped");

        top_kinect->addModel("apple_juice", MODEL_DIR + "/kinect/apple_juice");
        top_kinect->addModel("beer_bottle", MODEL_DIR + "/kinect/beer_bottle");
        top_kinect->addModel("chocolate_milk", MODEL_DIR + "/kinect/chocolate_milk");
        top_kinect->addModel("coke_rwc2013", MODEL_DIR + "/kinect/coke_rwc2013");
        top_kinect->addModel("cookies", MODEL_DIR + "/kinect/cookies");
        top_kinect->addModel("crackers", MODEL_DIR + "/kinect/crackers");
        top_kinect->addModel("deodorant", MODEL_DIR + "/kinect/deodorant");
        top_kinect->addModel("fanta", MODEL_DIR + "/kinect/fanta");
        top_kinect->addModel("fresh_discs", MODEL_DIR + "/kinect/fresh_discs");
        top_kinect->addModel("garlic_sauce", MODEL_DIR + "/kinect/garlic_sauce");
        top_kinect->addModel("milk", MODEL_DIR + "/kinect/milk");
        top_kinect->addModel("orange_juice", MODEL_DIR + "/kinect/orange_juice");
        top_kinect->addModel("peanut_butter", MODEL_DIR + "/kinect/peanut_butter");
        top_kinect->addModel("seven_up", MODEL_DIR + "/kinect/seven_up");
        top_kinect->addModel("tooth_paste", MODEL_DIR + "/kinect/tooth_paste");

        top_kinect->setRaytracing(raytrace);

        amigo->registerSensor(top_kinect);
        amigo->getLink("openni_rgb_optical_frame")->addChild(top_kinect);


        LRF* base_lrf = new LRF("/base_scan", "/front_laser");
        amigo->registerSensor(base_lrf);
        amigo->getLink("front_laser")->addChild(base_lrf);

        LRF* torso_lrf = new LRF("/top_scan", "/torso_laser");
        amigo->registerSensor(torso_lrf);
        amigo->getLink("torso_laser")->addChild(torso_lrf);
        cout << "TORSO_LASER " << amigo->getLink("torso_laser") << endl;

        //tf::Transform tf_base_link_to_top_laser;
        //tf_base_link_to_top_laser.setOrigin(tf::Vector3(0.31, 0, 1.0));
        //tf_base_link_to_top_laser.setRotation(tf::Quaternion(0, 0, 0, 1));
        //LRF* laser_range_finder_top_ = new LRF("/top_scan", "/front_laser");
        //this->registerSensor(laser_range_finder_top_, tf_base_link_to_top_laser);

        amigo->setPose(robot_pos, robot_ori);

        SIM->addObject("amigo", amigo);
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

        SIM->step(dt);

        if (count % 10 == 0) {
            visualization_msgs::MarkerArray marker_array = SIM->getROSVisualizationMessage();
            PUB_MARKER.publish(marker_array);
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

    delete SIM;

    return 0;
}
