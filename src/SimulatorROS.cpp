/*

    By Sjoerd van den Dries (2013)

    TODO:
       - Get rid of counting and replace with event queue
       - Parallelize controllers from sensors



*/

#include "fast_simulator/SimulatorROS.h"

#include "fast_simulator/Amigo.h"
#include "fast_simulator/Pico.h"
#include "fast_simulator/Pera.h"
#include "fast_simulator/StandaloneKinect.h"
#include "fast_simulator/StandaloneLRF.h"
#include "fast_simulator/LRF.h"

#include "fast_simulator/simulator.h"
#include "fast_simulator/ModelParser.h"
//#include "fast_simulator/Octree.h"
#include "fast_simulator/util.h"

#include <geolib/Box.h>
#include <geolib/HeightMap.h>
#include <geolib/ros/msg_conversions.h>

#include <tue/filesystem/crawler.h>

using namespace std;

SimulatorROS::SimulatorROS(ros::NodeHandle& nh, const std::string& model_file, const std::string& model_dir)
    : nh_(nh), model_parser_(new ModelParser(model_file, model_dir)), model_dir_(model_dir) {
    pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("/fast_simulator/visualization", 10);
    srv_set_object_ = nh_.advertiseService("/fast_simulator/set_object", &SimulatorROS::setObject, this);

}

SimulatorROS::~SimulatorROS() {
    srv_set_object_.shutdown();
    delete model_parser_;
}

void SimulatorROS::parseModelFile(const std::string& filename, const std::string& model_dir) {
    faces_.insert("loy");
    faces_.insert("tim");
    faces_.insert("rob");
    faces_.insert("erik");
    faces_.insert("sjoerd");
}

Object* SimulatorROS::getObjectFromModel(const std::string& model_name, const std::string& id) {
    string parse_error;
    Object* obj = model_parser_->parse(model_name, parse_error);
    if (!parse_error.empty()) {
        ROS_ERROR("While parsing model file: %s", parse_error.c_str());
    }

    if (obj) {
        return obj;
    }

    if (model_name == "pico") {
        Pico* pico = new Pico(nh_, true); //publish_localization);

        // add laser
        LRF* laser_range_finder_ = new LRF("/pico/laser", "/pico/laser");
        pico->registerSensor(laser_range_finder_);
        pico->getLink("laser")->addChild(laser_range_finder_);

//        Sonar* front_sonar_ = new Sonar("/pico/sonar_front", "/pico/sonar_front");
//        pico->registerSensor(front_sonar_);
//        pico->getLink("sonar_front")->addChild(front_sonar_);

        return pico;
    } else if (model_name == "pera") {
        Pera* pera = new Pera(nh_);
        return pera;
    } else if (model_name == "amigo") {
        Amigo* amigo = new Amigo(nh_, true); //publish_localization);

        // add kinect
        Kinect* top_kinect = new Kinect();

        top_kinect->addRGBTopic("/amigo/top_kinect/rgb/image_rect_color");
        top_kinect->addRGBTopic("/amigo/top_kinect/rgb/image_color");
        top_kinect->addDepthTopic("/amigo/top_kinect/depth_registered/image");
        top_kinect->addDepthTopic("/amigo/top_kinect/depth_registered/image_rect");
        top_kinect->addRGBCameraInfoTopic("/amigo/top_kinect/rgb/camera_info");
        top_kinect->addDepthCameraInfoTopic("/amigo/top_kinect/depth_registered/camera_info");
        top_kinect->addPointCloudTopic("/amigo/top_kinect/rgb/points");
        top_kinect->addPointCloudTopic("/amigo/top_kinect/depth_registered/points");
        top_kinect->setRGBFrame("/amigo/top_kinect/openni_rgb_optical_frame");
        top_kinect->setDepthFrame("/amigo/top_kinect/openni_rgb_optical_frame");

        // load object models
        tue::filesystem::Crawler crawler(model_dir_ + "/kinect");
        crawler.setIgnoreHiddenDirectories(true);
        crawler.setRecursive(false);

        tue::filesystem::Path p;
        while (crawler.nextPath(p))
        {
            top_kinect->addModel(p.filename(), p.string());
        }

//        //top_kinect->addModel("loy", MODEL_DIR + "/kinect/loy");
//        top_kinect->addModel("coke", model_dir_ + "/kinect/coke_cropped");
//        top_kinect->addModel("cif", model_dir_ + "/kinect/cif_cropped");
//        top_kinect->addModel("tea_pack", model_dir_ + "/kinect/tea_pack_cropped");

//        for(set<string>::iterator it = faces_.begin(); it != faces_.end(); ++it) {
//            top_kinect->addModel("face_" + *it, model_dir_ + "/kinect/face_" + *it);
//        }

//        top_kinect->addModel("drops", model_dir_ + "/kinect/drops_cropped");
//        top_kinect->addModel("marmalade", model_dir_ + "/kinect/marmalade_cropped");
//        top_kinect->addModel("tomato_soup", model_dir_ + "/kinect/tomato_soup_cropped");
//        top_kinect->addModel("cleaner", model_dir_ + "/kinect/cleaner_cropped");

//        top_kinect->addModel("energy_drink", model_dir_ + "/kinect/energy_drink_cropped");
//        top_kinect->addModel("sponge", model_dir_ + "/kinect/sponge_cropped");
//        top_kinect->addModel("veggie_noodles", model_dir_ + "/kinect/veggie_noodles_cropped");

//        top_kinect->addModel("apple_juice", model_dir_ + "/kinect/apple_juice");
//        top_kinect->addModel("beer_bottle", model_dir_ + "/kinect/beer_bottle");
//        top_kinect->addModel("chocolate_milk", model_dir_ + "/kinect/chocolate_milk");
//        top_kinect->addModel("coke_rwc2013", model_dir_ + "/kinect/coke_rwc2013");
//        top_kinect->addModel("cookies", model_dir_ + "/kinect/cookies");
//        top_kinect->addModel("crackers", model_dir_ + "/kinect/crackers");
//        top_kinect->addModel("deodorant", model_dir_ + "/kinect/deodorant");
//        top_kinect->addModel("fanta", model_dir_ + "/kinect/fanta");
//        top_kinect->addModel("fresh_discs", model_dir_ + "/kinect/fresh_discs");
//        top_kinect->addModel("garlic_sauce", model_dir_ + "/kinect/garlic_sauce");
//        top_kinect->addModel("milk", model_dir_ + "/kinect/milk");
//        top_kinect->addModel("orange_juice", model_dir_ + "/kinect/orange_juice");
//        top_kinect->addModel("peanut_butter", model_dir_ + "/kinect/peanut_butter");
//        top_kinect->addModel("seven_up", model_dir_ + "/kinect/seven_up");
//        top_kinect->addModel("tooth_paste", model_dir_ + "/kinect/tooth_paste");

        amigo->registerSensor(top_kinect);
        amigo->getLink("top_kinect/openni_rgb_optical_frame")->addChild(top_kinect);


        LRF* base_lrf = new LRF("/amigo/base_front_laser", "/amigo/base_laser");
        amigo->registerSensor(base_lrf);
        amigo->getLink("base_laser")->addChild(base_lrf);

        LRF* torso_lrf = new LRF("/amigo/torso_laser", "/amigo/torso_laser");
        amigo->registerSensor(torso_lrf);
        amigo->getLink("torso_laser")->addChild(torso_lrf);

        //tf::Transform tf_base_link_to_top_laser;
        //tf_base_link_to_top_laser.setOrigin(tf::Vector3(0.31, 0, 1.0));
        //tf_base_link_to_top_laser.setRotation(tf::Quaternion(0, 0, 0, 1));
        //LRF* laser_range_finder_top_ = new LRF("/top_scan", "/front_laser");
        //this->registerSensor(laser_range_finder_top_, tf_base_link_to_top_laser);

        return amigo;
    } else if (model_name == "kinect") {
        StandaloneKinect* kinect = new StandaloneKinect(nh_, model_dir_);
        return kinect;
    } else if (model_name == "lrf") {
        StandaloneLRF* lrf = new StandaloneLRF(nh_);
        return lrf;
    } else if (model_name == "box") {
        Object* obj = new Object(model_name);
        obj->setShape(geo::Box(geo::Vector3(-0.4, -0.4, 0), geo::Vector3(0.4, 0.4, 1)));
        return obj;
    }
//    else if (model_name == "person") {
//        Object* obj = new Object(model_name);

//        Object* body = new Object("body", id + "-body");
//        body->setShape(ModelParser::getHeightMapFromImage(model_dir_ + "/laser/body.pgm", 1, 0.025));
//        obj->addChild(body, geo::Transform(geo::Matrix3::identity(), geo::Vector3(0, 0, 0.5)));
//        //body->setPose(tf::Vector3(0, 0, 1), tf::Quaternion(0, 0, 0, 1));
//        //obj->addChild(body, tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.5, 0.5, 0.5)));

//        string face_type = "loy";
//        set<string>::iterator it_face_type = faces_.find(id);
//        if (it_face_type != faces_.end()) {
//            face_type = *it_face_type;
//        }

//        Object* face = new Object("face_" + face_type, id + "-face");
//        obj->addChild(face, geo::Transform(geo::Matrix3::identity(), geo::Vector3(0, 0, 1.6)));

//    }

    cout << "Model " << model_name << " not found" << endl;

    return 0;
}

void SimulatorROS::addObject(const std::string& id, Object* obj) {
    simulator_.addObject(id, obj);
}

void SimulatorROS::start() {
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

        simulator_.step(dt);

        if (count % 10 == 0) {
            visualization_msgs::MarkerArray marker_array = getROSVisualizationMessage();
            pub_marker_.publish(marker_array);
        }

        ros::Duration cycle_time = ros::Time::now() - t_start;
        if (cycle_time > max_cycle_time) {
            max_cycle_time = cycle_time;
        }

        //cout << "Max main cycle duration: " << max_cycle_time << " seconds" << endl;

        ++count;
        r.sleep();
    }
}

bool SimulatorROS::setObject(fast_simulator::SetObject::Request& req, fast_simulator::SetObject::Response& res) {

    Object* obj = simulator_.getObject(req.id);

    if (req.action == fast_simulator::SetObject::Request::SET_POSE) {

        if (!obj) {
            obj = getObjectFromModel(req.type, req.id);
            if (!obj) {
                obj = new Object(req.type);
            }
            simulator_.addObject(req.id, obj);
        }

        geo::Transform pose;
        geo::convert(req.pose, pose);
        obj->setPose(pose);

        cout << "Set " << req.id << " (type: " << req.type << "): " << pose << endl;
    } else {
        if (!obj) {
            res.result_msg = "Object with id " + req.id + " does not exist";
            return true;
        }

        if (req.action == fast_simulator::SetObject::Request::DELETE) {
            simulator_.removeObject(req.id);
        } else if (req.action == fast_simulator::SetObject::Request::SET_PARAMS) {
            if (!obj) {
                res.result_msg = "Object with id " + req.id + " does not exist";
                return true;
            }

            for(unsigned int i = 0; i < req.param_names.size(); ++i) {
                obj->setParameter(req.param_names[i], req.param_values[i]);
            }
        }
    }

    return true;
}


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//
//                                      ROS VISUALIZATION
//
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

visualization_msgs::MarkerArray SimulatorROS::getROSVisualizationMessage() {
    map<string, Object*> objects = simulator_.getObjects();

    visualization_msgs::MarkerArray marker_array;

    for(map<string, Object*>::const_iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj) {

        Object& obj = *it_obj->second;

        geo::Transform pose = obj.getAbsolutePose();

        const vector<Object>& children = obj.getChildren();
        for(vector<Object>::const_iterator it_child = children.begin(); it_child != children.end(); ++it_child) {
            const Object& child = *it_child;

            geo::ShapePtr shape = child.getShape();
            if (shape) {
                const geo::Box* box = dynamic_cast<const geo::Box*>(shape.get());
                if (box) {
                    geo::Vector3 size = box->getSize();

                    if (size.x < 20 && size.y < 20 && size.z < 20) { // quick hack to prevent visualization of ground plane

                        visualization_msgs::Marker m_box;
                        m_box.action =  visualization_msgs::Marker::ADD;
                        m_box.header.frame_id = "/map";
                        m_box.header.stamp = ros::Time::now();

                        geo::Transform child_rel_pose = child.getRelativePose();
                        child_rel_pose.setOrigin(child_rel_pose.getOrigin() + box->getCenter());

                        geo::convert(pose * child_rel_pose, m_box.pose);

                        m_box.color.a = 1;
                        m_box.color.r = 0.8;
                        m_box.color.g = 0.8;
                        m_box.color.b = 1;

                        m_box.type = visualization_msgs::Marker::CUBE;

                        m_box.scale.x = size.getX();
                        m_box.scale.y = size.getY();
                        m_box.scale.z = size.getZ();

                        m_box.lifetime = ros::Duration(1.0);

                        marker_array.markers.push_back(m_box);
                    }
                }
            }
        }



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

        m.header.frame_id = "/map";
        geo::convert(pose, m.pose);

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
        geo::convert(pose, m_text.pose);
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
