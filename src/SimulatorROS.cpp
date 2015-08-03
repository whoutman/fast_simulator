/*

    By Sjoerd van den Dries (2013)

    TODO:
       - Get rid of counting and replace with event queue
       - Parallelize controllers from sensors

*/

#include "fast_simulator/SimulatorROS.h"

#include "fast_simulator/Amigo.h"
#include "fast_simulator/Sergio.h"
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

// Heightmap loading
#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

SimulatorROS::SimulatorROS(ros::NodeHandle& nh, const std::string& model_file, const std::string& model_dir)
    : nh_(nh), model_parser_(new ModelParser(model_file, model_dir)), model_dir_(model_dir)
{
    pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("/fast_simulator/visualization", 10);
    srv_set_object_ = nh_.advertiseService("/fast_simulator/set_object", &SimulatorROS::setObject, this);

}

// ----------------------------------------------------------------------------------------------------

SimulatorROS::~SimulatorROS()
{
    srv_set_object_.shutdown();
    delete model_parser_;
}

// ----------------------------------------------------------------------------------------------------

void SimulatorROS::configure(tue::Configuration& config)
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

            // - - - - - - - - - - - - - - - - - - - - - - - -
            // Load pose

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

            // - - - - - - - - - - - - - - - - - - - - - - - -
            // Load type

            std::string type;
            config.value("type", type, tue::OPTIONAL);

            Object* obj = 0;
            if (type == "kinect")
            {
                std::string topic, frame_id;
                if (config.value("topic", topic) && config.value("frame", frame_id))
                {
                    StandaloneKinect* kinect = new StandaloneKinect(nh_, topic, frame_id, model_dir_);
                    obj = kinect;
                }
            }
            else
            {
                if (type.empty())
                {
                    obj = new Object("", id);
                }
                else
                {
                    obj = getObjectFromModel(type, id);
                }

            }

            // - - - - - - - - - - - - - - - - - - - - - - - -
            // Load shape

            if (config.readGroup("shape"))
            {
                std::string shape_type;
                config.value("type", shape_type);

                if (shape_type == "heightmap")
                {
                    double height, resolution;
                    std::string image_filename;

                    if (config.value("image", image_filename) && config.value("height", height)
                            && config.value("resolution", resolution))
                    {
                        cv::Mat img = cv::imread(image_filename, CV_LOAD_IMAGE_GRAYSCALE);
                        if (img.data)
                        {
                            // Create shape from heightmap

                            int h = img.rows;
                            int w = img.cols;

                            std::vector<std::vector<double> > grid(w, std::vector<double>(h, 0));
                            for(unsigned int y = 0; y < h; ++y)
                                for(unsigned int x = 0; x < w; ++x)
                                    grid[x][h - y - 1] = (height * (255 - img.at<unsigned char>(y, x))) / 255;

                            geo::Shape shape = geo::HeightMap::fromGrid(grid, resolution);

                            for(unsigned int i = 0; i < shape.getMesh().getPoints().size(); ++i)
                            {
                                std::cout << shape.getMesh().getPoints()[i] << std::endl;
                            }

                            std::cout << "Walls have: " << shape.getMesh().getTriangleIs().size() << " triangles" << std::endl;

                            obj->setShape(shape);
                        }
                        else
                        {
                            config.addError("Could not read heightmap image '" + image_filename + "'");
                        }
                    }
                }
                else
                {
                    config.addError("Unknown shape type: '" + type + "'");
                }

                config.endGroup();
            }

            // - - - - - - - - - - - - - - - - - - - - - - - -
            // Add object

            if (obj)
            {
                std::cout << "Added object: id = '" << id << "', type = '" << type << "', pose = " << pose << std::endl;

                obj->setPose(pose);
                addObject(id, obj);
            }

        }

        config.endArray();
    }
}

// ----------------------------------------------------------------------------------------------------

void SimulatorROS::parseModelFile(const std::string& filename, const std::string& model_dir)
{
    faces_.insert("loy");
    faces_.insert("tim");
    faces_.insert("rob");
    faces_.insert("erik");
    faces_.insert("sjoerd");
}

// ----------------------------------------------------------------------------------------------------

Object* SimulatorROS::getObjectFromModel(const std::string& model_name, const std::string& id)
{
    if (model_name == "pico") {
        Pico* pico = new Pico(nh_);

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
        Amigo* amigo = new Amigo(nh_);

        // add kinect
        Kinect* top_kinect = new Kinect();

        top_kinect->setRGBFrame("/amigo/top_kinect/openni_rgb_optical_frame");
        top_kinect->setDepthFrame("/amigo/top_kinect/openni_rgb_optical_frame");
        top_kinect->setRGBDName("/amigo/top_kinect/rgbd");

        // load object models
        tue::filesystem::Crawler crawler(model_dir_ + "/kinect");
        crawler.setIgnoreHiddenDirectories(true);
        crawler.setRecursive(false);

        tue::filesystem::Path p;
        while (crawler.nextPath(p))
        {
            top_kinect->addModel(p.filename(), p.string());
        }

        amigo->registerSensor(top_kinect);
        amigo->getLink("top_kinect/openni_rgb_optical_frame")->addChild(top_kinect);


        LRF* base_lrf = new LRF("/amigo/base_laser/scan", "/amigo/base_laser");
        amigo->registerSensor(base_lrf);
        amigo->getLink("base_laser")->addChild(base_lrf);

        LRF* torso_lrf = new LRF("/amigo/torso_laser/scan", "/amigo/torso_laser");
        amigo->registerSensor(torso_lrf);
        amigo->getLink("torso_laser")->addChild(torso_lrf);

        //tf::Transform tf_base_link_to_top_laser;
        //tf_base_link_to_top_laser.setOrigin(tf::Vector3(0.31, 0, 1.0));
        //tf_base_link_to_top_laser.setRotation(tf::Quaternion(0, 0, 0, 1));
        //LRF* laser_range_finder_top_ = new LRF("/top_scan", "/front_laser");
        //this->registerSensor(laser_range_finder_top_, tf_base_link_to_top_laser);

        return amigo;
    } else if (model_name == "sergio") {
        Sergio* sergio = new Sergio(nh_);

        // add kinect
        Kinect* top_kinect = new Kinect();

        top_kinect->setRGBFrame("/sergio/top_kinect/openni_rgb_optical_frame");
        top_kinect->setDepthFrame("/sergio/top_kinect/openni_rgb_optical_frame");
        top_kinect->setRGBDName("/sergio/top_kinect/rgbd");

        // load object models
        tue::filesystem::Crawler crawler(model_dir_ + "/kinect");
        crawler.setIgnoreHiddenDirectories(true);
        crawler.setRecursive(false);

        tue::filesystem::Path p;
        while (crawler.nextPath(p))
        {
            top_kinect->addModel(p.filename(), p.string());
        }

        sergio->registerSensor(top_kinect);
        sergio->getLink("top_kinect/openni_rgb_optical_frame")->addChild(top_kinect);

        LRF* base_lrf = new LRF("/sergio/base_laser/scan", "/sergio/base_laser");
        sergio->registerSensor(base_lrf);
        sergio->getLink("base_laser")->addChild(base_lrf);

        LRF* torso_lrf = new LRF("/sergio/torso_laser/scan", "/sergio/torso_laser");
        sergio->registerSensor(torso_lrf);
        sergio->getLink("torso_laser")->addChild(torso_lrf);

        return sergio;
    } else if (model_name == "kinect") {
        StandaloneKinect* kinect = new StandaloneKinect(nh_, "/kinect/rgbd", "/kinect/frame", model_dir_);
        return kinect;
    } else if (model_name == "lrf") {
        StandaloneLRF* lrf = new StandaloneLRF(nh_);
        return lrf;
    } else if (model_name == "box") {
        Object* obj = new Object(model_name);
        obj->setShape(geo::Box(geo::Vector3(-0.4, -0.4, 0), geo::Vector3(0.4, 0.4, 1)));
        return obj;
    }
    else if (model_name == "loy" || model_name == "rob" || model_name == "tim" || model_name == "erik" || model_name == "sjoerd")
    {
        Object* obj = new Object(model_name);

        Object* body = new Object("body", id + "-body");
        body->setShape(geo::Box(geo::Vector3(-0.2, -0.2, 0), geo::Vector3(0.2, 0.2, 1.8)));
        obj->addChild(body, geo::Transform(geo::Matrix3::identity(), geo::Vector3(0, 0, 0)));

        Object* face = new Object(model_name + "-face", id + "-face");
        obj->addChild(face, geo::Transform(geo::Matrix3::identity(), geo::Vector3(0, 0, 1.7)));

        return obj;
    }

    std::string parse_error;
    Object* obj = model_parser_->parse(model_name, id, parse_error);
    if (!parse_error.empty()) {
        ROS_ERROR("While parsing model file: %s", parse_error.c_str());
    }

    if (obj) {
        return obj;
    }

    std::cout << "Model " << model_name << " not found" << std::endl;

    return 0;
}

// ----------------------------------------------------------------------------------------------------

void SimulatorROS::addObject(const std::string& id, Object* obj)
{
    simulator_.addObject(id, obj);
}

// ----------------------------------------------------------------------------------------------------

void SimulatorROS::start()
{
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

// ----------------------------------------------------------------------------------------------------

bool SimulatorROS::setObject(fast_simulator::SetObject::Request& req, fast_simulator::SetObject::Response& res)
{
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

        std::cout << "Set " << req.id << " (type: " << req.type << "): " << pose << std::endl;
    }
    else if (req.action == fast_simulator::SetObject::Request::SET_PATH)
    {
        std::vector<geo::Transform> path;
        for(unsigned int i = 0; i < req.path.size(); ++i)
        {
            geo::Transform pose;
            geo::convert(req.path[i], pose);
            path.push_back(pose);
        }

        obj->setPath(path, req.path_velocity);
    }
    else
    {
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

// ----------------------------------------------------------------------------------------------------

visualization_msgs::MarkerArray SimulatorROS::getROSVisualizationMessage()
{
    std::map<std::string, Object*> objects = simulator_.getObjects();

    visualization_msgs::MarkerArray marker_array;

    for(std::map<std::string, Object*>::const_iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj) {

        Object& obj = *it_obj->second;

        geo::Transform pose = obj.getAbsolutePose();

        const std::vector<Object>& children = obj.getChildren();
        for(std::vector<Object>::const_iterator it_child = children.begin(); it_child != children.end(); ++it_child) {
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
