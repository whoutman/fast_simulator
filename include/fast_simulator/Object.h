#ifndef _FAST_SIMULATOR_OBJECT_H_
#define _FAST_SIMULATOR_OBJECT_H_

#include <tf/tf.h>

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

// TODO: better data structure
#include <geometry_msgs/Twist.h>

#include <geolib/datatypes.h>
#include "fast_simulator/Event.h"

#include <queue>

class World;

class Object {

    friend class World;

    friend class Kinect; // for testing

public:

    Object(const std::string& type = "", const std::string& id = "");

    virtual ~Object();

    Object(const Object& orig);

    static Object* fromModel(const Object& model);

    void addChild(Object* child);

    void addChild(Object* child, const geo::Transform& pose);

    const std::vector<Object>& getChildren() const;

    void getChildrenRecursive(std::vector<Object*>& objects);

    geo::Transform getRelativePose() const;

    geo::Transform getAbsolutePose() const;

    virtual void step(double dt);

    const std::string& getID() const;

//    void setBoundingBox(const geo::Box& box);

    void setShape(const geo::Shape& shape);

    geo::ShapePtr getShape() const;

    void setPose(const geo::Transform& pose);

    void setPath(const std::vector<geo::Transform>& path, double path_vel)
    {
        path_ = std::queue<geo::Transform>();
        for(unsigned int i = 0; i < path.size(); ++i)
            path_.push(path[i]);
        path_vel_ = path_vel;
    }

    const std::string& getType() const;

    bool intersect(const geo::Ray& r, float t0, float t1, double& distance) const;

    void getBoundingBox(tf::Vector3 &min, tf::Vector3 &max) const;    

    virtual void setParameter(const std::string& param_name, const std::string& value);

    std::string toString(const std::string &indent = "") const;

private:

    bool has_pose_;

    geo::Transform pose_;

    geo::Transform pose_inv_;

    std::queue<geo::Transform> path_;

    double path_vel_;

protected:

    std::string id_;

    std::string type_;

    Object* parent_;

    geo::ShapePtr shape_;

    geometry_msgs::Twist velocity_;

    // scheduler

    std::map<std::string, ros::Time> scheduled_events_;

    std::vector<Object> parts_;

    /*

    int current_goal_;

    std::vector<tf::Stamped<tf::Pose> > path_;

    int visualization_id_;
    */

};

#endif
