#ifndef _FAST_SIMULATOR_OBJECT_H_
#define _FAST_SIMULATOR_OBJECT_H_

#include <tf/tf.h>

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

// TODO: better data structure
#include <geometry_msgs/Twist.h>

#include "fast_simulator/Box.h"
#include "fast_simulator/Event.h"

class World;
class Object;

class ObjectDescription {

public:

    ObjectDescription();

    virtual ~ObjectDescription();

    std::string id_;

    std::string type_;

    Object* parent_;

    Box* bounding_box_;

    Shape* shape_;

    geometry_msgs::Twist velocity_;

    // scheduler

    std::map<std::string, ros::Time> scheduled_events_;

};

class Object {

    friend class World;

    friend class Kinect; // for testing

public:

    Object(const std::string& type = "");

    virtual ~Object();

    Object(const Object& orig);

    void addChild(Object* child);

    void addChild(Object* child, const tf::Transform& rel_pose);

    tf::Transform getAbsolutePose() const;

    virtual void step(double dt);

    World* getWorldHandle();

    const std::string& getID() const;

    void setBoundingBox(const Box& box);

    void setShape(const Shape& box);

    void setPose(const tf::Vector3& pos, const tf::Quaternion& rot);

    const std::string& getType() const;

    bool intersect(const Ray &r, float t0, float t1, double& distance) const;

    std::string toString(const std::string &indent = "") const;

protected:

    bool has_pose_;

    tf::Transform pose_;

    boost::shared_ptr<ObjectDescription> description_;

    std::vector<Object> parts_;

    /*

    int current_goal_;

    std::vector<tf::Stamped<tf::Pose> > path_;

    int visualization_id_;
    */

};

#endif
