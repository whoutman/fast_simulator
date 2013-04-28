#ifndef _FAST_SIMULATOR_OBJECT_H_
#define _FAST_SIMULATOR_OBJECT_H_

#include <tf/tf.h>

#include <string>
#include <vector>

// TODO: better data structure
#include <geometry_msgs/Twist.h>

#include "fast_simulator/Box.h"
#include "fast_simulator/Event.h"

class World;

class Object {

    friend class World;

public:

    Object(const std::string& type = "");

    virtual ~Object();

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

protected:

    std::string id_;

    std::string type_;

    Object* parent_;

    Box* bounding_box_;

    Shape* shape_;

    bool has_pose_;

    tf::Transform pose_;

    std::vector<Object*> parts_;

    geometry_msgs::Twist velocity_;

    // scheduler

    std::map<std::string, ros::Time> scheduled_events_;


    /*


    int current_goal_;

    std::vector<tf::Stamped<tf::Pose> > path_;

    int visualization_id_;
    */

};

#endif
