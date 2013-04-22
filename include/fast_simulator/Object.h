#ifndef _FAST_SIMULATOR_OBJECT_H_
#define _FAST_SIMULATOR_OBJECT_H_

#include <tf/tf.h>

#include <string>
#include <vector>

// TODO: better data structure
#include <geometry_msgs/Twist.h>

#include "fast_simulator/Box.h"

class World;

class Object {

    friend class World;

public:

    Object();

    virtual ~Object();

    void addChild(Object* child, const tf::Transform& rel_pose);

    tf::Transform getAbsolutePose() const;

    virtual void step(double dt);

    World* getWorldHandle();

    Object* parent_;

    std::string id_;

    std::string type_;

    tf::Transform pose_;

    geometry_msgs::Twist velocity_;

    int current_goal_;

    std::vector<tf::Stamped<tf::Pose> > path_;

    std::vector<Object*> parts_;

    int visualization_id_;

    void setBoundingBox(const Box& box);

    void setShape(const Box& box);

    bool intersect(const Ray &r, float t0, float t1, double& distance) const;

protected:

    Box* bounding_box_;

    Box* shape_;

};

#endif
