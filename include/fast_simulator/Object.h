#ifndef _FAST_SIMULATOR_OBJECT_H_
#define _FAST_SIMULATOR_OBJECT_H_

#include <tf/tf.h>

#include <string>
#include <vector>

// TODO: better data structure
#include <geometry_msgs/Twist.h>

class World;

class Object {

    friend class World;

public:

    Object();

    virtual ~Object();

    void addChild(const std::string& type, double dx, double dy, double dz);

    tf::Stamped<tf::Pose> getAbsolutePose() const;

    virtual void step(double dt);

    World* getWorldHandle();

    Object* parent_;

    std::string id_;

    std::string type_;

    tf::Stamped<tf::Pose> pose_;

    geometry_msgs::Twist velocity_;

    int current_goal_;

    std::vector<tf::Stamped<tf::Pose> > path_;

    std::vector<Object*> parts_;

    int visualization_id_;

private:

    World* world_;

};

#endif
