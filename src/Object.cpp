#include "fast_simulator/Object.h"
#include "fast_simulator/World.h"

using namespace std;

Object::Object() : parent_(0), current_goal_(-1) {

}

Object::~Object() {
    // delete parts
    for(vector<Object*>::const_iterator it_part = parts_.begin(); it_part != parts_.end(); ++it_part) {
        delete *it_part;
    }
}

void Object::addChild(const string& type, double dx, double dy, double dz) {
    Object* child = new Object();
    child->id_ = "";
    child->type_ = type;
    child->parent_ = this;
    child->pose_.setOrigin(tf::Vector3(dx, dy, dz));
    child->pose_.setRotation(tf::Quaternion(0, 0, 0, 1));
    child->pose_.frame_id_ = "PARENT";
    parts_.push_back(child);
}

World* Object::getWorldHandle() {
    return world_;
}

tf::Stamped<tf::Pose> Object::getAbsolutePose() const {
    if (!parent_) {
        return pose_;
    }

    tf::Stamped<tf::Pose> pose = parent_->getAbsolutePose();
    pose.setOrigin(pose.getOrigin() + pose_.getOrigin());
    return pose;
}

void Object::step(double dt) {
    tf::Quaternion q;
    q.setRPY(velocity_.angular.x * dt, velocity_.angular.y * dt, velocity_.angular.z * dt);

    tf::Transform t1(tf::Quaternion(0, 0, 0, 1), tf::Vector3(velocity_.linear.x * dt, velocity_.linear.y * dt, velocity_.linear.z * dt));
    tf::Transform t2(pose_.getRotation(), tf::Vector3(0, 0, 0));

    tf::Vector3 trans = (t2 * t1).getOrigin();

    pose_.setOrigin(pose_.getOrigin() + trans);

    pose_.setRotation(pose_.getRotation() * q);
}


