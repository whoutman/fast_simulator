#include "fast_simulator/Object.h"
#include "fast_simulator/World.h"

using namespace std;

Object::Object() : parent_(0), current_goal_(-1), bounding_box_(0), shape_(0) {

}

Object::~Object() {
    // delete parts
    for(vector<Object*>::const_iterator it_part = parts_.begin(); it_part != parts_.end(); ++it_part) {
        delete *it_part;
    }

    delete bounding_box_;
    delete shape_;
}

void Object::addChild(Object* child, const tf::Transform& rel_pose) {
    child->pose_ = rel_pose;
    child->parent_ = this;
    parts_.push_back(child);
}

World* Object::getWorldHandle() {
    return &World::getInstance();
}

tf::Transform Object::getAbsolutePose() const {
    if (!parent_) {
        return pose_;
    }
    return parent_->getAbsolutePose() * pose_;
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

void Object::setBoundingBox(const Box& box) {
    bounding_box_ = new Box(box);
}

void Object::setShape(const Box& box) {
    shape_ = new Box(box);
}

bool Object::intersect(const Ray &r, float t0, float t1, double& distance) const {
    if (bounding_box_ && !bounding_box_->intersect(r, t0, t1, distance)) {
        return false;
    }

    bool has_intersect = false;
    distance = t1;

    double dist;
    if (shape_ && shape_->intersect(r, t0, t1, dist)) {
        has_intersect = true;
        distance = min(distance, dist);
    }

    for(vector<Object*>::const_iterator it_part = parts_.begin(); it_part != parts_.end(); ++it_part) {
        Object* obj = *it_part;
        if (obj->intersect(r, t0, t1, dist)) {
            has_intersect = true;
            distance = min(distance, dist);
        }
    }
    return has_intersect;
}


