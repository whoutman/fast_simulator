#include "fast_simulator/Object.h"
#include "fast_simulator/World.h"

#include "fast_simulator/util.h"

using namespace std;

Object::Object(const string& type) : type_(type), parent_(0), bounding_box_(0), shape_(0), has_pose_(false) {
    pose_.setOrigin(tf::Vector3(0, 0, 0));
    pose_.setRotation(tf::Quaternion(0, 0, 0, 1));
}

Object::~Object() {
    // delete parts
    for(vector<Object*>::const_iterator it_part = parts_.begin(); it_part != parts_.end(); ++it_part) {
        delete *it_part;
    }

    delete bounding_box_;
    delete shape_;
}

void Object::addChild(Object* child) {
    child->parent_ = this;
    parts_.push_back(child);
}

void Object::addChild(Object* child, const tf::Transform& rel_pose) {
    child->pose_ = rel_pose;
    child->has_pose_ = true;
    addChild(child);
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

void Object::setShape(const Shape& shape) {
    shape_ = shape.clone();
}

void Object::setPose(const tf::Vector3& pos, const tf::Quaternion& rot) {
    pose_.setOrigin(pos);
    pose_.setRotation(rot);
    has_pose_ = true;
}

const string& Object::getID() const {
    return id_;
}

const string& Object::getType() const {
    return type_;
}

bool Object::intersect(const Ray &r, float t0, float t1, double& distance) const {
    if (!bounding_box_ && !shape_ && parts_.empty()) {
        return false;
    }

    Ray r_transformed = r;
    if (has_pose_) {
        tf::Transform inv = this->pose_.inverse();
        tf::Transform inv_rot(inv.getRotation(), tf::Vector3(0, 0, 0));

        r_transformed = Ray(inv * r.origin, inv_rot * r.direction);
    }

    if (bounding_box_ && !bounding_box_->intersect(r_transformed, t0, t1, distance)) {
        return false;
    }

    //t0 = distance - 0.1;
    bool has_intersect = false;
    distance = t1;

    double dist;
    if (shape_ && shape_->intersect(r_transformed, t0, t1, dist)) {
        has_intersect = true;
        distance = min(distance, dist);
    }

    for(vector<Object*>::const_iterator it_part = parts_.begin(); it_part != parts_.end(); ++it_part) {
        Object* obj = *it_part;
        if (obj->intersect(r_transformed, t0, t1, dist)) {
            has_intersect = true;
            distance = min(distance, dist);
        }
    }
    return has_intersect;
}


