#include "fast_simulator/Object.h"
#include "fast_simulator/World.h"

#include "fast_simulator/util.h"

using namespace std;

ObjectDescription::ObjectDescription() : parent_(0), bounding_box_(0), shape_(0) {
}

ObjectDescription::~ObjectDescription() {
    delete bounding_box_;
    delete shape_;
}

Object::Object(const string& type) : has_pose_(false), description_(new ObjectDescription()) {
    pose_.setOrigin(tf::Vector3(0, 0, 0));
    pose_.setRotation(tf::Quaternion(0, 0, 0, 1));
    pose_inv_ = pose_.inverse();
    description_->type_ = type;
}

Object::~Object() {
}

Object::Object(const Object& orig) : has_pose_(orig.has_pose_), pose_(orig.pose_), pose_inv_(orig.pose_inv_),
        description_(orig.description_), parts_(orig.parts_) {

}

void Object::addChild(Object* child) {
    child->description_->parent_ = this;
    parts_.push_back(*child);
}

void Object::addChild(Object* child, const tf::Transform& rel_pose) {
    child->pose_ = rel_pose;
    child->pose_inv_ = pose_.inverse();
    child->has_pose_ = true;
    addChild(child);
}

World* Object::getWorldHandle() {
    return &World::getInstance();
}

tf::Transform Object::getAbsolutePose() const {
    if (!description_->parent_) {
        return pose_;
    }
    return description_->parent_->getAbsolutePose() * pose_;
}

void Object::step(double dt) {
    geometry_msgs::Twist& vel = description_->velocity_;

    tf::Quaternion q;
    q.setRPY(vel.angular.x * dt, vel.angular.y * dt, vel.angular.z * dt);

    tf::Transform t1(tf::Quaternion(0, 0, 0, 1), tf::Vector3(vel.linear.x * dt, vel.linear.y * dt, vel.linear.z * dt));
    tf::Transform t2(pose_.getRotation(), tf::Vector3(0, 0, 0));

    tf::Vector3 trans = (t2 * t1).getOrigin();

    pose_.setOrigin(pose_.getOrigin() + trans);
    pose_.setRotation(pose_.getRotation() * q);
}

void Object::setBoundingBox(const Box& box) {
    description_->bounding_box_ = new Box(box);
}

void Object::setShape(const Shape& shape) {
    description_->shape_ = shape.clone();
}

void Object::setPose(const tf::Vector3& pos, const tf::Quaternion& rot) {
    pose_.setOrigin(pos);
    pose_.setRotation(rot);
    pose_inv_ = pose_.inverse();
    has_pose_ = true;
}

const string& Object::getID() const {
    return description_->id_;
}

const string& Object::getType() const {
    return description_->type_;
}

bool Object::intersect(const Ray &r, float t0, float t1, double& distance) const {
    if (getID() == "amigo") {
        return false;
    }

    if (!description_->bounding_box_ && !description_->shape_ && parts_.empty()) {
        return false;
    }

    Ray r_transformed = r;
    if (has_pose_) {
        tf::Transform inv_rot(pose_inv_.getRotation(), tf::Vector3(0, 0, 0));
        r_transformed = Ray(pose_inv_ * r.origin, inv_rot * r.direction);
    }

    r.nr_intersection_calcs_++;

    if (description_->bounding_box_ && !description_->bounding_box_->intersect(r_transformed, t0, t1, distance)) {
        r.nr_intersection_calcs_ += r_transformed.nr_intersection_calcs_;
        return false;
    }

    //t0 = distance - 0.1;
    bool has_intersect = false;
    distance = t1;

    double dist;
    if (description_->shape_ && description_->shape_->intersect(r_transformed, t0, t1, dist)) {
        has_intersect = true;
        distance = min(distance, dist);
    }

    for(vector<Object>::const_iterator it_part = parts_.begin(); it_part != parts_.end(); ++it_part) {
        const Object& obj = *it_part;
        if (obj.intersect(r_transformed, t0, distance, dist)) {
            has_intersect = true;
            distance = min(distance, dist);
        }
    }

    r.nr_intersection_calcs_ += r_transformed.nr_intersection_calcs_;
    return has_intersect;
}

#include <sstream>
#include "fast_simulator/util.h"

string Object::toString(const string& indent) const {
    stringstream s;
    s << indent << "Object " << getID() << endl
      << indent << "    " << "(" << pose_.getOrigin().getX() << ", " << pose_.getOrigin().getY() << ", " << pose_.getOrigin().getZ() << ")" << endl;

    for(vector<Object>::const_iterator it_part = parts_.begin(); it_part != parts_.end(); ++it_part) {
        s << it_part->toString(indent + "    ");
    }
    return s.str();
}
