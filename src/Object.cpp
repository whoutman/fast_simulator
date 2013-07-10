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

Object::Object(const string& type, const std::string& id) : has_pose_(false), description_(new ObjectDescription()) {
    pose_.setOrigin(tf::Vector3(0, 0, 0));
    pose_.setRotation(tf::Quaternion(0, 0, 0, 1));
    pose_inv_ = pose_.inverse();
    description_->type_ = type;
    description_->id_ = id;
}

Object::~Object() {
}

Object::Object(const Object& orig) : has_pose_(orig.has_pose_), pose_(orig.pose_), pose_inv_(orig.pose_inv_),
        description_(orig.description_), parts_(orig.parts_) {

}

Object* Object::fromModel(const Object& model) {
    // TODO: check if this is OK!
    Object* obj = new Object(model);
    obj->description_ = boost::shared_ptr<ObjectDescription>(new ObjectDescription(*model.description_));
    return obj;
}

void Object::addChild(Object* child) {
    child->description_->parent_ = this;
    parts_.push_back(*child);
}

//void Object::addChild(Object* child, const tf::Transform& rel_pose) {
void Object::addChild(Object* child, const tf::Vector3& pos, const tf::Quaternion& rot) {
    child->setPose(pos, rot);
    addChild(child);
}

const std::vector<Object>& Object::getChildren() const {
    return parts_;
}

tf::Transform Object::getRelativePose() const {
    return pose_;
}

tf::Transform Object::getAbsolutePose() const {
    if (!description_->parent_) {
        return pose_;
    }
    return description_->parent_->getAbsolutePose() * pose_;    // TODO: parent_ may be invalid, FIX!
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

const Shape* Object::getShape() const {
    if (!description_) {
        return 0;
    }
    return description_->shape_;
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

void Object::getChildrenRecursive(std::vector<Object*>& objects) {
    for(vector<Object>::iterator it_part = parts_.begin(); it_part != parts_.end(); ++it_part) {
        Object& obj = *it_part;
        objects.push_back(&obj);
        obj.getChildrenRecursive(objects);
    }
}

void Object::getBoundingBox(tf::Vector3& min, tf::Vector3& max) const {
    description_->shape_->getBoundingBox(min, max);
    for(vector<Object>::const_iterator it_part = parts_.begin(); it_part != parts_.end(); ++it_part) {
        const Object& obj = *it_part;

        tf::Vector3 min2, max2;
        obj.getBoundingBox(min2, max2);
        min.setX(std::min(min.x(), min2.x()));
        min.setY(std::min(min.y(), min2.y()));
        min.setZ(std::min(min.z(), min2.z()));

        max.setX(std::max(max.x(), max2.x()));
        max.setY(std::max(max.y(), max2.y()));
        max.setZ(std::max(max.z(), max2.z()));
    }
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
