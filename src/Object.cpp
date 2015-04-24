#include "fast_simulator/Object.h"
#include "fast_simulator/World.h"

#include "geolib/Shape.h"

#include "fast_simulator/util.h"

using namespace std;

Object::Object(const string& type, const std::string& id) : has_pose_(false), id_(id), type_(type), parent_(0) {   
    pose_ = geo::Transform(geo::Matrix3::identity(), geo::Vector3(0, 0, 0));
    pose_inv_ = pose_.inverse();
}

Object::~Object() {
}

Object::Object(const Object& orig) : has_pose_(orig.has_pose_), pose_(orig.pose_), pose_inv_(orig.pose_inv_),
        id_(orig.id_), type_(orig.type_), parent_(orig.parent_), shape_(orig.shape_), velocity_(orig.velocity_),
        scheduled_events_(orig.scheduled_events_), parts_(orig.parts_) {

}

Object* Object::fromModel(const Object& model) {
    // TODO: check if this is OK!
    Object* obj = new Object(model);
    //obj->description_ = boost::shared_ptr<ObjectDescription>(new ObjectDescription(*model.description_));
    return obj;
}

void Object::addChild(Object* child) {
    child->parent_ = this;
    parts_.push_back(*child);
}

//void Object::addChild(Object* child, const tf::Transform& rel_pose) {
void Object::addChild(Object* child, const geo::Transform& pose) {
    child->setPose(pose);
    addChild(child);
}

const std::vector<Object>& Object::getChildren() const {
    return parts_;
}

geo::Transform Object::getRelativePose() const {
    return pose_;
}

geo::Transform Object::getAbsolutePose() const {
    if (!parent_) {
        return pose_;
    }
    return parent_->getAbsolutePose() * pose_;    // TODO: parent_ may be invalid, FIX!
}

void Object::step(double dt) {
//    tf::Quaternion q;
//    q.setRPY(velocity_.angular.x * dt, velocity_.angular.y * dt, velocity_.angular.z * dt);

//    tf::Transform t1(tf::Quaternion(0, 0, 0, 1), tf::Vector3(velocity_.linear.x * dt, velocity_.linear.y * dt, velocity_.linear.z * dt));
//    tf::Transform t2(pose_.getRotation(), tf::Vector3(0, 0, 0));

//    tf::Vector3 trans = (t2 * t1).getOrigin();

    geo::Vector3 trans(0, 0, 0);
    if (!path_.empty())
    {
        const geo::Transform& goal = path_.front();
        geo::Vector3 diff = goal.t - pose_.t;

        double dist = diff.length();
        geo::Vector3 diff_n = diff / dist;

        double traveled_dist = dt * path_vel_;

        if (dist < traveled_dist)
        {
            pose_.t = goal.t;
            path_.pop();
        }
        else
        {
            trans = traveled_dist * diff_n;
        }
    }
    else
    {
        trans = pose_.getBasis() * geo::Vector3(velocity_.linear.x * dt, velocity_.linear.y * dt, velocity_.linear.z * dt);
    }

    geo::Matrix3 rot;
    rot.setRPY(velocity_.angular.x * dt, velocity_.angular.y * dt, velocity_.angular.z * dt);

    pose_.setOrigin(pose_.getOrigin() + trans);
    pose_.setBasis(pose_.getBasis() * rot);
}

void Object::setShape(const geo::Shape& shape) {
    shape_ = geo::ShapePtr(shape.clone());
}

geo::ShapePtr Object::getShape() const {
    return shape_;
}

void Object::setPose(const geo::Transform& pose) {
    pose_ = pose;
    pose_inv_ = pose_.inverse();
    has_pose_ = true;
}

const string& Object::getID() const {
    return id_;
}

const string& Object::getType() const {
    return type_;
}

bool Object::intersect(const geo::Ray& r, float t0, float t1, double& distance) const {
    if (getID() == "amigo") {
        return false;
    }

    if (!shape_ && parts_.empty()) {
        return false;
    }

    geo::Ray r_transformed = r;
    if (has_pose_) {
        r_transformed = geo::Ray(pose_inv_ * r.origin_, pose_inv_.getBasis() * r.direction_);
    }

//    r.nr_intersection_calcs_++;

//    if (description_->bounding_box_ && !description_->bounding_box_->intersect(r_transformed, t0, t1, distance)) {
//        r.nr_intersection_calcs_ += r_transformed.nr_intersection_calcs_;
//        return false;
//    }

    //t0 = distance - 0.1;
    bool has_intersect = false;
    distance = t1;

    double dist;
    if (shape_ && shape_->intersect(r_transformed, t0, t1, dist)) {
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

//    r.nr_intersection_calcs_ += r_transformed.nr_intersection_calcs_;
    return has_intersect;
}

void Object::getChildrenRecursive(std::vector<Object*>& objects) {
    for(vector<Object>::iterator it_part = parts_.begin(); it_part != parts_.end(); ++it_part) {
        Object& obj = *it_part;
        objects.push_back(&obj);
        obj.getChildrenRecursive(objects);
    }
}

//void Object::getBoundingBox(tf::Vector3& min, tf::Vector3& max) const {
//    description_->shape_->getBoundingBox(min, max);
//    for(vector<Object>::const_iterator it_part = parts_.begin(); it_part != parts_.end(); ++it_part) {
//        const Object& obj = *it_part;

//        tf::Vector3 min2, max2;
//        obj.getBoundingBox(min2, max2);
//        min.setX(std::min(min.x(), min2.x()));
//        min.setY(std::min(min.y(), min2.y()));
//        min.setZ(std::min(min.z(), min2.z()));

//        max.setX(std::max(max.x(), max2.x()));
//        max.setY(std::max(max.y(), max2.y()));
//        max.setZ(std::max(max.z(), max2.z()));
//    }
//}

void Object::setParameter(const std::string& param_name, const std::string& value) {

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
