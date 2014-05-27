#include "fast_simulator/LRF.h"
#include "fast_simulator/World.h"

#include <limits>

#include <geolib/Ray.h>

LRF::LRF(const std::string& topic, const std::string& frame_id) : frame_id_(frame_id) {
    rate_ = 40;

    lrf_.setAngleLimits(-2, 2);
    lrf_.setRangeLimits(0.05, 60.0);
    lrf_.setNumBeams(918);

    ros::NodeHandle nh;
    pub_laser_scan = nh.advertise<sensor_msgs::LaserScan>(topic, 10);

    scan.header.frame_id = frame_id_;
    scan.angle_min = lrf_.getAngleMin();
    scan.angle_max = lrf_.getAngleMax();
    scan.angle_increment = lrf_.getAngleIncrement();
    scan.time_increment = 0;
    scan.scan_time = 0;
    scan.range_min = lrf_.getRangeMin();
    scan.range_max = lrf_.getRangeMax();
}

LRF::~LRF() {

}

void LRF::step(World& world) {
    scan.header.stamp = ros::Time::now();
    scan.ranges.clear();

    geo::Transform laser_pose = getAbsolutePose();

    std::vector<double> ranges;

    const std::map<std::string, Object*>& objects = world.getObjects();

    for(std::map<std::string, Object*>::const_iterator it_obj = objects.begin(); it_obj != objects.end(); ++it_obj) {
        Object* obj = it_obj->second;

        geo::ShapePtr shape = obj->getShape();
        if (shape) {
            lrf_.render(*shape, laser_pose, obj->getAbsolutePose(), ranges);
        }

        std::vector<Object*> children;
        obj->getChildrenRecursive(children);

        for(std::vector<Object*>::const_iterator it = children.begin(); it != children.end(); ++it) {
            const Object& child = **it;
            geo::ShapePtr child_shape = child.getShape();
            if (child_shape) {
                lrf_.render(*child_shape, laser_pose, child.getAbsolutePose(), ranges);
            }
        }
    }

    scan.ranges.resize(ranges.size());
    scan.intensities.clear(); //resize(ranges.size());

    for(unsigned int i = 0; i < ranges.size(); ++i) {
        double range = ranges[i];
        if (range == 0) {
            range = std::numeric_limits<double>::infinity();
        }
        scan.ranges[i] = range;
//        scan.intensities[i] = 101;
    }

    pub_laser_scan.publish(scan);
}
