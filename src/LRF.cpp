#include "fast_simulator/LRF.h"
#include "fast_simulator/World.h"

LRF::LRF(const std::string& topic, const std::string& frame_id) : frame_id_(frame_id) {
    ros::NodeHandle nh;
    pub_laser_scan = nh.advertise<sensor_msgs::LaserScan>(topic, 10);

    scan.header.frame_id = frame_id_;
    scan.angle_min = -2.09439492226;
    scan.angle_max = 2.09439492226;
    scan.angle_increment = 0.00614192103967;
    scan.time_increment = 0;
    scan.scan_time = 0;
    scan.range_min = 0.5;
    scan.range_max = 10.0;

    laser_ray_deltas_.clear();
    for(double angle = scan.angle_min; angle <= scan.angle_max; angle += scan.angle_increment) {
        tf::Quaternion q;
        q.setRPY(0, 0, angle);

        tf::Transform t;
        t.setOrigin(tf::Vector3(0, 0, 0));
        t.setRotation(q);

        laser_ray_deltas_.push_back(t * tf::Vector3(1, 0, 0));

        //cout << laser_ray_deltas_.back().getX() << ", " << laser_ray_deltas_.back().getY() << ", " << laser_ray_deltas_.back().getZ() << endl;
    }

    //cout << laser_ray_deltas_.size() << endl;
}

LRF::~LRF() {

}

void LRF::publishScan() {
    std::cout << "LRF::publishScan" << std::endl;

    scan.header.stamp = ros::Time::now();

    scan.ranges.clear();
    scan.intensities.clear();

    tf::Transform tf_map_to_laser = getAbsolutePose();

    tf::Vector3 laser_origin = tf_map_to_laser.getOrigin();

    for(unsigned int i = 0; i < laser_ray_deltas_.size(); ++i) {
        tf::Vector3 dir = tf::Transform(tf_map_to_laser.getRotation()) * laser_ray_deltas_[i];
        Ray r(laser_origin, dir);
        double distance;
        if (!getWorldHandle()->intersect(r, 0, 9, distance)) {
            distance = 9;
        }

        scan.ranges.push_back(distance);
        scan.intensities.push_back(101);
    }

    pub_laser_scan.publish(scan);
}
