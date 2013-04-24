#include "fast_simulator/Sonar.h"
#include "fast_simulator/World.h"

Sonar::Sonar(const std::string& topic, const std::string& frame_id) : frame_id_(frame_id) {
    ros::NodeHandle nh;
    pub_ = nh.advertise<sensor_msgs::Range>(topic, 10);

    output_.min_range = 0.05;
    output_.max_range = 3.0;
    output_.header.frame_id = frame_id_;
    output_.radiation_type = sensor_msgs::Range::ULTRASOUND;
    output_.field_of_view = 0.6;

    ray_deltas_.clear();

    double half_fov = output_.field_of_view / 2;

    // TODO: nicer solution
    for(double y = -half_fov; y < half_fov; y += 0.025) {
        for(double x = -half_fov; x < half_fov; x += 0.025) {
            ray_deltas_.push_back(tf::Vector3(1, x, y));
        }
    }

}

Sonar::~Sonar() {

}

void Sonar::publishScan() {
    output_.header.stamp = ros::Time::now();

    tf::Transform tf_map_to_sonar = getAbsolutePose();

    tf::Vector3 sonar_origin = tf_map_to_sonar.getOrigin();

    output_.range = 6.0;
    for(unsigned int i = 0; i < ray_deltas_.size(); ++i) {
        tf::Vector3 dir = tf::Transform(tf_map_to_sonar.getRotation()) * ray_deltas_[i];
        Ray r(sonar_origin, dir);

        double distance;
        if (getWorldHandle()->intersect(r, 0, 9, distance)) {
            if (distance < output_.range) {
                output_.range = distance;
            }
        }
    }

    pub_.publish(output_);
}
