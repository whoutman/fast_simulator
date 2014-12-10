#include "fast_simulator/Sonar.h"
#include "fast_simulator/World.h"

#include <geolib/Ray.h>

Sonar::Sonar(const std::string& topic, const std::string& frame_id) : frame_id_(frame_id) {
    ros::NodeHandle nh;
    pub_ = nh.advertise<sensor_msgs::Range>(topic, 10);

    output_.min_range = 0.02;
    output_.max_range = 2.0;
    output_.header.frame_id = frame_id_;
    output_.radiation_type = sensor_msgs::Range::ULTRASOUND;
    output_.field_of_view = 0.7; // 40 degrees

    ray_deltas_.clear();

    double half_fov = output_.field_of_view / 2;
    double half_fov_vetical = 0.056 / 2;

    // TODO: nicer solution
    for(double y = -half_fov_vetical; y < half_fov_vetical; y += 0.01) {
        for(double x = -half_fov; x < half_fov; x += 0.025) {
            ray_deltas_.push_back(geo::Vector3(1, x, y));
        }
    }

}

Sonar::~Sonar() {

}

void Sonar::step(World& world) {
    output_.header.stamp = ros::Time::now();

    geo::Transform tf_map_to_sonar = getAbsolutePose();

    geo::Vector3 sonar_origin = tf_map_to_sonar.getOrigin();

    output_.range = output_.max_range;
    for(unsigned int i = 0; i < ray_deltas_.size(); ++i) {
        geo::Vector3 dir = tf_map_to_sonar.getBasis() * ray_deltas_[i];
        geo::Ray r(sonar_origin, dir);

        double distance;
        if (world.intersect(r, 0, 9, distance)) {
            if (distance < output_.range) {
                output_.range = distance;
            }
        }
    }

    pub_.publish(output_);
}
