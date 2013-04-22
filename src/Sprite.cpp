#include "fast_simulator/Sprite.h"

#include <opencv2/highgui/highgui.hpp>

Sprite::Sprite(const std::string& filename, double resolution, double z_min, double z_max) : width_(0), height_(0),
    resolution_(resolution), z_min_(z_min), z_max_(z_max) {

    cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);   // Read the file

    if (image.data ) {
        width_ = image.cols;
        height_ = image.rows;

        for(unsigned int y = 0; y < height_; ++y) {
            for(unsigned int x = 0; x < width_; ++x) {
                sprite_.push_back(image.at<unsigned char>(y, x) < 128);
                std::cout << sprite_.back();
            }
            std::cout << std::endl;
        }
    }
}

bool Sprite::intersect(const Ray& r, float t0, float t1, double& distance) const {
    tf::Vector3 delta = r.direction * resolution_;

    tf::Vector3 v = r.origin;
    distance = 0;
    for(; distance < t1; distance += resolution_) {
        if (z_min_ < v.z() && v.z() < z_max_) {
            int mx = v.x() / resolution_;
            int my = v.y() / resolution_;

            // if ... return true
        }
        v += delta;
    }

    return false;
}
