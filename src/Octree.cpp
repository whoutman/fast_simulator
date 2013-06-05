#include "fast_simulator/Octree.h"

// for loading images
#include <opencv2/highgui/highgui.hpp>

using namespace std;

Octree::Octree() {

}

Octree::~Octree() {

}


Octree* Octree::clone() const {
    return new Octree(*this);
}

bool Octree::intersect(const Ray& r, float t0, float t1, double& distance) const {
    return root_.intersect(r, t0, t1, distance);
}

void Octree::getBoundingBox(tf::Vector3& min, tf::Vector3& max) const {

}

Octree Octree::fromHeightImage(const std::string& filename, double height, double resolution) {
    Octree octree;

    cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);   // Read the file

    if (image.data ) {
        vector<vector<double> > map;
        map.resize(image.cols);

        for(int x = 0; x < image.cols; ++x) {
            map[x].resize(image.rows);
            for(int y = 0; y < image.rows; ++y) {
                map[x][y] = 1 - (double)image.at<unsigned char>(y, x) / 255 * height;
            }
        }

        std::cout << "Loaded height map " << filename << std::endl;

        createQuadTree(map, 0, 0, image.cols, image.rows, resolution, &octree.root_);
    } else {
        std::cout << "Could not load height map " << filename << std::endl;
    }

    return octree;
}

void Octree::createQuadTree(const vector<vector<double> >& map,
                            unsigned int mx_min, unsigned int my_min,
                            unsigned int mx_max, unsigned int my_max,
                            double resolution, Object* parent) {

    double max_height = 0;
    for(unsigned int mx = mx_min; mx < mx_max; ++mx) {
        for(unsigned int my = my_min; my < my_max; ++my) {
            max_height = std::max(max_height, map[mx][my]);
        }
    }

    if (max_height == 0) {
        return;
    }

    Object* obj = new Object();
    tf::Vector3 min_map((double)mx_min * resolution,
                        (double)my_min * resolution, 0);
    tf::Vector3 max_map((double)mx_max * resolution,
                        (double)my_max * resolution, max_height);
    obj->setBoundingBox(Box(min_map, max_map));
    // parent->addChild(obj, tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)));

    //cout << indent << mx_min << " - " << mx_max << ", " << my_min << " - " << my_max << endl;

    // cout << indent << toString(min_map) << ", " << toString(max_map) << endl;


    if (mx_max - mx_min <= 2 || my_max - my_min <= 2) {
        for(unsigned int mx = mx_min; mx < mx_max; ++mx) {
            for(unsigned int my = my_min; my < my_max; ++my) {
                if (map[mx][my] > 0 ) {
                    tf::Vector3 pos((double)mx * resolution, (double)my * resolution, 0);

                    Object* child = new Object();
                    child->setShape(Box(pos, tf::Vector3(pos.x() + resolution, pos.y() + resolution, map[mx][my])));
                    obj->addChild(child);
                }
            }
        }
    } else {

        unsigned int cx = (mx_max + mx_min) / 2;
        unsigned int cy = (my_max + my_min) / 2;

        createQuadTree(map, mx_min, my_min, cx, cy, resolution, obj);
        createQuadTree(map, cx , my_min, mx_max, cy, resolution, obj);
        createQuadTree(map, mx_min, cy , cx, my_max, resolution, obj);
        createQuadTree(map, cx , cy , mx_max, my_max, resolution, obj);
    }

    parent->addChild(obj);
}
