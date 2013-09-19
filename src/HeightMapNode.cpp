#include "fast_simulator/HeightMapNode.h"

HeightMapNode::HeightMapNode(const Box& box) : box_(box), occupied_(false) {
    for(unsigned int i = 0; i < 4; ++i) {
        children_[i] = 0;
    }
}

HeightMapNode::HeightMapNode(const HeightMapNode& orig) : box_(orig.box_), occupied_(orig.occupied_) {
    for(unsigned int i = 0; i < 4; ++i) {
        if (orig.children_[i]) {
            children_[i] = orig.children_[i]->clone();
        } else {
            children_[i] = 0;
        }
    }
}

HeightMapNode::~HeightMapNode() {

}

HeightMapNode* HeightMapNode::clone() const {
    return new HeightMapNode(*this);
}

bool HeightMapNode::intersect(const Ray& r, float t0, float t1, double& distance) const {
    if (!box_.intersect(r, t0, t1, distance)) {
        return false;
    }

    if (occupied_) {
        return true;
    }

    if (occupied_) {
        return box_.intersect(r, t0, t1, distance);
    }
}

/*
void HeightMapNode::add(const tf::Vector3& p) {

}

void HeightMapNode::getCubes(std::vector<Box>& cubes, const tf::Vector3& offset) const {

}



bool HeightMapNode::intersect(const tf::Vector3& p) const {

}

bool HeightMapNode::intersect(const Box& b) const {

}
*/
