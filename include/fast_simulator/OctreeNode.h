#ifndef _OctreeNode_H_
#define _OctreeNode_H_

#include <tf/transform_datatypes.h>
#include "Ray.h"
#include "Box.h"

class Octree;

class OctreeNode {

public:

    OctreeNode(double size, Octree* octree);

    OctreeNode(const OctreeNode& orig, Octree* tree);

    virtual ~OctreeNode();

    OctreeNode* clone(Octree* tree) const;

    void add(const tf::Vector3& p);

    void getCubes(std::vector<Box>& cubes, const tf::Vector3& offset) const;

    bool intersect(const Ray& r, float t0, float t1, double& distance, const tf::Vector3& offset) const;

    void raytrace(const tf::Vector3& o, const tf::Vector3& dir, float t0, float t1, const tf::Vector3& offset);

    bool intersect(const tf::Vector3& p) const;

    bool intersect(const Box& b) const;

protected:

    double size_;

    double split_;

    OctreeNode* children_[8];

    bool occupied_;

    Octree* tree_;

};

#endif
