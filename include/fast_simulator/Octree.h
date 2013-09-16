#ifndef _Octree_H_
#define _Octree_H_

#include "Shape.h"
#include "OctreeNode.h"
#include "Box.h"

class Octree : public Shape {

    friend class OctreeNode;

public:

    Octree(double size, double resolution = 0.1);

    Octree(const Octree& orig);

    virtual ~Octree();    

    Octree* clone() const;

    void getBoundingBox(tf::Vector3 &min, tf::Vector3 &max) const;

    void clear();

    void add(const tf::Vector3& p);

    void getCubes(std::vector<Box>& cubes) const;

    double setResolution(double resolution);

    double getResolution() const;

    bool intersect(const Ray& r, float t0, float t1, double& distance) const;

    void raytrace(const Ray& r, float t0, float t1);

    bool intersect(const tf::Vector3& p) const;

    bool intersect(const Box& b) const;

    static Octree fromHeightImage(const std::string& filename, double height, double resolution);

protected:

    double resolution_;

    tf::Vector3 offset_;

    tf::Vector3 max_;

    double size_;

    OctreeNode* root_;

};

#endif
