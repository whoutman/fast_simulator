#ifndef _FAST_SIMULATOR_OCTREE_H_
#define _FAST_SIMULATOR_OCTREE_H_

#include "Shape.h"
#include "Object.h"

class Octree : public Shape {

public:

    Octree();

    virtual ~Octree();

    virtual Octree* clone() const;

    virtual bool intersect(const Ray &, float t0, float t1, double& distance) const;

    virtual void getBoundingBox(tf::Vector3& min, tf::Vector3& max) const;

    static Octree fromHeightImage(const std::string& filename, double height, double resolution);

protected:

    Object root_;

    static void createQuadTree(const std::vector<std::vector<double> >& map, unsigned int mx_min, unsigned int my_min,
                               unsigned int mx_max, unsigned int my_max, double resolution, Object* parent);

};

#endif
