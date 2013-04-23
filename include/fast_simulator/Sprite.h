#ifndef _FAST_SIMULATOR_SPRITE_H_
#define _FAST_SIMULATOR_SPRITE_H_

#include "Shape.h"

class Sprite : public Shape {

public:

    Sprite(const std::string& filename, double resolution, double z_min, double z_max);

    Sprite* clone() const;

    bool intersect(const Ray &, float t0, float t1, double& distance) const;

protected:

    unsigned int width_, height_;

    double resolution_;

    double z_min_, z_max_;

    std::vector<bool> sprite_;

};


#endif
