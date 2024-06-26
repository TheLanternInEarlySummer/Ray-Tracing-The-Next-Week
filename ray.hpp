#ifndef RAY_HPP
#define RAY_HPP

#include "vec3.hpp"

class ray {
   public:
    ray() {}
    ray(const point3& origin, const vec3& direction) : orig(origin), dir(direction) {}
    ray(const point3& origin, const vec3& direction, double time) : orig(origin), dir(direction), tm(time) {}

    const point3& origin() const { return orig; }
    const vec3& direction() const { return dir; }
    double time() const { return tm; }

    point3 at(double t) const {
        return orig + t * dir;
    }

   private:
    point3 orig;
    vec3 dir;
    double tm;
};

#endif