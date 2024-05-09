#ifndef SPHERE_HPP
#define SPHERE_HPP

#include "hittable.hpp"
#include "rtweekend.hpp"

class sphere : public hittable {
   public:
    // Stationary Sphere
    sphere(const point3& center, double radius, shared_ptr<material> mat) : center1(center), radius(fmax(0, radius)), mat(mat), is_moving(false) {
        vec3 rvec = vec3(radius, radius, radius);
        bbox = aabb(center1 - rvec, center1 + rvec);
    }
    // Moving Sphere, moves linearly from center1 at time = 0 to center2 at time = 1
    sphere(const point3& center1, const point3& center2, double radius, shared_ptr<material> mat) : center1(center1), radius(fmax(0, radius)), mat(mat), is_moving(true) {
        vec3 rvec = vec3(radius, radius, radius);
        aabb box1(center1 - rvec, center1 + rvec);
        aabb box2(center2 - rvec, center2 + rvec);
        bbox = aabb(box1, box2);

        center_vec = center2 - center1;
    }

    const char* className() const override {
        return "sphere : hittable";
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        // puts("enter sphere hit");
        // std::clog << ray_tmin << " " << ray_tmax << "**\n";
        point3 center = is_moving ? sphere_center(r.time()) : center1;
        vec3 oc = center - r.origin();
        double a = r.direction().length_squared();
        auto h = dot(r.direction(), oc);
        auto c = oc.length_squared() - radius * radius;
        double discriminant = h * h - a * c;
        // puts("exit sphere hit");
        if (discriminant < 0.0)
            return false;

        double sqrtd = sqrt(discriminant);
        double root = (h - sqrtd) / a;
        if (!ray_t.surrounds(root)) {
            root = (h + sqrtd) / a;
            if (!ray_t.surrounds(root)) {
                return false;
            }
        }

        rec.t = root;
        rec.p = r.at(rec.t);
        vec3 outward_normal = (rec.p - center) / radius;
        rec.set_face_normal(r, outward_normal);
        get_sphere_uv(outward_normal, rec.u, rec.v);
        rec.mat = mat;
        return true;
    }

    aabb bounding_box() const override { return bbox; }

   private:
    point3 center1;
    double radius;
    shared_ptr<material> mat;
    bool is_moving;
    vec3 center_vec;  // velocity
    aabb bbox;

    point3 sphere_center(double time) const {
        return center1 + time * center_vec;
    }

    static void get_sphere_uv(const point3& p, double& u, double& v) {
        double theta = acos(-p.y());
        double phi = atan2(-p.z(), p.x()) + pi;
        u = phi / (2 * pi);
        v = theta / pi;
    }
};

#endif