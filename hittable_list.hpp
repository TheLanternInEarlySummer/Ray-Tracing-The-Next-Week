#ifndef HITTABLE_LIST_HPP
#define HITTABLE_LIST_HPP

#include <vector>

#include "hittable.hpp"
#include "rtweekend.hpp"

class hittable_list : public hittable {
   public:
    std::vector<shared_ptr<hittable>> objects;

    hittable_list() {}
    hittable_list(shared_ptr<hittable> object) { add(object); }

    void clear() { objects.clear(); }

    void add(shared_ptr<hittable> object) {
        objects.push_back(object);
        bbox = aabb(bbox, object->bounding_box());
    }

    const char* className() const override {
        return "hittable_list : hittable";
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        hit_record tmp_rec;
        bool hit_anything = false;
        auto closest_so_far = ray_t.max;

        // puts("traverse all objects");
        for (const auto& object : objects) {
            // std::cout << "\nobject is " << (*object).className() << "\n";
            if (object->hit(r, interval(ray_t.min, closest_so_far), tmp_rec)) {
                hit_anything = true;
                closest_so_far = tmp_rec.t;
                // std::clog << closest_so_far << "*\n";
                rec = tmp_rec;
            }
        }
        // puts("end traverse the objects");
        return hit_anything;
    }

    aabb bounding_box() const override { return bbox; }

   private:
    aabb bbox;
};

#endif