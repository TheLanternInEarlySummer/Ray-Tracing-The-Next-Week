#ifndef BVH_H
#define BVH_H

#include <algorithm>

#include "aabb.hpp"
#include "hittable.hpp"
#include "hittable_list.hpp"
#include "rtweekend.hpp"

class bvh_node : public hittable {
   public:
    bvh_node(hittable_list list) : bvh_node(list.objects, 0, list.objects.size()) {
    }

    bvh_node(std::vector<shared_ptr<hittable>>& objects, size_t start, size_t end) {
        bbox = aabb::empty;
        for (size_t object_index = start; object_index < end; ++object_index)
            bbox = aabb(bbox, objects[object_index]->bounding_box());
        int axis = bbox.longest_axis();
        size_t object_span = end - start;
        // in the bbox, if only have 1 or 2 object(s), we use the object itself rather than a bbox
        if (object_span == 1) {
            left = right = objects[start];
        } else if (object_span == 2) {
            left = objects[start];
            right = objects[start + 1];
        } else {
            auto comparator = (axis == 0) ? box_x_compare : ((axis == 1) ? box_y_compare : box_z_compare);
            std::sort(objects.begin() + start, objects.begin() + end, comparator);
            int mid = start + object_span / 2;
            left = make_shared<bvh_node>(objects, start, mid);
            right = make_shared<bvh_node>(objects, mid, end);
        }
    }

    const char* className() const override {
        return "bvh_node : hittable";
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        // puts("enter bvh hit");
        if (!bbox.hit(r, ray_t)) {
            // puts("exit bvh hit");
            return false;
        }
        // if size_t became 0, we check sphere hit, rather than bvh_node hit.
        bool hit_left = left->hit(r, ray_t, rec);
        bool hit_right = right->hit(r, interval(ray_t.min, hit_left ? rec.t : ray_t.max), rec);
        // puts("exit bvh hit");
        return hit_left || hit_right;
    }

    aabb bounding_box() const override { return bbox; }

   private:
    shared_ptr<hittable> left;
    shared_ptr<hittable> right;
    aabb bbox;

    static bool box_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b, int axis_index) {
        interval a_axis_interval = a->bounding_box().axis_interval(axis_index);
        interval b_axis_interval = b->bounding_box().axis_interval(axis_index);
        return a_axis_interval.min < b_axis_interval.min;
    }

    static bool box_x_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
        return box_compare(a, b, 0);
    }

    static bool box_y_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
        return box_compare(a, b, 1);
    }

    static bool box_z_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
        return box_compare(a, b, 2);
    }
};

#endif