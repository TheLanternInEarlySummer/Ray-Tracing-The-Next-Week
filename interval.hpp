#ifndef INTERVAL_HPP
#define INTERVAL_HPP

#include "rtweekend.hpp"

class interval {
   public:
    double min, max;
    interval() : min(+infinity), max(-infinity) {}
    interval(double min, double max) : min(min), max(max) {}
    interval(const interval& a, const interval& b) {
        min = (a.min <= b.min) ? a.min : b.min;
        max = (a.max >= b.max) ? a.max : b.max;
    }

    double size() const {
        return max - min;
    }

    bool contains(double x) const {
        // in close interval
        return min <= x && x <= max;
    }

    bool surrounds(double x) const {
        // in open interval
        return min < x && x < max;
    }

    double clamp(double x) const {
        if (x < min) return min;
        if (x > max) return max;
        return x;
    }

    interval expand(double delta) const {
        double padding = delta / 2.0;
        return interval(min - padding, max + padding);
    }

    static const interval empty, universe;
};
const interval interval::empty = interval(+infinity, -infinity);
const interval interval::universe = interval(-infinity, +infinity);

interval operator+(const interval& ival, double displacement) {
    return interval(ival.min + displacement, ival.max + displacement);
}

interval operator+(double displacement, const interval& ival) {
    return ival + displacement;
}

#endif