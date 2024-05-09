#ifndef TEXTURE_HPP
#define TEXTURE_HPP

#include "perlin.hpp"
#include "rtw_std_image.hpp"
#include "rtweekend.hpp"

class texture {
   public:
    virtual ~texture() = default;
    virtual color value(double u, double v, const point3& p) const = 0;
};

class solid_color : public texture {
   public:
    solid_color(const color& albedo) : albedo(albedo) {}
    solid_color(double red, double green, double blue) : solid_color(color(red, green, blue)) {}

    color value(double u, double v, const point3& p) const override {
        return albedo;
    }

   private:
    color albedo;
};

class checker_texture : public texture {
   public:
    checker_texture(double scale, shared_ptr<texture> even, shared_ptr<texture> odd) : inv_scale(1.0 / scale), even(even), odd(odd) {}
    checker_texture(double scale, const color& c1, const color& c2) : inv_scale(1.0 / scale), even(make_shared<solid_color>(c1)), odd(make_shared<solid_color>(c2)) {}

    color value(double u, double v, const point3& p) const override {
        int xInteger = int(std::floor(inv_scale * p.x()));
        int yInteger = int(std::floor(inv_scale * p.y()));
        int zInteger = int(std::floor(inv_scale * p.z()));
        bool isOdd = (xInteger + yInteger + zInteger) % 2;
        return isOdd ? odd->value(u, v, p) : even->value(u, v, p);
    }

   private:
    double inv_scale;
    shared_ptr<texture> even;
    shared_ptr<texture> odd;
};

class image_texture : public texture {
   public:
    image_texture(const char* filename) : image(filename) {}

    color value(double u, double v, const point3& p) const override {
        if (image.height() <= 0) return color(0, 1, 1);
        u = interval(0, 1).clamp(u);
        v = 1.0 - interval(0, 1).clamp(v);  // flip v to image coordinates

        auto i = int(u * image.width());
        auto j = int(v * image.height());
        auto pixel = image.pixel_data(i, j);

        auto color_scale = 1.0 / 255.0;
        return color(color_scale * pixel[0], color_scale * pixel[1], color_scale * pixel[2]);
    }

   private:
    rtw_image image;
};

class noise_texture : public texture {
   public:
    noise_texture() {}
    noise_texture(double scale) : scale(scale) {}

    color value(double u, double v, const point3& p) const override {
        // return color(1, 1, 1) * 0.5 * (1.0 + noise.noise(scale * p));
        // return color(1, 1, 1) * noise.turb(p, 7);
        return color(1, 1, 1) * 0.5 * (1 + sin(scale * p.z() + 8 * noise.turb(p, 7)));
    }

   private:
    perlin noise;
    double scale;
};

#endif