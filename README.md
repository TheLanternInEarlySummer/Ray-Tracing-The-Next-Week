@[toc]
# Ray Tracing: The Next Week总结

##  前言

本文为Ebook, "Ray Tracing: The Next Week" by Peter Shirley的总结。主要内容包括动态模糊，BVH，纹理贴图，柏林噪声，局部光照，物体移动。

最终场景图如下：
![](https://img-blog.csdnimg.cn/direct/f7485529d1a8457fa0a91fc94397f6ea.png#pic_center)
![](https://img-blog.csdnimg.cn/direct/f504900552184509b7c3d3456325d9ca.png#pic_center)



## 动态模糊

实现原理：对于光线，我们引入光线达到物体的时间 tm，$tm \in [0, 1]$

```cpp
double time() const { return tm; }
```
对于物体，我们引入两个中心点 center1，center2，分别表示在 tm = 0 时物体的中心点坐标和 tm = 1时

```cpp
sphere(const point3& center1, const point3& center2, double radius, shared_ptr<material> mat) : center1(center1), radius(fmax(0, radius)), mat(mat), is_moving(true) {
    vec3 rvec = vec3(radius, radius, radius);
    aabb box1(center1 - rvec, center1 + rvec);
    aabb box2(center2 - rvec, center2 + rvec);
    bbox = aabb(box1, box2);

    center_vec = center2 - center1;
}
```
当计算光线与物体交点时，取当前物体中心（线性插值）

```cpp
point3 sphere_center(double time) const {
    return center1 + time * center_vec;
}
```
效果图：
![](https://img-blog.csdnimg.cn/direct/76d790242aa344cdb26f6013568819fb.png#pic_center)
## BVH (Bounding Volume Hierarchies)

对于光线与物体求交，我们之前的方法是遍历所有物体，求交点，取离屏幕最近的展示到屏幕上。这种方法的时间效率太低了。
为此我们引入包围盒的概念，其核心思想为：对于包围盒内的物体，如果光线与其没有交点，那么光线与其里面的物体都没有交点。
那么，对于包围盒，如何划分？我可以使用BVH。
BVH的思想类似二分，将所有物体分为左包围盒和右包围盒，求光线与左右包围盒的交点。重复这个过程，直到求完所有交点。
![](https://img-blog.csdnimg.cn/direct/225ff257f44d445789aab543b0b69468.png#pic_center)


```cpp
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
```

```cpp
boolhit(const ray& r, interval ray_t, hit_record& rec) const override {
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
```
## 纹理贴图

### 空间纹理

空间纹理直接对3d空间中的每个点进行着色。本次，实现的是一个网格纹理，$(\lfloor x \rfloor + \lfloor y \rfloor + \lfloor z \rfloor) \ \& \ 1$

```cpp
color value(double u, double v, const point3& p) const override {
     int xInteger = int(std::floor(inv_scale * p.x()));
     int yInteger = int(std::floor(inv_scale * p.y()));
     int zInteger = int(std::floor(inv_scale * p.z()));
     bool isOdd = (xInteger + yInteger + zInteger) % 2;
     return isOdd ? odd->value(u, v, p) : even->value(u, v, p);
 }
```
![](https://img-blog.csdnimg.cn/direct/09a6ec0a35a84fe296354247844bb3a0.png#pic_center)

### 平面纹理

对于平面纹理，本次实现的是一个地球的贴图
考虑如何建立二维平面到三维物体的映射。
$\theta$为从 -y 轴开始，逆时针旋转的角度；
$\phi$为从-x轴开始，逆时针旋转的角度；
由 $y = -\cos(\theta); x = -\cos(\phi)\sin(\theta);z=\sin(\phi)\sin(\theta)$，得$\phi=atan2(-z,x)+\pi;\theta=acrcos(-y)$

```cpp
static void get_sphere_uv(const point3& p, double& u, double& v) {
    double theta = acos(-p.y());
    double phi = atan2(-p.z(), p.x()) + pi;
    u = phi / (2 * pi);
    v = theta / pi;
}
```

![](https://img-blog.csdnimg.cn/direct/bf7deac55aa04321b77e64a171310724.png#pic_center)
## 柏林噪声

通过随机化，来模拟固体纹理，如大理石等
实现原理：
对于纹理颜色，通过设置noise值来控制其灰度的大小

```cpp
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
```
为了使生成的纹理在一定程度上连续，模拟湍流效果，我们设置turb函数，其主要功能在于扩大noise的随机范围，使得周围噪声对当前噪声产生影响

```cpp
double turb(const point3& p, int depth) const {
    double accum = 0.0;
    point3 temp_p = p;
    double weight = 1.0;

    for (int i = 0; i < depth; ++i) {
        accum += weight * noise(temp_p);
        weight *= 0.5;
        temp_p *= 2;
    }

    return fabs(accum);
}
```
![](https://img-blog.csdnimg.cn/direct/67d064865b354c589bef4da864902b5b.png#pic_center)

## 局部光照

本次的光照是以材质的形式存在的。
其具体作用过程为：为某个物体（本例中为一个平行四边形）添加diffuse_light材质，设置emitted函数，作为其的发射光。
在get_color中，更新得到的颜色为 color_from_emission + color_from_scatter

```cpp
color ray_color(const ray& r, int depth, const hittable& world) {
    // puts("enter the ray_color");
    if (depth <= 0)
        return color(0, 0, 0);  // all abosorb
    hit_record rec;
    if (!world.hit(r, interval(0.001, infinity), rec))
        return background;

    ray scattered;
    color attenuation;
    color color_from_emission = rec.mat->emitted(rec.u, rec.v, rec.p);

    if (!rec.mat->scatter(r, rec, attenuation, scattered))
        return color_from_emission;
    color color_from_scatter = attenuation * ray_color(scattered, depth - 1, world);

    return color_from_emission + color_from_scatter;
}
```

## 物体移动

本次主要实现了位移和旋转。在实现过程中，我们并没有改变实际物体的位置，而是通过改变入射光线来获得交点，然后再改变交点。

### 位移

其实现细节主要分为：1. 将光线向后移动补偿值 2. 求移动后光线与当前物体的交点 3. 将交点往补偿值的方向移动（补偿值为我们指定的位移向量）

```cpp
class translate : public hittable {
   public:
    translate(shared_ptr<hittable> object, const vec3& offset) : object(object), offset(offset) {
        bbox = object->bounding_box() + offset;
    }

    const char* className() const override {
        return "translate : hittable";
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        // Move the ray backwards by the offset
        ray offset_r(r.origin() - offset, r.direction(), r.time());
        if (!object->hit(offset_r, ray_t, rec))  // ?
            return false;

        // move the intersection point forwards by the offset
        rec.p += offset;
        return true;
    }

    aabb bounding_box() const override { return bbox; }

   private:
    shared_ptr<hittable> object;
    vec3 offset;
    aabb bbox;
};
```

### 旋转

其实现细节主要分为3步：1. 将光线从世界坐标改到物体坐标（即反方向旋转入射光线） 2. 求旋转后光线与当前物体的交点 3. 往正方向旋转交点和法线

```cpp
bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
    // Change the ray from world space to object space
    point3 origin = r.origin();
    vec3 direction = r.direction();

    origin[0] = cos_theta * r.origin()[0] - sin_theta * r.origin()[2];
    origin[2] = sin_theta * r.origin()[0] + cos_theta * r.origin()[2];

    direction[0] = cos_theta * r.direction()[0] - sin_theta * r.direction()[2];
    direction[2] = sin_theta * r.direction()[0] + cos_theta * r.direction()[2];

    ray rotated_r(origin, direction, r.time());

    // Detemine whether an intersection exists in object space
    if (!object->hit(rotated_r, ray_t, rec)) return false;

    // Change the point and normal from object space to world space
    point3 p = rec.p;
    p[0] = cos_theta * rec.p[0] + sin_theta * rec.p[2];
    p[2] = -sin_theta * rec.p[0] + cos_theta * rec.p[2];

    vec3 normal = rec.normal;
    normal[0] = cos_theta * rec.normal[0] + sin_theta * rec.normal[2];
    normal[2] = -sin_theta * rec.normal[0] + cos_theta * rec.normal[2];

    rec.p = p;
    rec.normal = normal;

    return true;
}
```

## The Whole Code

## 参考资料

[Ray Tracing The Next Week](https://raytracing.github.io/books/RayTracingTheNextWeek.html)


