#pragma once

namespace geom {

struct Point2 {
    double x;
    double y;
};

struct Point3 {
    explicit Point3(const Point2& p, double z=0): x(p.x), y(p.y), z(z) {}

    double x;
    double y;
    double z;
};

} // namespace geom