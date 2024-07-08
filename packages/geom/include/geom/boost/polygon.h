#pragma once

#include "geom/complex_polygon.h"
#include "geom/boost/point.h"
#include "geom/boost/ring.h"

#include <boost/geometry.hpp>
#include <vector>

namespace boost {
namespace geometry {
namespace traits {
template<>
struct tag<ComplexPolygon> {
    typedef polygon_tag type;
};

template<>
struct ring_const_type<ComplexPolygon> {
    typedef const Polygon& type;
};
template<>
struct ring_mutable_type<ComplexPolygon> {
    typedef Polygon& type;
};
template<>
struct interior_const_type<ComplexPolygon> {
    typedef const std::vector<Polygon>& type;
};
template<>
struct interior_mutable_type<ComplexPolygon> {
    typedef std::vector<Polygon>& type;
};

template<>
struct exterior_ring<ComplexPolygon> {
    static Polygon& get(ComplexPolygon& p) { return (p.outer); }
    static const Polygon& get(const ComplexPolygon& p) { return (p.outer); }
};

template<>
struct interior_rings<ComplexPolygon> {
    static std::vector<Polygon> get(ComplexPolygon& p) {
        return std::vector<Polygon>(
            std::vector<Polygon>::iterator(p.inners.begin()),
            std::vector<Polygon>::iterator(p.inners.end()));
    }
    static const std::vector<Polygon> get(const ComplexPolygon& p) {
        return std::vector<Polygon>(
            std::vector<Polygon>::const_iterator(p.inners.begin()),
            std::vector<Polygon>::const_iterator(p.inners.end()));
    }
};
}  // namespace traits
}  // namespace geometry
}  // namespace boost
