#pragma once

#include "geom/vector.h"
#include "geom/vector3.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>

BOOST_GEOMETRY_REGISTER_POINT_2D(truck::geom::Vec2, double, cs::cartesian, x, y)
BOOST_GEOMETRY_REGISTER_POINT_3D(truck::geom::Vec3, double, cs::cartesian, x, y, z)
