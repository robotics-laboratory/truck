#pragma once

#include "geom/polygon.h"
#include "geom/boost/point.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/ring.hpp>

BOOST_GEOMETRY_REGISTER_RING(truck::geom::Polygon)
