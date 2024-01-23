#pragma once

#include "geom/complex_polygon.h"
#include "geom/line.h"

namespace truck::map {

class Map {
  public:
    Map(geom::ComplexPolygons polygons);

    static Map fromGeoJson(const std::string& path);

    const geom::ComplexPolygons& polygons() const;

  private:
    geom::ComplexPolygons polygons_;
};

}  // namespace truck::map