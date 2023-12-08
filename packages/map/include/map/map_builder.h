#pragma once

#include <geom/complex_polygon.h>

namespace truck::map {

class Map {
  public:
    Map(const std::string& path);

    const geom::ComplexPolygon& getComplexPolygon() const;

  private:
    void parseGeoJSON();

    geom::ComplexPolygon map_;
    std::string path_;
};

}  // namespace truck::map