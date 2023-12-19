#include "map/map_builder.h"

#include "geom/segment.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace truck::nav_mesh {

struct DrawParams {
    std::string path;

    struct Size {
        int pixels;
        int scale;
    } size;

    struct Color {
        std::vector<int> background;
        std::vector<int> outer_poly;
        std::vector<int> inner_poly;
        std::vector<int> offset_poly;
        std::vector<int> straight_skeleton;
        std::vector<int> mesh;
    } color;

    struct Show {
        bool mesh;
        bool polys;
        bool offset_polys;
        bool straight_skeleton;
    } show;
};

struct NavMeshParams {
    double dist;
    double offset;
    bool grid_filter;
};

class NavMesh {
  public:
    NavMesh(const NavMeshParams& params);

    NavMesh& setPolygons(const geom::ComplexPolygons& polygons);
    NavMesh& build();

    const std::vector<geom::Vec2>& mesh() const;

    void draw(const DrawParams& draw_params);

  private:
    void buildStraightSkeleton();
    void buildOffsetPolygons();
    void buildMesh();

    void drawMesh(const DrawParams& draw_params) const;
    void drawPolygons(const DrawParams& draw_params) const;
    void drawOffsetPolygons(const DrawParams& draw_params) const;
    void drawStraightSkeleton(const DrawParams& draw_params) const;

    struct Obj {
        std::vector<geom::Vec2> mesh;
        geom::ComplexPolygons polygons;
        geom::Polygons offset_polygons;
        std::vector<geom::Segment> straight_skeleton;
    } obj_;

    NavMeshParams params_;

    cv::Mat frame_;
};

}  // namespace truck::nav_mesh