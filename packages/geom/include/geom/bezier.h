#include "geom/pose.h"
#include "geom/vector.h"

namespace truck::geom {

struct CurvePose {
    Pose pose;
    double curvature = 0.0;
};

struct CurvePoses : public std::vector<CurvePose> {
    using std::vector<CurvePose>::vector;
    using std::vector<CurvePose>::operator=;

    Poses AsPoses() const noexcept;
};

CurvePoses bezier1(const Vec2& p0, const Vec2& p1, size_t n);
CurvePoses bezier1(const Vec2& p0, const Vec2& p1, double step);

CurvePoses bezier2(const Vec2& p0, const Vec2& p1, const Vec2& p2, size_t n);
CurvePoses bezier2(const Vec2& p0, const Vec2& p1, const Vec2& p2, double step);

CurvePoses bezier3(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, size_t n);
CurvePoses bezier3(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, double step);

}  // namespace truck::geom
