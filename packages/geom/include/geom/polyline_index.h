#pragma once

#include "geom/pose.h"
#include "geom/distance.h"
#include "geom/interpolation.h"
#include "geom/motion_state.h"
#include "common/exception.h"
#include <optional>

namespace truck::geom {

struct PolylineIdx {
    size_t seg_n = 0;
    double t = 0;  // [0, 1]
};

template<class P>
struct AdvanceResult {
    P point;
    PolylineIdx poly_idx;
    bool reached_end = false;
};

template<typename P>
class PolylineIndex {
  public:
    PolylineIndex(std::vector<P>&& range) : points_(std::move(range)) {
        VERIFY(points_.size() >= 2);

        distances_.reserve(points_.size());
        distances_.push_back(0);

        auto prev = points_.begin();
        for (auto curr = points_.begin() + 1; curr != points_.end(); prev = curr, ++curr) {
            distances_.push_back(distance(*prev, *curr));
        }
    }

    P At(const PolylineIdx idx) const {
        const auto& a = points_[idx.seg_n];
        const auto& b = points_[idx.seg_n + 1];
        return LinearInterpolator<P>{}(a, b, idx.t);
    }

    AdvanceResult<P> AdvanceFromBegin(double dist) const {
        return AdvanceFrom(PolylineIdx{}, dist);
    }

    AdvanceResult<P> AdvanceFrom(PolylineIdx from_id, double dist) const {
        double start = distances_[from_id.seg_n];
        double target = distances_[from_id.seg_n] + dist;

        size_t l = from_id.seg_n;
        size_t r = distances_.size();

        while (l < r) {
            size_t mid = l + (r - l) / 2;
            if (distances_[mid] <= target) {
                l = mid + 1;
            } else {
                r = mid;
            }
        }

        --l;

        auto rem_dist = dist - distances_[l];
        auto t = clamp(rem_dist / distance(points_[l], points_[l + 1]), 0., 1.);

        return AdvanceResult<P>{
            .point = Interpolator<P>{}(points_[l], points_[r], t),
            .poly_idx = l,
            .reached_end = rem_dist <= 0.};
    }

    auto begin() const { return points_.begin(); }
    auto end() const { return points_.end(); }

    const auto& Distances() const { return distances_; }

  private:
    std::vector<P> points_;
    std::vector<double> distances_;
};
}  // namespace truck::geom
