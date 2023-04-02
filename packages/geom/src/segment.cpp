#include "geom/segment.h"

#include "geom/bezue.h"

namespace truck::geom {

Poses Segment::trace(double step) const noexcept { return bezue1(begin, end, step); }

}  // namespace truck::geom