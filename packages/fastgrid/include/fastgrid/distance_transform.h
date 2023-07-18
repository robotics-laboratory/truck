#pragma once

#include "fastgrid/grid.h"
#include "fastgrid/holder.h"

namespace truck::fastgrid {

void DistanceTransformApprox3(const U8Grid& input, S32Grid& buf, F32Grid& out) noexcept;

void DistanceTransformApprox3(const U8Grid& input, F32Grid& out) noexcept;

F32GridHolder DistanceTransformApprox3(const U8Grid& input) noexcept;

void DistanceTransformApprox5(const U8Grid& input, S32Grid& buf, F32Grid& out) noexcept;

void DistanceTransformApprox5(const U8Grid& input, F32Grid& out) noexcept;

F32GridHolder DistanceTransformApprox5(const U8Grid& input) noexcept;

}  // namespace truck::fastgrid
