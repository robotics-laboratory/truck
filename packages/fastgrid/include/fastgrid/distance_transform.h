#pragma once

#include "fastgrid/grid.h"
#include "fastgrid/holder.h"

namespace truck::fastgrid {

void distanceTransformApprox3(const U8Grid& input, S32Grid& buf, F32Grid& out) noexcept;

void distanceTransformApprox3(const U8Grid& input, F32Grid& out) noexcept;

F32GridHolder distanceTransformApprox3(const U8Grid& input) noexcept;

void distanceTransformApprox5(const U8Grid& input, S32Grid& buf, F32Grid& out) noexcept;

void distanceTransformApprox5(const U8Grid& input, F32Grid& out) noexcept;

F32GridHolder distanceTransformApprox5(const U8Grid& input) noexcept;

}  // namespace truck::fastgrid
