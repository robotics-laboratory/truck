#pragma once

#include "icp_odometry/common.h"
#include "icp_odometry/import_bag.h"

#include <vector>

namespace truck::icp_odometry {
    void optimizePoses(Matcher::ICP &, std::vector <ICPOdometryData> &, double);

    DataPoints getMergedDataPoints(std::vector <ICPOdometryData> &);

    std::vector <DataPoints> getTransformedDataPoints(std::vector <ICPOdometryData> &);
}
