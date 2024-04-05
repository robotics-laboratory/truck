#pragma once

#include "icp_odometry/common.h"

#include <vector>

namespace truck::icp_odometry {
	void optimizePoses(std::vector < TransformationParameters > );
}

