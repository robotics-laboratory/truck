#pragma once

#include "lidar_map/common.h"

namespace truck::lidar_map {

struct ICPBuilderParams {};

class ICPBuilder {
  public:
    ICPBuilder(const ICPBuilderParams& params);
    ICP build();

  private:
    ICPBuilderParams params_;
};

ICP buildBaseICP();

}  // namespace truck::lidar_map
