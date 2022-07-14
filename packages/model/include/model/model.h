#pragma once

#include "common/math.h"
#include "model/params.h"
#include "yaml-cpp/yaml.h"

namespace truck::model {

struct Steering {
    geom::Angle left;
    geom::Angle right;
};

class Model {
  public:
    Model(const std::string& config_path);

    double baseMaxAbsCurvature() const;
    Limits<double> baseVelocityLimits() const;

    double rearToBaseCurvature(double C) const;
    double baseToRearCurvature(double C) const;

    Steering rearCurvatureToSteering(double C) const;

  private:
    struct Cache {
        double width_half;
        double max_abs_curvature;
    } cache_;

    Params params_;
};

}  // namespace truck::model