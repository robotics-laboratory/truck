#pragma once

#include "model/model.h"
#include "motion/trajectory.h"

namespace truck::speed {

class GreedyPlanner {
  public:
    struct Parameters {
        double distance_to_obstacle = 0.7;
    };

    GreedyPlanner(const Parameters& params, const model::Model& model);

    GreedyPlanner& setScheduledVelocity(double scheduled_velocity) {
        const Limits<double> limit{.0, model_.baseVelocityLimits().max};

        VERIFY_FMT(
            limit.isMet(scheduled_velocity),
            "Scheduled velocity %f not in [%f, %f], backward motion is not supported!",
            scheduled_velocity,
            limit.min,
            limit.max);

        scheduled_velocity_ = scheduled_velocity;

        return *this;
    }

    void fill(motion::Trajectory& trajectory) const;

  private:
    Parameters params_;
    model::Model model_;
    double scheduled_velocity_ = 0.0;
};

}  // namespace truck::speed