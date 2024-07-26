#include "lidar_map/icp.h"

namespace truck::lidar_map {

ICPBuilder::ICPBuilder(const ICPBuilderParams& params) : params_(params) { /** @todo */
}

ICP ICPBuilder::build() { /** @todo */
}

ICP buildBaseICP() {
    ICP icp;
    icp.setDefault();

    PointMatcherSupport::Parametrizable::Parameters matcher_params;
    matcher_params["knn"] = "1";
    matcher_params["epsilon"] = "3.16";

    std::shared_ptr<Matcher::Matcher> matcher =
        Matcher::get().MatcherRegistrar.create("KDTreeMatcher", matcher_params);

    PointMatcherSupport::Parametrizable::Parameters error_minimizer_params;
    error_minimizer_params["force2D"] = "1";

    std::shared_ptr<Matcher::ErrorMinimizer> error_minimizer =
        Matcher::get().ErrorMinimizerRegistrar.create(
            "PointToPlaneErrorMinimizer", error_minimizer_params);

    PointMatcherSupport::Parametrizable::Parameters outlier_filter_params;
    outlier_filter_params["robustFct"] = "cauchy";
    outlier_filter_params["tuning"] = "1.0";
    outlier_filter_params["distanceType"] = "point2plane";

    std::shared_ptr<Matcher::OutlierFilter> outlier_filter =
        Matcher::get().OutlierFilterRegistrar.create("RobustOutlierFilter", outlier_filter_params);

    icp.matcher = matcher;
    icp.errorMinimizer = error_minimizer;
    icp.outlierFilters.push_back(outlier_filter);

    return icp;
}

}  // namespace truck::lidar_map
