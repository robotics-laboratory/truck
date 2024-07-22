#include "lidar_map/icp.h"

namespace truck::lidar_map {

ICPBuilder::ICPBuilder(const ICPBuilderParams& params) : params_(params) { /** @todo */
}

ICP ICPBuilder::build() {
    ICP icp;
    icp.setDefault();

    PointMatcherSupport::Parametrizable::Parameters kdtree_params;
    kdtree_params["knn"] = "1";
    kdtree_params["epsilon"] = "3.16";

    PointMatcherSupport::Parametrizable::Parameters point_to_plane_params_params;
    point_to_plane_params_params["force2D"] = "1";

    PointMatcherSupport::Parametrizable::Parameters robust_outlier_filter_params;
    robust_outlier_filter_params["robustFct"] = "cauchy";
    robust_outlier_filter_params["tuning"] = "1.0";
    robust_outlier_filter_params["distanceType"] = "point2plane";

    std::shared_ptr<Matcher::Matcher> kdtree =
        Matcher::get().MatcherRegistrar.create("KDTreeMatcher", kdtree_params);

    std::shared_ptr<Matcher::ErrorMinimizer> point_to_plane =
        Matcher::get().ErrorMinimizerRegistrar.create(
            "PointToPlaneErrorMinimizer", point_to_plane_params_params);

    std::shared_ptr<Matcher::OutlierFilter> robust_outlier_filter =
        Matcher::get().OutlierFilterRegistrar.create(
            "RobustOutlierFilter", robust_outlier_filter_params);

    icp.matcher = kdtree;
    icp.errorMinimizer = point_to_plane;
    icp.outlierFilters.push_back(robust_outlier_filter);

    return icp;
}

}  // namespace truck::lidar_map
