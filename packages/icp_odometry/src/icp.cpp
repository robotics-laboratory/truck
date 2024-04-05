#include "icp_odometry/icp.h"

#include "icp_odometry/common.h"

#include <pointmatcher/PointMatcher.h>


namespace truck::icp_odometry {
	std::map <RobustFct, std::string> RobustFctMap = {
			{RobustFct::Cauchy,              "cauchy"},
			{RobustFct::Welsch,              "welsch"},
			{RobustFct::SwitchableConstaint, "sc"},
			{RobustFct::GemanMcClure,        "gm"},
			{RobustFct::Tukey,               "tukey"},
			{RobustFct::Huber,               "huber"},
			{RobustFct::L1,                  "L1"},
			{RobustFct::Student,             "student"}
	};

	std::map <CustomOutlierFilter, std::string> CustomOutlierFilterMap = {
			{CustomOutlierFilter::TrimmedDistOutlierFilter,    "TrimmedDistOutlierFilter"},
			{CustomOutlierFilter::VarTrimmedDistOutlierFilter, "VarTrimmedDistOutlierFilter"},
			{CustomOutlierFilter::MedianDistOutlierFilter,     "MedianDistOutlierFilter"},
			{CustomOutlierFilter::SurfaceNormalOutlierFilter,  "SurfaceNormalOutlierFilter"},
	};

	Matcher::ICP getDefaultICP(const DistType distType) {
		Matcher::ICP icp;
		PointMatcherSupport::Parametrizable::Parameters params;
		std::string name;

		icp.setDefault();

		name = "KDTreeMatcher";
		params["knn"] = "1";
		params["epsilon"] = "3.16";
		std::shared_ptr <Matcher::Matcher> kdtree = Matcher::get().MatcherRegistrar.create(name, params);
		params.clear();
		icp.matcher = kdtree;

		if (distType == DistType::Point2Point) {
			name = "PointToPointErrorMinimizer";
			std::shared_ptr <Matcher::ErrorMinimizer> pointToPoint = Matcher::get().ErrorMinimizerRegistrar.create(name);
			icp.errorMinimizer = pointToPoint;
		} else {
			name = "PointToPlaneErrorMinimizer";
			params["force2D"] = "1";
			std::shared_ptr <Matcher::ErrorMinimizer> pointToPlane = Matcher::get().ErrorMinimizerRegistrar.create(name, params);
			icp.errorMinimizer = pointToPlane;
			params.clear();
		}

		return icp;
	}

	Matcher::ICP getICPWithRobustOutlierFilter(const DistType distType, const RobustFct robustFct, const float tuning) {
		Matcher::ICP icp = getDefaultICP(distType);
		PointMatcherSupport::Parametrizable::Parameters params;
		std::string name;

		name = "RobustOutlierFilter";
		params["robustFct"] = RobustFctMap[robustFct];
		params["tuning"] = std::to_string(tuning);
		if (distType == DistType::Point2Point) {
			params["distanceType"] = "point2point";
		} else {
			params["distanceType"] = "point2plane";
		}
		std::shared_ptr <Matcher::OutlierFilter> robustOutlierFilter = Matcher::get().OutlierFilterRegistrar.create(name, params);
		params.clear();
		icp.outlierFilters.clear();
		icp.outlierFilters.push_back(robustOutlierFilter);

		return icp;
	}

	Matcher::ICP getICPWithCustomOutlierFilter(const DistType distType, const CustomOutlierFilter customOutlierFilter) {
		Matcher::ICP icp = getDefaultICP(distType);
		PointMatcherSupport::Parametrizable::Parameters params;
		std::string name;

		name = CustomOutlierFilterMap[customOutlierFilter];
		std::shared_ptr <Matcher::OutlierFilter> outlierFilter = Matcher::get().OutlierFilterRegistrar.create(name, params);
		icp.outlierFilters.clear();
		icp.outlierFilters.push_back(outlierFilter);

		return icp;
	}

	std::vector <TransformationParameters> mapToTransformations(Matcher::ICP icp, const std::vector <DataPoints> &data_points) {
		std::vector <TransformationParameters> result;

		for (int i = 1; i < data_points.size(); i++) {
			result.push_back(icp(data_points[i - 1], data_points[i]));
		}

		return result;
	}
}