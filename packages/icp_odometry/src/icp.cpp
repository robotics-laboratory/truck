#include "icp_odometry/icp.h"

#include "icp_odometry/common.h"
#include "icp_odometry/config.h"
#include "icp_odometry/g2o.h"

#include <pointmatcher/PointMatcher.h>


namespace truck::icp_odometry {
	std::map<RobustFct, std::string> RobustFctMap = {
			{RobustFct::Cauchy,              "cauchy"},
			{RobustFct::Welsch,              "welsch"},
			{RobustFct::SwitchableConstaint, "sc"},
			{RobustFct::GemanMcClure,        "gm"},
			{RobustFct::Tukey,               "tukey"},
			{RobustFct::Huber,               "huber"},
			{RobustFct::L1,                  "L1"},
			{RobustFct::Student,             "student"}
	};

	std::map<CustomOutlierFilter, std::string> CustomOutlierFilterMap = {
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
		std::shared_ptr<Matcher::Matcher> kdtree = Matcher::get().MatcherRegistrar.create(name, params);
		params.clear();
		icp.matcher = kdtree;

		if (distType == DistType::Point2Point) {
			name = "PointToPointErrorMinimizer";
			std::shared_ptr<Matcher::ErrorMinimizer> pointToPoint = Matcher::get().ErrorMinimizerRegistrar.create(name);
			icp.errorMinimizer = pointToPoint;
		} else {
			name = "PointToPlaneErrorMinimizer";
			params["force2D"] = "1";
			std::shared_ptr<Matcher::ErrorMinimizer> pointToPlane = Matcher::get().ErrorMinimizerRegistrar.create(name, params);
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
		std::shared_ptr<Matcher::OutlierFilter> robustOutlierFilter = Matcher::get().OutlierFilterRegistrar.create(name, params);
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
		std::shared_ptr<Matcher::OutlierFilter> outlierFilter = Matcher::get().OutlierFilterRegistrar.create(name, params);
		icp.outlierFilters.clear();
		icp.outlierFilters.push_back(outlierFilter);

		return icp;
	}

	DataPoints createSinglePointCloud(const DataPoints& inputCloud, int pointIndex) {
		if (pointIndex < 0 || pointIndex >= inputCloud.getNbPoints()) {
			throw std::out_of_range("pointIndex is out of range.");
		}

		DataPoints::Labels featureLabels = inputCloud.featureLabels;
		DataPoints::Labels descriptorLabels = inputCloud.descriptorLabels;

		DataPoints newCloud(featureLabels, descriptorLabels, 1);

		newCloud.features.col(0) = inputCloud.features.col(pointIndex);

		if (newCloud.descriptors.rows() > 0) {
			newCloud.descriptors.col(0) = inputCloud.descriptors.col(pointIndex);
		}

		return newCloud;
	}

	DataPoints filterPointsWithNeighbors(const std::vector<DataPoints>& clouds, int k, float distance_threshold) {
		std::vector<std::shared_ptr<PointMatcher<float>::Matcher>> kdtrees;
		for (const auto& cloud : clouds) {
			std::shared_ptr<PointMatcher<float>::Matcher> kdtree = PointMatcher<float>::get().REG(PointMatcher<float>::Matcher).create("KDTreeMatcher", {
					{"knn", "1"},
					{"epsilon", "0"},
			});
			kdtree->init(cloud);
			kdtrees.push_back(kdtree);
		}

		DataPoints filtered_cloud(clouds[0].createSimilarEmpty(0));

		for (size_t cloudIdx = 0; cloudIdx < clouds.size(); ++cloudIdx) {
			const auto& queryCloud = clouds[cloudIdx];
			for (int i = 0; i < queryCloud.getNbPoints(); ++i) {
				DataPoints singleDataPoints = createSinglePointCloud(queryCloud, i);

				int count = 0;
				for (size_t otherCloudIdx = 0; otherCloudIdx < clouds.size(); ++otherCloudIdx) {
					if (cloudIdx == otherCloudIdx) continue;

					Matcher::Matches matches = kdtrees[otherCloudIdx]->findClosests(singleDataPoints);

					if (matches.dists.size() > 0 && matches.dists(0, 0) < distance_threshold) {
						++count;
					}
				}

				if (count >= k) {
					filtered_cloud.concatenate(singleDataPoints);
				}
			}
		}

		return filtered_cloud;
	}

	DataPoints decreaseDensityRegions(const DataPoints cloud) {
		std::string cellSize = std::to_string(globalConfig.voxelFilterSize);
		std::shared_ptr<PointMatcher<float>::DataPointsFilter> voxelGridFilter = PointMatcher<float>::get().DataPointsFilterRegistrar.create("VoxelGridDataPointsFilter", {
			{"vSizeX", cellSize},
			{"vSizeY", cellSize},
			{"vSizeZ", cellSize}
		});
		return voxelGridFilter->filter(cloud);
	}
}