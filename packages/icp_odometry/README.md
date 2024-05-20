## ICP Odometry

### Example of package use

```c++
#include "icp_odometry/config.h"
#include "icp_odometry/dvc.h"
#include "icp_odometry/g2o.h"
#include "icp_odometry/icp.h"
#include "icp_odometry/icp_odometry_node.h"
#include "icp_odometry/import_bag.h"
#include "icp_odometry/visualization.h"

#include <pointmatcher/PointMatcher.h>
#include <rclcpp/rclcpp.hpp>


using namespace truck::icp_odometry;

int main(int argc, char **argv) {
	std::cout << "BagPath: " << globalConfig.bagPath << '\n';
	std::vector<truck::icp_odometry::ICPOdometryData> icpOdometryData = truck::icp_odometry::readAllICPOdometryData(
			globalConfig.bagPath
	);
	PointMatcher<float>::ICP icp = getICPWithRobustOutlierFilter(DistType::Point2Plane, RobustFct::Cauchy);
	if (globalConfig.verbose) {
		std::cout << "data_points size: " << icpOdometryData.size() << std::endl;
	}
	std::vector<ICPOdometryData> selectedICPOdometryData = selectReferencePoints(icpOdometryData, globalConfig.referencePointsDistanceThreshold);
	if (globalConfig.verbose) {
		std::cout << "selectedICPOdometryData size: " << selectedICPOdometryData.size() << std::endl;
	}

	optimizePoses(icp, selectedICPOdometryData, globalConfig.icpEdgeMaxDistance);
	std::vector<DataPoints> transformedDataPoints = getTransformedDataPoints(selectedICPOdometryData);

	DataPoints filteredResult = filterPointsWithNeighbors(transformedDataPoints, globalConfig.neighborsFilterKnn, globalConfig.neighborsFilterDistanceThreshold);
	filteredResult = decreaseDensityRegions(filteredResult);

	saveDataPointsToMcap(filteredResult, globalConfig.resultPath, argc, argv);
}
```