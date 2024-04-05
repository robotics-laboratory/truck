## ICP Odometry

### Example of package use

```c++
#include "icp_odometry/dvc.h"
#include "icp_odometry/g2o.h"
#include "icp_odometry/icp.h"
#include "icp_odometry/icp_odometry_node.h"
#include "icp_odometry/import_bag.h"

#include <pointmatcher/PointMatcher.h>
#include <rclcpp/rclcpp.hpp>


using namespace truck::icp_odometry;

int main() {
	std::vector <truck::icp_odometry::DataPoints> data_points = truck::icp_odometry::read_all_data_points(
			"/truck/packages/icp_odometry/bags/atrium_final_3_0.db3"
	);
	PointMatcher<float>::ICP icp = GetICPWithRobustOutlierFilter(DistType::Point2Point, RobustFct::Huber);
	std::cout << "data_points size: " << data_points.size() << std::endl;
	std::vector<DataPoints> sampledDataPoints = createSamples(data_points, 100, 50, 10);
	std::cout << "sampledDataPoints size: " << sampledDataPoints.size() << std::endl;
	std::vector<TransformationParameters> transformations = MapToTransformations(icp, sampledDataPoints);
	std::cout << "transformations size: " << transformations.size() << std::endl;
	std::string data = serializeTransformations(transformations);
	std::cout << "serialized transformations: " << data << std::endl;
	std::string path = "./icp_odometry/transformation_dumps/example_transformations_dump.txt";
	writeToFile(data, path);
	optimizePoses(transformations);
}
```