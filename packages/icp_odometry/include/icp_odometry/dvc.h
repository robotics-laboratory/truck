#pragma once

#include "icp_odometry/common.h"
#include "icp_odometry/import_bag.h"


namespace truck::icp_odometry {
	const size_t TRANSFORMATION_ROW_SIZE = 3;
	const size_t TRANSFORMATION_COLS_SIZE = 3;

	std::vector <DataPoints> createSamples(std::vector <DataPoints>, size_t, size_t, size_t);

	std::string serializeTransformations(std::vector <Eigen::Matrix3d>);

	std::vector <TransformationParameters> parseTransformations(std::string);

	void writeToFile(std::string, std::string);

	std::string readFromFile(std::string);

	std::vector<ICPOdometryData> selectReferencePoints(std::vector<ICPOdometryData>, double);
}