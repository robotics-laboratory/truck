#pragma once

#include "icp_odometry/common.h"


namespace truck::icp_odometry {
	const size_t TRANSFORMATION_ROW_SIZE = 3;
	const size_t TRANSFORMATION_COLS_SIZE = 3;

	std::vector <DataPoints> createSamples(std::vector <DataPoints>, size_t, size_t, size_t);

	std::string serializeTransformations(std::vector <TransformationParameters>);

	std::vector <TransformationParameters> parseTransformations(std::string);

	void writeToFile(std::string, std::string);

	std::string readFromFile(std::string);
}