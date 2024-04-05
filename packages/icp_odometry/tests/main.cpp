#include <gtest/gtest.h>

#include "icp_odometry/common.h"
#include "icp_odometry/dvc.h"

#include <pointmatcher/PointMatcher.h>

#include <cstdlib>


using namespace truck::icp_odometry;

class DataPointsTest : public ::testing::Test {
protected:
	std::vector <DataPoints> dataPointsVector;
	std::vector <TransformationParameters> transformations;

	DataPoints generateDataPoints() {
		const int NUM_POINTS = 100;

		DataPoints::Labels featureLabels;
		featureLabels.push_back(DataPoints::Label("x", 1));
		featureLabels.push_back(DataPoints::Label("y", 1));
		featureLabels.push_back(DataPoints::Label("z", 1));

		DataPoints::Labels descriptorLabels;

		DataPoints result(featureLabels, descriptorLabels, NUM_POINTS);

		for (size_t i = 0; i < NUM_POINTS; ++i) {
			result.features(0, i) = static_cast<float>(std::rand());
			result.features(1, i) = static_cast<float>(std::rand());
			result.features(2, i) = 1.0f;
		}

		return result;
	}

	TransformationParameters generateTransformation() {
		TransformationParameters transformation(TRANSFORMATION_ROW_SIZE, TRANSFORMATION_COLS_SIZE);

		for (int i = 0; i < TRANSFORMATION_ROW_SIZE; i++) {
			for (int j = 0; j < TRANSFORMATION_COLS_SIZE; j++) {
				transformation(i, j) = rand();
			}
		}

		return transformation;
	}

	void SetUp() override {
		const int NUM_DATA_POINTS = 100;
		const int NUM_TRANSFORMATIONS = 5;

		for (size_t i = 0; i < NUM_DATA_POINTS; ++i) {
			DataPoints dataPoints = this->generateDataPoints();
			this->dataPointsVector.push_back(dataPoints);
		}

		for (size_t i = 0; i < NUM_TRANSFORMATIONS; ++i) {
			this->transformations.push_back(this->generateTransformation());
		}
	}
};

TEST_F(DataPointsTest, TestSerialization) {
	std::string serialization = serializeTransformations(transformations);
	std::vector <TransformationParameters> data = parseTransformations(serialization);
	std::string againSerialization = serializeTransformations(data);

	EXPECT_EQ(serialization, againSerialization);
	ASSERT_TRUE(false);
}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
