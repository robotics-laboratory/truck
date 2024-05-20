#include "icp_odometry/dvc.h"

#include <filesystem>
#include <fstream>
#include <string>
#include <sstream>


namespace truck::icp_odometry {
	std::vector <DataPoints> createSamples(std::vector <DataPoints> dataPointsVector, size_t offset, size_t step, size_t limit) {
		std::vector <DataPoints> result;

		for (int i = offset, ctr = 0; ctr < limit && i < dataPointsVector.size(); i += step, ++ctr) {
			result.push_back(dataPointsVector[i]);
		}

		return result;
	}

	std::string serializeTransformations(std::vector <Eigen::Matrix3d> transformations) {
		std::stringstream ss;

		for (auto &transformation: transformations) {
			for (int i = 0; i < TRANSFORMATION_ROW_SIZE; i++) {
				for (int j = 0; j < TRANSFORMATION_COLS_SIZE; j++) {
					ss << transformation(i, j);
					if (i != (TRANSFORMATION_ROW_SIZE - 1) || j != (TRANSFORMATION_COLS_SIZE - 1)) {
						ss << ' ';
					} else {
						ss << '\n';
					}
				}
			}
		}

		return ss.str();
	}

	std::vector <TransformationParameters> parseTransformations(std::string data) {
		std::vector <TransformationParameters> result;

		std::istringstream input_stream(data);
		std::string matrixStr;
		while (std::getline(input_stream, matrixStr, '\n')) {
			TransformationParameters transformation(TRANSFORMATION_ROW_SIZE, TRANSFORMATION_COLS_SIZE);
			std::stringstream ss;
			ss << matrixStr;

			for (int i = 0; i < TRANSFORMATION_ROW_SIZE; i++) {
				for (int j = 0; j < TRANSFORMATION_COLS_SIZE; j++) {
					ss >> transformation(i, j);
				}
			}

			result.push_back(transformation);
		}

		return result;
	}

	void writeToFile(std::string data, std::string path) {
		std::filesystem::path dir = std::filesystem::path(path).parent_path();
		if (!std::filesystem::exists(dir)) {
			std::filesystem::create_directories(dir);
		}

		std::ofstream file(path);
		file << data;
		file.close();
	}

	std::string readFromFile(std::string path) {
		std::string result;
		std::ifstream file(path);
		if (file.is_open()) {
			std::stringstream strStream;
			strStream << file.rdbuf();
			result = strStream.str();
		}
		return result;
	}
}