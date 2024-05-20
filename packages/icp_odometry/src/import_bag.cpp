#include "icp_odometry/import_bag.h"

#include "icp_odometry/common.h"
#include "icp_odometry/conversion.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <optional>
#include <vector>


const std::string HARDWARE_ODOMETRY_TOPIC = "/hardware/odom";
const std::string EKF_ODOMETRY_TOPIC = "/ekf/odometry/filtered";
const std::string LASER_SCAN_TOPIC = "/lidar/scan";


namespace truck::icp_odometry {
	template<typename T>
	std::optional<T> readNextMessage(std::unique_ptr<rosbag2_cpp::readers::SequentialReader>& reader, std::string topicName) {
		while (reader->has_next()) {
			rosbag2_storage::SerializedBagMessageSharedPtr msg = reader->read_next();

			if (msg->topic_name != topicName) {
				continue;
			}

			rclcpp::SerializedMessage serializedMsg(*msg->serialized_data);
			typename T::SharedPtr rosMsg = std::make_shared<T>();

			rclcpp::Serialization <T> serialization;
			serialization.deserialize_message(&serializedMsg, rosMsg.get());

			return *rosMsg;
		}
		return std::nullopt;
	}

	std::optional<DataPoints> readNextDataPoints(std::unique_ptr<rosbag2_cpp::readers::SequentialReader>& reader) {
		std::optional<sensor_msgs::msg::LaserScan> laserScan = readNextMessage<sensor_msgs::msg::LaserScan>(reader, LASER_SCAN_TOPIC);
		if (!laserScan) {
			return std::nullopt;
		}
		return toDataPoints(*laserScan);
	}

	std::optional<ICPOdometryData> readNextICPOdometryData(std::unique_ptr<rosbag2_cpp::readers::SequentialReader>& reader) {
		std::optional<DataPoints> dataPoints = readNextDataPoints(reader);
		if (!dataPoints) {
			return std::nullopt;
		}

		std::optional<nav_msgs::msg::Odometry> odometry = readNextMessage<nav_msgs::msg::Odometry>(reader, EKF_ODOMETRY_TOPIC);
		if (!odometry) {
			return std::nullopt;
		}

		return ICPOdometryData{.icpDataPoints = *dataPoints, .odometry = *odometry, .optimizedOdometry = nav_msgs::msg::Odometry()};
	}

	std::unique_ptr<rosbag2_cpp::readers::SequentialReader> getSequentialReader(std::string pathToBag) {
		rosbag2_storage::StorageOptions storageOptions;
		storageOptions.uri = pathToBag;
		storageOptions.storage_id = "sqlite3";

		rosbag2_cpp::ConverterOptions converterOptions;
		converterOptions.input_serialization_format = "cdr";
		converterOptions.output_serialization_format = "cdr";

		auto reader = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
		reader->open(storageOptions, converterOptions);
		return reader;
	}

	std::vector<ICPOdometryData> readAllICPOdometryData(std::string pathToBag, size_t limit) {
		size_t ctr = 0;
		std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader = getSequentialReader(pathToBag);

		std::vector<ICPOdometryData> result;

		while (limit == -1 || ctr++ < limit) {
			std::optional<ICPOdometryData> icpOdometryData = readNextICPOdometryData(reader);
			if (!icpOdometryData) {
				break;
			} else {
				result.push_back(*icpOdometryData);
			}
		}

		return result;
	}

	std::vector<DataPoints> readAllDataPoints(std::string pathToBag) {
		std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader = getSequentialReader(pathToBag);

		std::vector<DataPoints> result;

		while (true) {
			std::optional<DataPoints> dataPoints = readNextDataPoints(reader);
			if (!dataPoints) {
				break;
			} else {
				result.push_back(*dataPoints);
			}
		}

		return result;
	}
}
