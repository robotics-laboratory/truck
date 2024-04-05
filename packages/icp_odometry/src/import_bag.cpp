#include "icp_odometry/import_bag.h"

#include "icp_odometry/common.h"
#include "icp_odometry/conversion.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <optional>
#include <vector>


namespace truck::icp_odometry {

    std::optional <DataPoints> readNextDataPoints(rosbag2_cpp::readers::SequentialReader &reader) {
        while (reader.has_next()) {
            rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();

            if (msg->topic_name != "/lidar/scan") {
                continue;
            }

            rclcpp::SerializedMessage serializedMsg(*msg->serialized_data);
            sensor_msgs::msg::LaserScan::SharedPtr rosMsg = std::make_shared<sensor_msgs::msg::LaserScan>();

            rclcpp::Serialization <sensor_msgs::msg::LaserScan> serialization;
            serialization.deserialize_message(&serializedMsg, rosMsg.get());

            DataPoints dataPoints = toDataPoints(*rosMsg);
            return dataPoints;
        }
        return std::nullopt;
    }

    std::vector <DataPoints> readAllDataPoints(std::string pathToBag) {
        rosbag2_storage::StorageOptions storageOptions;
		storageOptions.uri = pathToBag;
		storageOptions.storage_id = "sqlite3";

        rosbag2_cpp::ConverterOptions converterOptions;
		converterOptions.input_serialization_format = "cdr";
		converterOptions.output_serialization_format = "cdr";

        rclcpp::Serialization <sensor_msgs::msg::LaserScan> serialization;

        rosbag2_cpp::readers::SequentialReader reader;
        reader.open(storageOptions, converterOptions);

        std::vector <DataPoints> result;

        while (true) {
            std::optional <DataPoints> dataPoints = readNextDataPoints(reader);
            if (!dataPoints) {
                break;
            } else {
                result.push_back(*dataPoints);
            }
        }

        return result;
    }
}
