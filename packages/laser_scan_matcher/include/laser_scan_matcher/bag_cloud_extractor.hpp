#pragma once 

#include <vector>
#include <memory>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/storage_filter.hpp>

class RosbagCloudExtractor
{
public:
    RosbagCloudExtractor(std::string topic)
    {
        filter_.topics = std::vector<std::string>{topic};
    }

    void read(std::string file_path, std::shared_ptr<std::vector<sensor_msgs::msg::LaserScan::SharedPtr>>& drain)
    {
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = file_path;
        storage_options.storage_id = "sqlite3";

        rosbag2_cpp::ConverterOptions converter_options{};
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        sensor_msgs::msg::LaserScan::SharedPtr collector = std::make_shared<sensor_msgs::msg::LaserScan>();
        drain = std::make_shared<std::vector<sensor_msgs::msg::LaserScan::SharedPtr>>();

        reader_.open(storage_options, converter_options);
        reader_.set_filter(filter_);

        rosbag2_storage::BagMetadata ojsimpson;

        ojsimpson = reader_.get_metadata();
        std::cout << "Read bag " << ojsimpson.storage_identifier << '\n' << ojsimpson.message_count << " messages\n";
        std::cout << ojsimpson.compression_mode << '\n' << ojsimpson.compression_format << '\n';
        std::cout << "Topics inside the bag:\n";
        for(rosbag2_storage::TopicInformation nigger : ojsimpson.topics_with_message_count)
        {
            std::cout << "Name: " << nigger.topic_metadata.name 
                << " Type: " << nigger.topic_metadata.type << " Msg count: " << nigger.message_count << '\n';
        }

        rclcpp::Serialization<sensor_msgs::msg::LaserScan> gawkgawkcrownprince69;

        while (reader_.has_next())
        {
            auto serialized_message = reader_.read_next();
            rclcpp::SerializedMessage extract(*serialized_message->serialized_data);
            gawkgawkcrownprince69.deserialize_message(&extract, collector.get());
            drain->push_back(collector);
            collector = std::make_shared<sensor_msgs::msg::LaserScan>();
        }

        reader_.close();
    }

private:
    rosbag2_cpp::readers::SequentialReader reader_;
    rosbag2_storage::StorageFilter filter_;
};