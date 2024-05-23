#include "icp_odometry/visualization.h"

#include "icp_odometry/icp_odometry_node.h"
#include "icp_odometry/conversion.h"
#include "icp_odometry/icp.h"

#include <pointmatcher/PointMatcher.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <sstream>
#include <thread>


inline std_msgs::msg::ColorRGBA createColorRGBA(uint8_t r, uint8_t g, uint8_t b, float a) {
    std_msgs::msg::ColorRGBA color;
    color.r = static_cast<float_t>(r) / 255;
    color.g = static_cast<float_t>(g) / 255;
    color.b = static_cast<float_t>(b) / 255;
    color.a = a;
    return color;
}

const std::string FRAME_ID = "common";

namespace truck::icp_odometry {

    std::vector <std_msgs::msg::ColorRGBA> COLORS = {
            createColorRGBA(0, 128, 0, 1), createColorRGBA(255, 0, 0, 1),
            createColorRGBA(0, 0, 255, 1), createColorRGBA(255, 255, 0, 1),
            createColorRGBA(128, 0, 128, 1), createColorRGBA(255, 165, 0, 1),
            createColorRGBA(0, 0, 139, 1), createColorRGBA(46, 139, 87, 1),
            createColorRGBA(0, 191, 255, 1), createColorRGBA(64, 224, 208, 1),
            createColorRGBA(220, 20, 60, 1), createColorRGBA(165, 42, 42, 1),
            createColorRGBA(245, 245, 220, 1), createColorRGBA(255, 182, 193, 1),
            createColorRGBA(128, 128, 128, 1), createColorRGBA(75, 0, 130, 1),
            createColorRGBA(46, 139, 87, 1), createColorRGBA(128, 128, 0, 1),
            createColorRGBA(211, 211, 211, 1), createColorRGBA(245, 255, 250, 1),
    };

    const std_msgs::msg::ColorRGBA GREY = createColorRGBA(64, 64, 64, 1);
    const std_msgs::msg::ColorRGBA WHITE = createColorRGBA(255, 255, 255, 1);

    struct ICPView {
        Matcher::ICP icp;
        std_msgs::msg::ColorRGBA color;
        std::string publisherName;
    };

    class ColorProvider {
        size_t colorCtr = 0;

    public:
        std_msgs::msg::ColorRGBA getColor() { return COLORS[this->colorCtr++]; }
    };

    float get_element(Matcher::OutlierWeights m, int i, int j) { return m(i, j); }

    bool checkValueRange(float_t value) { return std::abs(value) <= 100; }

    visualization_msgs::msg::Marker dataPointsToMarker(DataPoints dataPoints, std::string frame_id, std_msgs::msg::ColorRGBA color, builtin_interfaces::msg::Time stamp) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = stamp;

        marker.frame_locked = true;

        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;

        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.000001;

        for (size_t i = 0; i < dataPoints.getNbPoints(); ++i) {
            geometry_msgs::msg::Point point;
            point.x = dataPoints.features(0, i);
            point.y = dataPoints.features(1, i);
            point.z = dataPoints.features(2, i);

            if (!checkValueRange(point.x) || !checkValueRange(point.y) || !checkValueRange(point.z)) {
                continue;
            }
            marker.points.push_back(point);

            marker.colors.push_back(color);
        }

        return marker;
    }

    geometry_msgs::msg::Point eigenVectorToPointMsg(Eigen::Vector3f vector) {
        geometry_msgs::msg::Point point_msg;
        point_msg.x = vector.x();
        point_msg.y = vector.y();
        point_msg.z = vector.z();
        return point_msg;
    }

    visualization_msgs::msg::Marker matchesToSegmentMarker(Matcher::Matches matches, std::string frame_id, DataPoints ref,
                                                           DataPoints data, Matcher::OutlierWeights outlierWeights, builtin_interfaces::msg::Time stamp) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = stamp;

        marker.frame_locked = true;

        marker.type = visualization_msgs::msg::Marker::LINE_LIST;

        marker.scale.x = 0.0025;

        for (long int i = 0; i < matches.ids.cols(); ++i) {
            const int refIndex = matches.ids(0, i);
            const int dataIndex = i;

            Eigen::Vector3f refPoint = ref.features.block(0, refIndex, 3, 1);
            Eigen::Vector3f dataPoint = data.features.block(0, dataIndex, 3, 1);
            const float weight = outlierWeights(0, i);

            marker.points.push_back(eigenVectorToPointMsg(refPoint));
            marker.points.push_back(eigenVectorToPointMsg(dataPoint));

            auto greyScale = 255.0 * weight;
            auto greyScaleColor = createColorRGBA(greyScale, greyScale, greyScale, 1);

            marker.colors.push_back(greyScaleColor);
            marker.colors.push_back(greyScaleColor);
        }

        return marker;
    }

    IcpVisualizationNode::IcpVisualizationNode() : Node("playback_node") {
        this->declare_parameter("bag_path", "/some/example/path");
        std::string bagPath = this->get_parameter("bag_path").as_string();

        ColorProvider colorProvider = ColorProvider();

        auto REFERENCE_COLOR = colorProvider.getColor();
        auto DATA_BEFORE_COLOR = colorProvider.getColor();

        std::vector <ICPView> ICPViews = {
                {getICPWithRobustOutlierFilter(DistType::Point2Point, RobustFct::Welsch),              colorProvider.getColor(), "welsch_p2point"},
                {getICPWithRobustOutlierFilter(DistType::Point2Plane, RobustFct::Welsch),              colorProvider.getColor(), "welsch_p2plane"},

                {getICPWithRobustOutlierFilter(DistType::Point2Point, RobustFct::SwitchableConstaint), colorProvider.getColor(), "switchable_constaint_p2point"},
                {getICPWithRobustOutlierFilter(DistType::Point2Plane, RobustFct::SwitchableConstaint), colorProvider.getColor(), "switchable_constaint_p2plane"},

                {getICPWithRobustOutlierFilter(DistType::Point2Point, RobustFct::GemanMcClure),        colorProvider.getColor(), "geman_mc_clure_p2point"},
                {getICPWithRobustOutlierFilter(DistType::Point2Plane, RobustFct::GemanMcClure),        colorProvider.getColor(), "geman_mc_clure_p2plane"},

                {getICPWithRobustOutlierFilter(DistType::Point2Point, RobustFct::Tukey),               colorProvider.getColor(), "tukey_p2point"},
                {getICPWithRobustOutlierFilter(DistType::Point2Plane, RobustFct::Tukey),               colorProvider.getColor(), "tukey_p2plane"},

                {getICPWithRobustOutlierFilter(DistType::Point2Point, RobustFct::Huber),               colorProvider.getColor(), "huber_p2point"},
                {getICPWithRobustOutlierFilter(DistType::Point2Plane, RobustFct::Huber),               colorProvider.getColor(), "huber_p2plane"},

                {getICPWithRobustOutlierFilter(DistType::Point2Point, RobustFct::L1),                  colorProvider.getColor(), "l1_p2point"},
                {getICPWithRobustOutlierFilter(DistType::Point2Plane, RobustFct::L1),                  colorProvider.getColor(), "l1_p2plane"},

                {getICPWithRobustOutlierFilter(DistType::Point2Point, RobustFct::Student),             colorProvider.getColor(), "student_p2point"},
                {getICPWithRobustOutlierFilter(DistType::Point2Plane, RobustFct::Student),             colorProvider.getColor(), "student_p2plane"},
        };

        publisher_icp_transformation_reference = this->create_publisher<visualization_msgs::msg::Marker>("/icp_transformation/reference", 10);
        publisher_icp_transformation_data_before = this->create_publisher<visualization_msgs::msg::Marker>("/icp_transformation/data_before", 10);

        for (auto icpView: ICPViews) {
            publishers_icp_transformation_data_before.push_back(
                    this->create_publisher<visualization_msgs::msg::Marker>("/icp_transformation/data_after/outlier_filter__" + icpView.publisherName, 10));
            publishers_icp_transformation_data_before_errors.push_back(
                    this->create_publisher<std_msgs::msg::Float32>("/icp_transformation/data_after/outlier_filter__" + icpView.publisherName + "_error", 10));
            publishers_icp_transformation_data_before_segments.push_back(
                    this->create_publisher<visualization_msgs::msg::Marker>("/icp_transformation/data_after/outlier_filter__" + icpView.publisherName + "_segments", 10));
        }

        std::optional <DataPoints> ref, data;

        auto icpOdometryData = readAllICPOdometryData("/truck/packages/icp_odometry/bags/atrium_final_3_0.db3", 100);

        for (int i = 1; i < icpOdometryData.size(); i++) {
            std::cout << "i: " << i << "\n" << std::flush;
            builtin_interfaces::msg::Time stamp = this->now();

            ref = icpOdometryData[i].icpDataPoints;
            data = icpOdometryData[i - 1].icpDataPoints;

            auto ref_marker = dataPointsToMarker(ref.value_or(DataPoints()), FRAME_ID, REFERENCE_COLOR, stamp);
            publisher_icp_transformation_reference->publish(ref_marker);

            auto data_marker = dataPointsToMarker(data.value_or(DataPoints()), FRAME_ID, DATA_BEFORE_COLOR, stamp);
            publisher_icp_transformation_data_before->publish(data_marker);

            for (size_t i = 0; i < ICPViews.size(); i++) {
                auto icpView = ICPViews[i];
                auto points_publisher = this->publishers_icp_transformation_data_before[i];
                auto segments_publisher = this->publishers_icp_transformation_data_before_segments[i];

                Matcher::TransformationParameters T = icpView.icp(*data, *ref);
                DataPoints data_out(*data);
                icpView.icp.transformations.apply(data_out, T);

                visualization_msgs::msg::Marker marker = dataPointsToMarker(data_out, FRAME_ID, icpView.color, stamp);
                points_publisher->publish(marker);

                icpView.icp.referenceDataPointsFilters.apply(*ref); // for fix normals to calc error a bit later
                icpView.icp.matcher->init(*ref);
                Matcher::Matches matches = icpView.icp.matcher->findClosests(data_out);
                Matcher::OutlierWeights outlierWeights = icpView.icp.outlierFilters.compute(data_out, *ref, matches);

                segments_publisher->publish(matchesToSegmentMarker(matches, FRAME_ID, *ref, *data, outlierWeights, stamp));
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    void saveDataPointsToMcap(const DataPoints &dataPoints, const std::string &filename, int argc, char **argv) {
        auto p = rclcpp::Clock().now();
        visualization_msgs::msg::Marker marker = dataPointsToMarker(dataPoints, "check", COLORS[0], p);
        std::unique_ptr <rosbag2_cpp::Writer> writer_ = std::make_unique<rosbag2_cpp::Writer>();

        writer_->open(filename);

        writer_->create_topic(
                {"synthetic",
                 "visualization_msgs/msg/Marker",
                 rmw_get_serialization_format(),
                 ""});

        rclcpp::Clock clock;
        rclcpp::Time time_stamp = clock.now();
        for (int32_t ii = 0; ii < 100; ++ii) {
            writer_->write(marker, "synthetic", time_stamp);
            time_stamp += rclcpp::Duration::from_seconds(1);
        }
    }

    void saveICPTransformationToMcap(const DataPoints &data, const DataPoints &ref, const DataPoints &dataOut, const std::string &filename) {
        auto timeNow = rclcpp::Clock().now();
        visualization_msgs::msg::Marker dataMarker = dataPointsToMarker(data, "frame", COLORS[1], timeNow);
        visualization_msgs::msg::Marker refMarker = dataPointsToMarker(ref, "frame", COLORS[0], timeNow);
        visualization_msgs::msg::Marker dataOutMarker = dataPointsToMarker(dataOut, "frame", COLORS[2], timeNow);
        std::unique_ptr <rosbag2_cpp::Writer> writer_ = std::make_unique<rosbag2_cpp::Writer>();

        writer_->open(filename);

        writer_->create_topic({"data", "visualization_msgs/msg/Marker", rmw_get_serialization_format(), ""});
        writer_->create_topic({"ref", "visualization_msgs/msg/Marker", rmw_get_serialization_format(), ""});
        writer_->create_topic({"dataOut", "visualization_msgs/msg/Marker", rmw_get_serialization_format(), ""});
        writer_->write(dataMarker, "data", timeNow);
        writer_->write(refMarker, "ref", timeNow);
        writer_->write(dataOutMarker, "dataOut", timeNow);
    }
}  // namespace truck::icp_odometry