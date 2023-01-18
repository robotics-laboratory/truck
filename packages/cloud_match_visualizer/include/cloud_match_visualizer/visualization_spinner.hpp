#pragma once

#include <memory>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

#include "cloud_match_visualizer/cloud_matcher.hpp"
#include "cloud_match_visualizer/bag_cloud_extractor.hpp"

namespace truck {
    class VisualizationSpinner : public rclcpp::Node {
public:
        explicit VisualizationSpinner() 
            : Node("visualization_spinner")
            , odom_qos_type_("DEFAULT")
            , rate_in_seconds_(0.5)
            , source_topic_("source_cloud")
            , target_topic_("target_cloud")
            , aligned_topic_("aligned_cloud")
            , frame_id_("visualisation_view")
            , input_topic_("/lidar/scan")
            , filepath_("")
            , idx1_(0)
            , idx2_(30) {
            RCLCPP_INFO(this->get_logger(), "The visualization node is initialized.");
            
            idx1_ = this->declare_parameter<int>("index1", idx1_);
            idx2_ = this->declare_parameter<int>("index2", idx2_);
            input_topic_ = this->declare_parameter("input_topic", input_topic_);
            filepath_ = this->declare_parameter("filepath", filepath_);
            if(filepath_ == "")
            {
                throw std::runtime_error("The path to a .db3 rosbag file must be specified.");
            }

            RCLCPP_INFO(this->get_logger(), "Rate of the publication in seconds: %f", rate_in_seconds_);

            if (odom_qos_type_ == "DEFAULT") {
                odom_qos_profile_ = rmw_qos_profile_default;
            }
            else if (odom_qos_type_ == "SENSOR_DATA") {
                odom_qos_profile_ = rmw_qos_profile_default;
            }
            else if (odom_qos_type_ == "SYSTEM_DEFAULT") {
                odom_qos_profile_ = rmw_qos_profile_system_default;
            }
            else {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Error: Invalid value assigned to the QoS parameter. Setting QoS profile to default.");
                odom_qos_profile_ = rmw_qos_profile_default;
                odom_qos_type_ = "DEFAULT";
            }
            RCLCPP_INFO(this->get_logger(), "QoS settings profile: %s", odom_qos_type_.c_str());

            launchICP(filepath_);

            source_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                source_topic_, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(odom_qos_profile_)));
            target_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                target_topic_, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(odom_qos_profile_)));
            aligned_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                aligned_topic_, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(odom_qos_profile_)));

            msg_loop_trigger_ = this->create_wall_timer(
                std::chrono::duration<double>(rate_in_seconds_),
                std::bind(&VisualizationSpinner::publishClouds, this));

            tf_publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
            tf_loop_trigger_ = this->create_wall_timer(
                std::chrono::duration<double>(rate_in_seconds_),
                std::bind(&VisualizationSpinner::publishTransform, this));
        }

        void publishTransform()
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->now();
            t.header.frame_id = "world";
            t.child_frame_id = frame_id_;

            t.transform.translation.x = 0.0;
            t.transform.translation.y = 0.0;
            t.transform.translation.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            tf_publisher_->sendTransform(t);
        }

        void launchICP(std::string filepath)
        {
            RosbagCloudExtractor reader(input_topic_);
            std::shared_ptr<std::vector<sensor_msgs::msg::LaserScan>> drain;

            reader.read(filepath, drain);

            sensor_msgs::msg::LaserScan source = (*drain)[idx1_];
            sensor_msgs::msg::LaserScan target = (*drain)[idx2_];

            RCLCPP_INFO(this->get_logger(), "The timestamp of the source cloud: %d.%u", source.header.stamp.sec, source.header.stamp.nanosec);
            RCLCPP_INFO(this->get_logger(), "The timestamp of the target cloud: %d.%u", target.header.stamp.sec, source.header.stamp.nanosec);

            CloudMatcher processor(source, target);
            if(!processor.align(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>())))
            {
                RCLCPP_ERROR(this->get_logger(), "this was not supposed to happen tbh");
                return;
            }
            //giganigga.visualize();

            RCLCPP_INFO(this->get_logger(), "The ICP score is: %f", processor.score());

            std::stringstream ss;
            ss << processor.transform();

            RCLCPP_INFO(this->get_logger(), "The transform for this ICP iteration is: \n[%s]", ss.str().c_str());

            PCLPointCloudToROS(*(processor.source_), source_);
            PCLPointCloudToROS(*(processor.target_), target_);
            PCLPointCloudToROS(*(processor.result_), aligned_);
        }

        inline void PCLPointCloudToROS(pcl::PointCloud<pcl::PointXYZ>& src, sensor_msgs::msg::PointCloud2& tar)
        {
            pcl::PCLPointCloud2 src_temp;
            pcl::toPCLPointCloud2(src, src_temp);
            pcl_conversions::fromPCL(src_temp, tar);
            return;
        }

        void publishClouds() {
            source_.header.stamp = this->now();
            source_.header.frame_id = frame_id_;

            target_.header.stamp = this->now();
            target_.header.frame_id = frame_id_;

            aligned_.header.stamp = this->now();
            aligned_.header.frame_id = frame_id_;

            source_publisher_->publish(source_);
            target_publisher_->publish(target_);
            aligned_publisher_->publish(aligned_);

            RCLCPP_INFO(this->get_logger(), "Pointcloud messages published at %f", this->now().seconds());
        }

        ~VisualizationSpinner() noexcept { };

private:
        sensor_msgs::msg::PointCloud2 source_;
        sensor_msgs::msg::PointCloud2 target_;
        sensor_msgs::msg::PointCloud2 aligned_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr source_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_publisher_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_;

        double rate_in_seconds_;
        rmw_qos_profile_t odom_qos_profile_;

        rclcpp::TimerBase::SharedPtr msg_loop_trigger_;
        rclcpp::TimerBase::SharedPtr tf_loop_trigger_;

        std::string source_topic_;
        std::string target_topic_;
        std::string aligned_topic_;

        std::string frame_id_;
        std::string input_topic_;
        std::string odom_qos_type_;
        std::string filepath_;
        int idx1_;
        int idx2_;
    };
}