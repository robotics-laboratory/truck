#pragma once

#include <memory>
#include <sstream>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/conversions.h>

#include "laser_scan_matcher/cloud_matching.hpp"
#include "laser_scan_matcher/bag_cloud_extractor.hpp"

namespace cmt {
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
            , icp_max_corr_dist_(0.05)
            , icp_max_inter_dist_(1)
            , icp_tf_threshold_(1e-8)
            , icp_iter_threshold_(50) 
            , step_(10)
            , publish_(0)
        {
            RCLCPP_INFO(this->get_logger(), "The visualization node is initialized.");
            
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

            icp_tf_threshold_ = this->declare_parameter<double>("icp_tf_threshold", icp_tf_threshold_);
            icp_max_corr_dist_ = this->declare_parameter<double>("icp_max_corr_dist", icp_max_corr_dist_);
            icp_max_inter_dist_ = this->declare_parameter<double>("icp_max_inter_dist", icp_max_inter_dist_);
            icp_iter_threshold_ = this->declare_parameter<int>("icp_iter_threshold", icp_iter_threshold_);
            step_ = this->declare_parameter<int>("step", step_);

            RCLCPP_INFO(this->get_logger(), "Max distance between correspondent points: %f", icp_max_corr_dist_);
            RCLCPP_INFO(this->get_logger(), "Max allowed transformation distance: %f", icp_tf_threshold_);
            RCLCPP_INFO(this->get_logger(), "Max number of ICP iterations: %d", icp_iter_threshold_);
            RCLCPP_INFO(this->get_logger(), "Max allowed correspondence MSE change: %f", icp_max_inter_dist_);
            RCLCPP_INFO(this->get_logger(), "Step of iteration through the scan bag: %d", step_);

            launchICP(filepath_);
        }

        void publishTransform()
        {
            if(publish_) 
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
        }

        void launchICP(std::string filepath)
        {
            RosbagCloudExtractor reader(input_topic_);
            CloudMatcher processor = CloudMatcher(icp_max_corr_dist_, icp_max_inter_dist_, icp_tf_threshold_, icp_iter_threshold_);
            std::shared_ptr<std::vector<sensor_msgs::msg::LaserScan>> drain;

            reader.read(filepath, drain);

            for(int i = step_; i < drain->size(); i += step_)
            {
                sensor_msgs::msg::LaserScan psrc = (*drain)[i];
                sensor_msgs::msg::LaserScan ptar = (*drain)[i - step_];

                PCLCloud source = PCLCloudFromROSScan(psrc);
                PCLCloud target = PCLCloudFromROSScan(ptar);

                RCLCPP_INFO(this->get_logger(), "The timestamp of the source cloud: %d.%u", psrc.header.stamp.sec, psrc.header.stamp.nanosec);
                RCLCPP_INFO(this->get_logger(), "The timestamp of the target cloud: %d.%u", ptar.header.stamp.sec, ptar.header.stamp.nanosec);

                Eigen::Matrix4f transform;

                if(!processor.ICPMotionEstimation(transform, source, target))
                {
                    RCLCPP_ERROR(this->get_logger(), "This was not supposed to happen tbh");

                    source_.push(PCLCloudToROSCloud(source));
                    target_.push(PCLCloudToROSCloud(target));
                    aligned_.push(PCLCloudToROSCloud(PCLCloud()));

                    continue;
                }

                RCLCPP_INFO(this->get_logger(), "The ICP score is: %f", processor.getScore());

                std::stringstream ss;
                ss << transform;

                RCLCPP_INFO(this->get_logger(), "The transform for this ICP iteration is: \n[%s]", ss.str().c_str());

                PCLCloud aligned;
                processor.getAlignedScan(aligned);

                source_.push(PCLCloudToROSCloud(source));
                target_.push(PCLCloudToROSCloud(target));
                aligned_.push(PCLCloudToROSCloud(aligned));
            }
            RCLCPP_INFO(this->get_logger(), "The bag has ended", processor.getScore());
            publish_ = false;
        }

        PCLCloud PCLCloudFromROSScan(sensor_msgs::msg::LaserScan prototype)
        {
            PCLCloud::Ptr output(new PCLCloud);
            sensor_msgs::msg::PointCloud2 mold;
            laser_geometry::LaserProjection projector;

            projector.projectLaser(prototype, mold);
            pcl::fromROSMsg(mold, *output);

            return *output;
        }

        void publishClouds() {
            if(!source_.size())
                return;
            sensor_msgs::msg::PointCloud2 src = source_.front();
            sensor_msgs::msg::PointCloud2 tar = target_.front();
            sensor_msgs::msg::PointCloud2 ali = aligned_.front();

            source_.pop();
            target_.pop();
            aligned_.pop();

            src.header.stamp = this->now();
            src.header.frame_id = frame_id_;

            tar.header.stamp = this->now();
            tar.header.frame_id = frame_id_;

            ali.header.stamp = this->now();
            ali.header.frame_id = frame_id_;

            source_publisher_->publish(src);
            target_publisher_->publish(tar);
            aligned_publisher_->publish(ali);

            RCLCPP_INFO(this->get_logger(), "Pointcloud messages published at %f", this->now().seconds());
        }

        ~VisualizationSpinner() noexcept { };

private:
        std::queue<sensor_msgs::msg::PointCloud2> source_;
        std::queue<sensor_msgs::msg::PointCloud2> target_;
        std::queue<sensor_msgs::msg::PointCloud2> aligned_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr source_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_publisher_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_;

        double rate_in_seconds_;
        rmw_qos_profile_t odom_qos_profile_;
        bool publish_;

        rclcpp::TimerBase::SharedPtr msg_loop_trigger_;
        rclcpp::TimerBase::SharedPtr tf_loop_trigger_;

        //icp parameters
        double icp_tf_threshold_; 
        double icp_max_corr_dist_;
        double icp_max_inter_dist_; 
        int icp_iter_threshold_; 
        int step_;

        std::string source_topic_;
        std::string target_topic_;
        std::string aligned_topic_;

        std::string frame_id_;
        std::string input_topic_;
        std::string odom_qos_type_;
        std::string filepath_;
    };
}