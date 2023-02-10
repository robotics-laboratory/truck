#pragma once

#include <iostream>
#include <functional>
#include <memory>

#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>

#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.hpp>

#include "cloud_matching.hpp"

namespace cmt {
    class OdometryComputer : public rclcpp::Node {
        //to do: more advanced parameters for icp algorithm
        typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;
    public:
        explicit OdometryComputer() :
        Node("odometry_computer")
        , scan_topic_("/lidar/scan")
        , odom_topic_("/cmt/odom")
        , odom_frame_("odom")
        , base_frame_("base_link")
        , show_path_(0)
        , odom_qos_profile_(rmw_qos_profile_sensor_data)
        , pose_(Eigen::Matrix4f::Identity()) //a temporary solution i guess
        {
            RCLCPP_INFO(this->get_logger(), "The node is initializing...");

            scan_topic_ = this->declare_parameter("scan_topic", scan_topic_);
            odom_topic_ = this->declare_parameter("odom_topic", odom_topic_);
            odom_frame_ = this->declare_parameter("odom_frame", odom_frame_);
            base_frame_ = this->declare_parameter("base_frame", base_frame_);
            show_path_ = this->declare_parameter<bool>("show_path", show_path_);

            RCLCPP_INFO(this->get_logger(), "Input scan topic: %s", scan_topic_.c_str());
            RCLCPP_INFO(this->get_logger(), "Base frame: %s", base_frame_.c_str());
            RCLCPP_INFO(this->get_logger(), "Output odometry topic: %s", odom_topic_.c_str());
            RCLCPP_INFO(this->get_logger(), "Specified odometry frame: %s", odom_frame_.c_str());
            RCLCPP_INFO(this->get_logger(), "Publish odometry path: %s", show_path_ ? "true" : "false");

            odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
                odom_topic_,
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(odom_qos_profile_)));

            scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                scan_topic_,
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(odom_qos_profile_)),
                std::bind(&OdometryComputer::callbackScan, this, std::placeholders::_1));

            tf_publisher_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

            warning_thread_ = this->create_wall_timer(
                std::chrono::duration<double>(5.0), std::bind(&OdometryComputer::executeWarningCheck, this));
            //to do: do smth with show path

            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

        ~OdometryComputer()
        {
            RCLCPP_INFO(this->get_logger(), "The node has been morbed successfully.");
        }

    private:
        void executeWarningCheck()
        {
            if (!callback_occured_) {
            RCLCPP_WARN(
                this->get_logger(),
                "Data from the topic /%s has not been received for more than 5 seconds."
                " Check that the input topic is being published correctly.",
                scan_topic_.c_str());
            }
        };

        bool processScan(PCLCloud scan, double timestamp)
        {
            Eigen::Matrix4f extract; 
            
            bool ok = cm.ICPMotionEstimation(extract, scan, last_scan_);

            if(!ok)
                {
                    rclcpp::Time cum = this->now();
                    RCLCPP_ERROR(this->get_logger(), "Failed to estimate the motion at %f.%lu", cum.seconds(), cum.nanoseconds());
                    return false;
                }

            double dt = abs(last_stamp_ - timestamp); //the measurement unit is seconds

            geometry_msgs::msg::TransformStamped tf_mold;
            tf_mold.transform = cmt::ROSTransformFromPCL(pose_ * extract);

            tf_mold.header.frame_id = odom_frame_;
            tf_mold.child_frame_id = base_frame_;
            tf_mold.header.stamp = this->now();

            nav_msgs::msg::Odometry odom_mold = computeOdometry(extract, dt); 
            odom_mold.header.frame_id = odom_frame_;
            odom_mold.child_frame_id = base_frame_;
            odom_mold.header.stamp = this->now();

            tf_publisher_->sendTransform(tf_mold);
            odom_publisher_->publish(odom_mold);

            RCLCPP_INFO(this->get_logger(), "Odometry processed from scan received at %f has been pubslished at %f", timestamp, this->now().seconds());

            return true;
        }

        bool obtainTransform(std::string from_frame_id
            , std::string to_frame_id
            , rclcpp::Time stamp
            , geometry_msgs::msg::TransformStamped::SharedPtr blank = std::make_shared<geometry_msgs::msg::TransformStamped>())
        {
            std::shared_ptr<std::string> err_msg;
            bool ret = tf_buffer_->canTransform(from_frame_id, to_frame_id, tf2_ros::fromMsg(stamp), tf2::durationFromSec(0.3), err_msg.get());
            if(!ret)
            {
                RCLCPP_ERROR(this->get_logger(), (*err_msg).c_str());
                return false;
            }

            try
            {
                *blank = tf_buffer_->lookupTransform(from_frame_id, to_frame_id, tf2_ros::fromMsg(stamp), tf2::durationFromSec(0.3)); //hardcoded duration for now
            }
            catch (tf2::TransformException& ex)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Could not fetch the transform from frame %s to frame %s at the time %f.%lu.", 
                    from_frame_id.c_str(),
                    to_frame_id.c_str(),
                    stamp.seconds(),
                    stamp.nanoseconds());
                return false;
            }
            return true;
        };

        void callbackScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            callback_occured_ = true;

            RCLCPP_INFO(this->get_logger(), "Scan acquired at %d.%u", msg->header.stamp.sec, msg->header.stamp.nanosec);

            //a check that allows for the correct conversion of the scan into a pointcloud
            if(!obtainTransform(
                base_frame_, 
                msg->header.frame_id, 
                rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec) 
                    + rclcpp::Duration::from_seconds(msg->ranges.size()*msg->time_increment)))
                RCLCPP_ERROR(this->get_logger(), "Scan processing failed, aborting odometry update.");

            //maybe represent it as a kd tree?
            PCLCloud scancloud = PCLCloudFromROS(msg);

            if(!init_collected_)
            {
                last_scan_ = scancloud;
                last_stamp_ = msg->header.stamp.sec;
                init_collected_ = true;
                RCLCPP_INFO(this->get_logger(), "Target scan set.");
                return;
            }

            bool ret = processScan(scancloud, msg->header.stamp.sec);

            if(ret)
            {
                last_stamp_ = msg->header.stamp.sec;
                last_scan_ = scancloud;
            }
        };

        PCLCloud PCLCloudFromROS(sensor_msgs::msg::LaserScan::SharedPtr prototype)
        {
            PCLCloud::Ptr output(new PCLCloud);
            sensor_msgs::msg::PointCloud2 mold;
            laser_geometry::LaserProjection projector;

            projector.transformLaserScanToPointCloud(prototype->header.frame_id, *prototype, mold, *tf_buffer_);
            pcl::fromROSMsg(mold, *output);

            return *output;
        };

        nav_msgs::msg::Odometry computeOdometry(Eigen::Matrix4f aff, double dt)
        {
            nav_msgs::msg::Odometry mold;

            float vx, vy, vz, vroll, vpitch, vyaw;
            pcl::getTranslationAndEulerAngles(Eigen::Affine3f(aff), vx, vy, vz, vroll, vpitch, vyaw);

            mold.twist.twist.linear.x = vx / dt;
			mold.twist.twist.linear.y = vy / dt;
			mold.twist.twist.linear.z = vz / dt;
			mold.twist.twist.angular.x = vroll / dt;
			mold.twist.twist.angular.y = vpitch / dt;
			mold.twist.twist.angular.z = vyaw / dt;

            pose_ *= aff;
            
            geometry_msgs::msg::Transform pose_data = cmt::ROSTransformFromPCL(pose_);

            mold.pose.pose.position.x = pose_data.translation.x;
			mold.pose.pose.position.y = pose_data.translation.y;
			mold.pose.pose.position.z = pose_data.translation.z;
			mold.pose.pose.orientation = pose_data.rotation;

            //primitive, but will do for now
            mold.pose.covariance.at(0) = 1e-3;
            mold.pose.covariance.at(7) = 1e-3;
            mold.pose.covariance.at(14) = 1e-3;
            mold.pose.covariance.at(21) = 1e-3;
            mold.pose.covariance.at(28) = 1e-3;
            mold.pose.covariance.at(35) = 1e-3;

            mold.twist.covariance.at(0) = 1e-3;
            mold.twist.covariance.at(7) = 1e-3;
            mold.twist.covariance.at(14) = 1e-3;
            mold.twist.covariance.at(21) = 1e-3;
            mold.twist.covariance.at(28) = 1e-3;
            mold.twist.covariance.at(35) = 1e-3;

            return mold;
        }

        //node parameters
        rmw_qos_profile_t odom_qos_profile_;
        std::string scan_topic_;
        std::string odom_topic_;
        std::string odom_frame_;
        std::string base_frame_;
        bool show_path_;

        //objects that allow for the logic of the class
        bool callback_occured_;
        bool init_collected_;
        PCLCloud last_scan_;
        double last_stamp_;
        Eigen::Matrix4f pose_;

        cmt::CloudMatcher cm;
        
        //rclcpp mechanics
        rclcpp::TimerBase::SharedPtr warning_thread_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    };
};