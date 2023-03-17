#pragma once

#include <iostream>
#include <functional>
#include <memory>
#include <sstream>

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
#include "laser_scan_matcher/bag_cloud_extractor.hpp"

namespace cmt {
    class OdometryComputer : public rclcpp::Node {
        //to do: more advanced parameters for icp algorithm
        typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;
    public:
        explicit OdometryComputer() :
        Node("odometry_computer")
        , scan_topic_("/lidar/scan")
        , odom_topic_("/cmt/odom")
        , scan_qos_type_("SENSOR_DATA")
        , odom_frame_("odom")
        , base_frame_("base_link")
        , show_path_(0)
        , rosbag_mode_(0)
        , filepath_("") 
        , rate_in_seconds_(0.5)
        , read_step_(10)
        , icp_tf_threshold_(1e-8)
        , icp_max_corr_dist_(0.05)
        , icp_max_inter_dist_(1)
        , icp_iter_threshold_(50)
        , odom_qos_profile_(rmw_qos_profile_sensor_data)
        , scan_qos_profile_(rmw_qos_profile_default)
        , callback_occurred_(0)
        , init_collected_(0)
        , loop_counter_(0)
        , pose_(Eigen::Matrix4f::Identity()) //a temporary solution i guess
        {
            RCLCPP_INFO(this->get_logger(), "The odometry computer node has begun initializing.");

            scan_topic_ = this->declare_parameter("scan_topic", scan_topic_);
            odom_topic_ = this->declare_parameter("odom_topic", odom_topic_);

            scan_qos_type_ = this->declare_parameter("scan_qos_profile", scan_qos_type_);

            if (strcmp(scan_qos_type_.c_str(), "DEFAULT") == 0) {
                scan_qos_profile_ = rmw_qos_profile_default;
            }
            else if (strcmp(scan_qos_type_.c_str(), "SENSOR_DATA") == 0) {
                scan_qos_profile_ = rmw_qos_profile_sensor_data;
            }
            else if (strcmp(scan_qos_type_.c_str(), "SYSTEM_DEFAULT") == 0) {
                scan_qos_profile_ = rmw_qos_profile_system_default;
            }
            else {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Error: Invalid value assigned to the QoS parameter. Setting QoS profile to default.");
                scan_qos_type_ = "DEFAULT";
            }

            odom_frame_ = this->declare_parameter("odom_frame", odom_frame_);
            base_frame_ = this->declare_parameter("base_frame", base_frame_);
            show_path_ = this->declare_parameter<bool>("show_path", show_path_); //not usable yet

            rosbag_mode_ = this->declare_parameter<bool>("rosbag_mode", rosbag_mode_);

            filepath_ = this->declare_parameter("filepath", filepath_);
            if(rosbag_mode_ && strcmp(filepath_.c_str(), "") == 0)
            {
                throw std::runtime_error("The path to a .db3 rosbag file must be specified.");
            }

            rate_in_seconds_ = this->declare_parameter<double>("rate_in_seconds", rate_in_seconds_);
            read_step_ = this->declare_parameter<int>("read_step", read_step_);

            icp_tf_threshold_ = this->declare_parameter<double>("icp_tf_threshold", icp_tf_threshold_);
            icp_max_corr_dist_ = this->declare_parameter<double>("icp_max_corr_dist", icp_max_corr_dist_);
            icp_max_inter_dist_ = this->declare_parameter<double>("icp_max_inter_dist", icp_max_inter_dist_);
            icp_iter_threshold_ = this->declare_parameter<int>("icp_iter_threshold", icp_iter_threshold_);
            
            RCLCPP_INFO(this->get_logger(), "QoS settings profile for the input topic: %s", scan_qos_type_.c_str());
            RCLCPP_INFO(this->get_logger(), "Input scan topic: %s", scan_topic_.c_str());
            RCLCPP_INFO(this->get_logger(), "Base frame: %s", base_frame_.c_str());
            RCLCPP_INFO(this->get_logger(), "Output odometry topic: %s", odom_topic_.c_str());
            RCLCPP_INFO(this->get_logger(), "Specified odometry frame: %s", odom_frame_.c_str());
            RCLCPP_INFO(this->get_logger(), "Publish odometry path: %s", show_path_ ? "true" : "false");
            RCLCPP_INFO(this->get_logger(), "Max distance between correspondent points: %f", icp_max_corr_dist_);
            RCLCPP_INFO(this->get_logger(), "Max allowed transformation distance: %f", icp_tf_threshold_);
            RCLCPP_INFO(this->get_logger(), "Max number of ICP iterations: %d", icp_iter_threshold_);
            RCLCPP_INFO(this->get_logger(), "Max allowed correspondence MSE change: %f", icp_max_inter_dist_);

            RCLCPP_INFO(this->get_logger(), "Input reading mode: %s", rosbag_mode_ ? "rosbag" : "online");
            if(rosbag_mode_)
            {
                RCLCPP_INFO(this->get_logger(), "Rosbag filepath: %s", filepath_.c_str());
                RCLCPP_INFO(this->get_logger(), "Rate of the publication in seconds: %f", rate_in_seconds_);
                RCLCPP_INFO(this->get_logger(), "Step of iteration through the bag scans: %d", read_step_);
            }

            tf_publisher_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
            odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
                odom_topic_,
                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(odom_qos_profile_)).reliability(odom_qos_profile_.reliability));
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            cm_ = CloudMatcher(icp_max_corr_dist_, icp_max_inter_dist_, icp_tf_threshold_, icp_iter_threshold_);

            if(rosbag_mode_)
            {
                readProcess(filepath_);
                msg_loop_trigger_ = this->create_wall_timer(
                    std::chrono::duration<double>(rate_in_seconds_),
                    std::bind(&OdometryComputer::loopProcess, this));
            }
            else
            {
                scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                    scan_topic_,
                    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(scan_qos_profile_)).reliability(scan_qos_profile_.reliability),
                    std::bind(&OdometryComputer::callbackScan, this, std::placeholders::_1));
                
                warning_thread_ = this->create_wall_timer(
                    std::chrono::duration<double>(5.0), std::bind(&OdometryComputer::executeWarningCheck, this));
            }
            //to do: do smth with show path
        }

        ~OdometryComputer()
        {
            RCLCPP_INFO(this->get_logger(), "The node has been morbed successfully.");
        }

    private:
        void loopProcess()
        {
            if(data_.size() == 0)
            {
                throw std::runtime_error("Cannot run the ICP process with no data available.");
            }

            if(loop_counter_ == 0)
            {
                init_collected_ = false;
            }

            callbackScan(data_[loop_counter_]);

            loop_counter_ = loop_counter_ + 1 < data_.size() ? loop_counter_ + 1 : 0;
        }

        void readProcess(std::string filepath)
        {
            RCLCPP_INFO(this->get_logger(), "The reading process has started at %f", this->now().seconds());

            RosbagCloudExtractor reader(scan_topic_);
            std::shared_ptr<std::vector<sensor_msgs::msg::LaserScan::SharedPtr>> drain;

            reader.read(filepath, drain);

            for(int i = 0; i < drain->size(); i += read_step_)
            {
                data_.push_back((*drain)[i]);
            }
            
            if(data_.size() > 0)
                RCLCPP_INFO(this->get_logger(), "The reading process has been finished successfully at %f, %ld messages read in total"
                    , this->now().seconds()
                    , data_.size());
            else
                RCLCPP_ERROR(this->get_logger(), "No messages have been acquired from the bag!");
        }

        std::string eigenToString(const Eigen::MatrixXf& cumballz)
        {
            std::stringstream ss;
            ss << cumballz;
            return ss.str();
        }

        std::string transformToString(const geometry_msgs::msg::Transform& cumballz)
        {
            std::stringstream ss;
            ss << "Translation vector:\t" 
                << cumballz.translation.x << '\t'
                << cumballz.translation.y << '\t'
                << cumballz.translation.z
                << '\n';
            ss << "Rotation quaternion:\t" 
                << cumballz.rotation.x << '\t'
                << cumballz.rotation.y << '\t'
                << cumballz.rotation.z << '\t'
                << cumballz.rotation.w;
            return ss.str();
        }

        void executeWarningCheck()
        {
            if (!callback_occurred_) {
            RCLCPP_WARN(
                this->get_logger(),
                "Data from the topic %s has not been received for more than 5 seconds."
                " Check that the input topic is being published correctly.",
                scan_topic_.c_str());
            }
        };

        bool processScan(PCLCloud scan, double timestamp)
        {
            Eigen::Matrix4f extract; 
            
            bool ok = cm_.ICPMotionEstimation(extract, last_scan_, scan);

            if(!ok)
                {
                    rclcpp::Time cum = this->now();
                    RCLCPP_ERROR(this->get_logger(), "Failed to estimate the motion at %f.%lu", cum.seconds(), cum.nanoseconds());
                    return false;
                }

            double dt = abs(last_stamp_ - timestamp); //the measurement unit is seconds
            RCLCPP_DEBUG(this->get_logger(), "processing scan at %f, last stamp: %f, dt: %f", 
                timestamp,
                last_stamp_,
                dt);

            RCLCPP_DEBUG(this->get_logger(), "Pose prior to update at %f:\n%s", 
                timestamp,
                eigenToString(pose_).c_str());
            RCLCPP_DEBUG(this->get_logger(), "Extracted tf at %f:\n%s", 
                timestamp,
                eigenToString(extract).c_str());

            pose_ = extract * pose_;

            RCLCPP_DEBUG(this->get_logger(), "Pose after the update at %f:\n%s", 
                timestamp,
                eigenToString(pose_).c_str());

            geometry_msgs::msg::TransformStamped tf_mold;
            tf_mold.transform = cmt::ROSTransformFromPCL(pose_);

            RCLCPP_DEBUG(this->get_logger(), "Updated pose in the ROS struct at %f:\n%s", 
                timestamp,
                transformToString(tf_mold.transform).c_str());

            tf_mold.header.frame_id = odom_frame_;
            tf_mold.child_frame_id = base_frame_;
            tf_mold.header.stamp = this->now();

            nav_msgs::msg::Odometry odom_mold = computeOdometry(extract, dt); 
            odom_mold.header.frame_id = odom_frame_;
            odom_mold.child_frame_id = base_frame_;
            odom_mold.header.stamp = this->now();

            tf_publisher_->sendTransform(tf_mold);
            odom_publisher_->publish(odom_mold);

            RCLCPP_INFO(this->get_logger(), "Odometry processed from scan received at %f has been pubslished at %f\nEuclidian fitness score for ICP: %f",
                timestamp, 
                this->now().seconds(),
                cm_.getScore());

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
            callback_occurred_ = true;

            RCLCPP_INFO(this->get_logger(), "Scan acquired at %d.%u", msg->header.stamp.sec, msg->header.stamp.nanosec);

            //maybe represent it as a kd tree?
            PCLCloud scancloud = PCLCloudFromROS(msg);
            double stamp = rclcpp::Time(msg->header.stamp).seconds();

            if(!init_collected_)
            {
                last_scan_ = scancloud;
                last_stamp_ = stamp;
                init_collected_ = true;
                RCLCPP_INFO(this->get_logger(), "Target scan set.");
                return;
            }

            bool ret = processScan(scancloud, stamp);

            if(ret)
            {
                last_stamp_ = stamp;
                last_scan_ = scancloud;
            }

            callback_occurred_ = false;
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

            mold.twist.twist.linear.x = vx; /// dt;
			mold.twist.twist.linear.y = vy; /// dt;
			mold.twist.twist.linear.z = 0;
			mold.twist.twist.angular.x = 0;
			mold.twist.twist.angular.y = 0;
			mold.twist.twist.angular.z = vyaw; /// dt;
            
            geometry_msgs::msg::Transform pose_data = cmt::ROSTransformFromPCL(pose_);

            mold.pose.pose.position.x = pose_data.translation.x;
			mold.pose.pose.position.y = pose_data.translation.y;
			mold.pose.pose.position.z = 0;
			mold.pose.pose.orientation = pose_data.rotation;

            //primitive, but will do for now
            mold.pose.covariance.at(0) = 1e-3;
            mold.pose.covariance.at(7) = 1e-3;

            mold.pose.covariance.at(21) = 1e-3;
            mold.pose.covariance.at(28) = 1e-3;
            mold.pose.covariance.at(35) = 1e-3;

            mold.twist.covariance.at(0) = 1e-3;
            mold.twist.covariance.at(7) = 1e-3;
            mold.twist.covariance.at(35) = 1e-3;

            return mold;
        }

        //node parameters
        std::string scan_topic_;
        std::string odom_topic_;
        std::string odom_frame_;
        std::string base_frame_;
        std::string scan_qos_type_;
        bool show_path_;
        bool rosbag_mode_;
        std::string filepath_;
        double rate_in_seconds_;
        int read_step_;

        //icp parameters
        double icp_tf_threshold_; 
        double icp_max_corr_dist_;
        double icp_max_inter_dist_; 
        int icp_iter_threshold_; 

        //objects that allow for the logic of the class
        rmw_qos_profile_t odom_qos_profile_;
        rmw_qos_profile_t scan_qos_profile_;
        bool init_collected_;
        bool callback_occurred_;
        int loop_counter_;

        cmt::CloudMatcher cm_;
        std::vector<sensor_msgs::msg::LaserScan::SharedPtr> data_;
        PCLCloud last_scan_;
        double last_stamp_;
        Eigen::Matrix4f pose_;
        
        //rclcpp mechanics

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;        
        
        rclcpp::TimerBase::SharedPtr warning_thread_;
        rclcpp::TimerBase::SharedPtr msg_loop_trigger_;
    };
};
