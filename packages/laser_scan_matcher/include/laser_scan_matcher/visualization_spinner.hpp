#pragma once

#include <memory>
#include <sstream>
#include <vector>

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


//make it extra oop, create an interface node ffs
namespace cmt {
    class VisualizationSpinner : public rclcpp::Node {
public:
        explicit VisualizationSpinner() 
            : Node("visualization_spinner")
            , scan_topic_("/lidar/scan")
            , scan_qos_type_("SENSOR_DATA")
            , pub_qos_type_("DEFAULT")
//            , frame_id_("visualisation_view")            
            , rosbag_mode_(0)
            , filepath_("")             
            , rate_in_seconds_(0.5)  
            , read_step_(10)       
            , icp_tf_threshold_(1e-8)
            , icp_max_corr_dist_(0.05)
            , icp_max_inter_dist_(1)
            , icp_iter_threshold_(50) 
            , scan_qos_profile_(rmw_qos_profile_default)
            , pub_qos_profile_(rmw_qos_profile_sensor_data)              
            , init_collected_(0)
            , callback_occurred_(0)
            , loop_counter_(0)
            , source_topic_("source_cloud")
            , target_topic_("target_cloud")
            , aligned_topic_("aligned_cloud")
        {
            RCLCPP_INFO(this->get_logger(), "The transform visualizer node has begun initializing.");

            scan_topic_ = this->declare_parameter("scan_topic", scan_topic_.c_str());

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
                    "Error: Invalid value assigned to the scan QoS parameter. Setting QoS profile to default.");
            }

            pub_qos_type_ = this->declare_parameter("pub_qos_profile", pub_qos_type_);

            if (strcmp(pub_qos_type_.c_str(), "DEFAULT") == 0) {
                pub_qos_profile_ = rmw_qos_profile_default;
            }
            else if (strcmp(pub_qos_type_.c_str(), "SENSOR_DATA") == 0) {
                pub_qos_profile_ = rmw_qos_profile_sensor_data;
            }
            else if (strcmp(pub_qos_type_.c_str(), "SYSTEM_DEFAULT") == 0) {
                pub_qos_profile_ = rmw_qos_profile_system_default;
            }
            else {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Error: Invalid value assigned to the odometry QoS parameter. Setting QoS profile to default.");
                pub_qos_type_ = "DEFAULT";
            }

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

            RCLCPP_INFO(this->get_logger(), "Input can topic: %s", scan_topic_.c_str());
            RCLCPP_INFO(this->get_logger(), "Input QoS settings profile: %s", scan_qos_type_.c_str());
            RCLCPP_INFO(this->get_logger(), "Publisher QoS settings profile: %s", pub_qos_type_.c_str());
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

            source_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                source_topic_, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(pub_qos_profile_)).reliability(pub_qos_profile_.reliability));
            target_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                target_topic_, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(pub_qos_profile_)).reliability(pub_qos_profile_.reliability));
            aligned_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                aligned_topic_, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(pub_qos_profile_)).reliability(pub_qos_profile_.reliability));
            tf_publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

            cm_ = CloudMatcher(icp_max_corr_dist_, icp_max_inter_dist_, icp_tf_threshold_, icp_iter_threshold_);

            if(rosbag_mode_)
            {
                readProcess(filepath_);
                msg_loop_trigger_ = this->create_wall_timer(
                    std::chrono::duration<double>(rate_in_seconds_),
                    std::bind(&VisualizationSpinner::loopProcess, this));
            }
            else
            {
                scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                    scan_topic_,
                    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(scan_qos_profile_)).reliability(scan_qos_profile_.reliability),
                    std::bind(&VisualizationSpinner::callbackScan, this, std::placeholders::_1));
                
                warning_thread_ = this->create_wall_timer(
                    std::chrono::duration<double>(5.0), std::bind(&VisualizationSpinner::executeWarningCheck, this));
            }
        }

        ~VisualizationSpinner() noexcept { };
private:
        void executeWarningCheck()
        {
            if (!rosbag_mode_ && !callback_occurred_) {
            RCLCPP_WARN(
                this->get_logger(),
                "Data from the topic %s has not been received for more than 5 seconds."
                " Check that the input topic is being published correctly.",
                scan_topic_.c_str());
            }
        };

        inline std::string eigenToString(const Eigen::MatrixXf& cumballz)
        {
            std::stringstream ss;
            ss << cumballz;
            return ss.str();
        }

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

            RCLCPP_INFO(this->get_logger(), "The transform for this ICP iteration is: \n[%s]", eigenToString(extract).c_str());

            publishClouds(PCLCloudToROSCloud(last_scan_), PCLCloudToROSCloud(scan), PCLCloudToROSCloud(cm_.getAlignedScan()));

            RCLCPP_INFO(this->get_logger(), "ICP processed on reference received at %f has been pubslished at %f,\
                \nEuclidian fitness score: %f\nTransformation: \n[%s]"
                , timestamp 
                , this->now().seconds()
                , cm_.getScore()
                , eigenToString(extract).c_str());

            return true;
        }

        void callbackScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            callback_occurred_ = true;

            RCLCPP_INFO(this->get_logger(), "Scan acquired at %d.%u", msg->header.stamp.sec, msg->header.stamp.nanosec);

            //maybe represent it as a kd tree?
            PCLCloud scancloud = PCLCloudFromROSScan(msg);
            double stamp = rclcpp::Time(msg->header.stamp).seconds();

            if(!init_collected_)
            {
                last_scan_ = scancloud;
                last_stamp_ = stamp;
                init_collected_ = true;
                RCLCPP_INFO(this->get_logger(), "Target scan set.");
                return;
            }

            frame_id_ = msg->header.frame_id;
            bool ret = processScan(scancloud, stamp);

            if(ret)
            {
                last_stamp_ = stamp;
                last_scan_ = scancloud;
            }
            callback_occurred_ = false;
        }

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

        PCLCloud PCLCloudFromROSScan(sensor_msgs::msg::LaserScan::SharedPtr prototype)
        {
            PCLCloud::Ptr output(new PCLCloud);
            sensor_msgs::msg::PointCloud2 mold;
            laser_geometry::LaserProjection projector;

            projector.projectLaser(*prototype, mold);
            pcl::fromROSMsg(mold, *output);

            return *output;
        };

        void publishClouds(sensor_msgs::msg::PointCloud2 src, sensor_msgs::msg::PointCloud2 tar, sensor_msgs::msg::PointCloud2 ali) {
            src.header.stamp = this->now();
            src.header.frame_id = frame_id_;

            tar.header.stamp = this->now();
            tar.header.frame_id = frame_id_;

            ali.header.stamp = this->now();
            ali.header.frame_id = frame_id_;

            source_publisher_->publish(src);
            target_publisher_->publish(tar);
            aligned_publisher_->publish(ali);

            RCLCPP_INFO(this->get_logger(), "Processed pointcloud messages published at %f", this->now().seconds());
        }

        //node parameters
        std::string scan_topic_;
        std::string scan_qos_type_;
        std::string pub_qos_type_;              
        std::string frame_id_;  
        std::string filepath_;
        bool rosbag_mode_;
        double rate_in_seconds_;
        
        //icp parameters
        double icp_tf_threshold_; 
        double icp_max_corr_dist_;
        double icp_max_inter_dist_; 
        int icp_iter_threshold_; 
        int read_step_;

        //objects that allow for the logic of the class
        rmw_qos_profile_t scan_qos_profile_;
        rmw_qos_profile_t pub_qos_profile_;
        std::string source_topic_;
        std::string target_topic_;
        std::string aligned_topic_;
        bool init_collected_;
        bool callback_occurred_;
        int loop_counter_;
        
        cmt::CloudMatcher cm_;
        std::vector<sensor_msgs::msg::LaserScan::SharedPtr> data_;
        PCLCloud last_scan_;
        double last_stamp_;
        
        //rclcpp mechanics
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
        //make this a laserscan
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr source_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_publisher_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_;

        rclcpp::TimerBase::SharedPtr warning_thread_;
        rclcpp::TimerBase::SharedPtr msg_loop_trigger_;
    };
}
