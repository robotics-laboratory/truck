#pragma once

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2_eigen/tf2_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.hpp>

namespace cmt {
    typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

    class CloudMatcher {

    public:
        CloudMatcher(double icp_max_corr_dist, double icp_max_inter_dist, double icp_tf_threshold, double icp_iter_threshold)
            : icp_max_corr_dist_(icp_max_corr_dist) 
            , icp_max_inter_dist_(icp_max_inter_dist)
            , icp_tf_threshold_(icp_tf_threshold)
            , icp_iter_threshold_(icp_iter_threshold)
            , last_reg_(nullptr)
        {}

        CloudMatcher() : CloudMatcher(0.05, 1, 1e-8, 50) {};

        bool ICPMotionEstimation(Eigen::Matrix4f& mold, PCLCloud source, PCLCloud target)
        {
            PCLCloud::Ptr psrc, ptar, preg;
            
            //downsample(psrc, source);
            //downsample(ptar, target);
            psrc = PCLCloud::Ptr(new PCLCloud(source));
            ptar = PCLCloud::Ptr(new PCLCloud(target));
            preg = PCLCloud::Ptr(new PCLCloud());

            icp_proc_.setMaxCorrespondenceDistance(icp_max_corr_dist_);
            icp_proc_.setMaximumIterations(icp_iter_threshold_);
            icp_proc_.setTransformationEpsilon(icp_tf_threshold_);
            icp_proc_.setEuclideanFitnessEpsilon(icp_max_inter_dist_);

            icp_proc_.setInputSource(psrc);
            icp_proc_.setInputTarget(ptar);

            icp_proc_.align(*preg);

            if(!icp_proc_.hasConverged())
                return false;
            
            mold = icp_proc_.getFinalTransformation(); 
            last_reg_ = preg;
            return true;
        }

        PCLCloud getAlignedScan()
        {
            if(last_reg_)
            {
                return *last_reg_;
            }
            return PCLCloud();
        }

        double getScore() { return icp_proc_.getFitnessScore(); }

    private:
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_proc_;
        PCLCloud::Ptr last_reg_;

        double icp_tf_threshold_; //threshold for the difference between transformations
        double icp_max_corr_dist_; //comes in meters
        double icp_max_inter_dist_; //the percentage change from the previous correspondence set MSE for the current one
        int icp_iter_threshold_; //maximum number of iterations
    };

    void downsample(PCLCloud::Ptr& mold, PCLCloud cloud)
    {
        //fill in
        return;
    }

    sensor_msgs::msg::PointCloud2 PCLCloudToROSCloud(pcl::PointCloud<pcl::PointXYZ> src)
    {
        sensor_msgs::msg::PointCloud2 output;

        pcl::PCLPointCloud2 src_temp;
        pcl::toPCLPointCloud2(src, src_temp);
        pcl_conversions::fromPCL(src_temp, output);

        return output;
    }

    geometry_msgs::msg::Transform ROSTransformFromPCL(Eigen::Matrix4f prototype)
    {
        geometry_msgs::msg::Transform result = tf2::eigenToTransform(Eigen::Transform<double, 3, 2>(Eigen::Affine3f(prototype))).transform;

        //using hint from rtab-map and normalizing the quaternion
        long double norm = 1.0 / sqrt(result.rotation.x * result.rotation.x + 
            result.rotation.y * result.rotation.y + 
            result.rotation.z * result.rotation.z + 
            result.rotation.w * result.rotation.w);
		result.rotation.x *= norm;
		result.rotation.y *= norm;
		result.rotation.z *= norm;
		result.rotation.w *= norm;

        return result;
    }
}  // namespace cmt