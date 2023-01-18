#pragma once

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <laser_geometry/laser_geometry.hpp>

class CloudMatcher : public pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>
{
    typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

    public:
    CloudMatcher(Cloud::Ptr one, Cloud::Ptr another) : executed_(0)
    {
        source_ = Cloud::Ptr(new Cloud(*one));
        target_ = Cloud::Ptr(new Cloud(*another));

        setInputSource(source_);
        setInputTarget(target_);
    }

    CloudMatcher(sensor_msgs::msg::LaserScan in, sensor_msgs::msg::LaserScan out)
    {
        source_ = Cloud::Ptr(new Cloud);
        target_ = Cloud::Ptr(new Cloud);

        sensor_msgs::msg::PointCloud2 in_cloud_msg;
        sensor_msgs::msg::PointCloud2 out_cloud_msg;

        laser_geometry::LaserProjection projector_;
        projector_.projectLaser(in, in_cloud_msg);
        projector_.projectLaser(out, out_cloud_msg);

        pcl::PCLPointCloud2 in_temp_cloud;
        pcl::PCLPointCloud2 out_temp_cloud;
        pcl_conversions::toPCL(in_cloud_msg, in_temp_cloud);
        pcl_conversions::toPCL(out_cloud_msg, out_temp_cloud);

        pcl::fromPCLPointCloud2(in_temp_cloud, *source_);
        pcl::fromPCLPointCloud2(out_temp_cloud, *target_);

        setInputSource(source_);
        setInputTarget(target_);
    }

    bool align(Cloud::Ptr temp)
    {
        executed_ = true;
        IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::align(*temp);
        result_ = temp;
        return hasConverged();
    }

    double score()
    {
        if(!executed_)
            throw std::runtime_error("le fucky wucky happened");
        return getFitnessScore();
    }

    Cloud result()
    {
        return *result_;
    }

    Eigen::Matrix4f transform()
    {
        if(!executed_)
            throw std::runtime_error("le fucky wucky happened");
        return getFinalTransformation();
    }

    void visualize()
    {
        pcl::visualization::PCLVisualizer viewer("cum obama fake jordans");
        int id0 = 0;
        int id1 = 1;

        viewer.createViewPort (0.0, 0.0, 0.5, 1.0, id0);
        viewer.createViewPort (0.5, 0.0, 1.0, 1.0, id1);

        double background_colour = 0;
        double text_colour = 1;

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colour_src( //white
            source_, 255, 255, 255);
        
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colour_tar( //green
            target_, 20, 180, 20);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colour_res( //red
            result_, 180, 20, 20);

        viewer.addPointCloud(source_, colour_src, "src_id0", id0);
        viewer.addPointCloud(source_, colour_src, "src_id1", id1);
        viewer.addPointCloud(target_, colour_tar, "tar_id0", id0);
        viewer.addPointCloud(result_, colour_res, "res_id1", id1);

        viewer.setBackgroundColor(background_colour, background_colour, background_colour, id0);
        viewer.setBackgroundColor(background_colour, background_colour, background_colour, id1);
        viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
        viewer.setSize (1280, 1024);

        while (!viewer.wasStopped ())
        {
            viewer.spinOnce ();
        }
        return;
    }

    Cloud::Ptr source_;
    Cloud::Ptr target_;
    Cloud::Ptr result_;
private:
    bool executed_;
};