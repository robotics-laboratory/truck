#include "occupancy_grid/occupancy_grid_node.h"

#include "common/math.h"

#include <cstdint>
#include <functional>

#include <algorithm>
#include <limits>
#include <cmath>
#include <vector>
#include <math.h>
#include <chrono>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace {
    template<typename T>
    struct DepthTraits {};

    template<>
    struct DepthTraits<uint16_t>
    {
        static inline bool valid(uint16_t depth) {return depth != 0;}
        static inline float toMeters(uint16_t depth) {return depth * 0.001f;}   // originally mm
        static inline uint16_t fromMeters(float depth) {return (depth * 1000.0f) + 0.5f;}        
    };

    // Handles uint16 depths
    template<typename T>
    void convertDepth(
        const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
        sensor_msgs::msg::PointCloud2::SharedPtr & cloud_in,
        sensor_msgs::msg::PointCloud2::SharedPtr & cloud_out,
        const image_geometry::PinholeCameraModel & model,
        const geometry_msgs::msg::TransformStamped & t_in,
        double range_max = 0.0
        )//const geometry_msgs::msg::TransformStamped & t_in,
{
    // Use correct principal point from calibration
    float center_x = model.cx();
    float center_y = model.cy();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    double unit_scaling = DepthTraits<T>::toMeters(T(1) );
    float constant_x = unit_scaling / model.fx();
    float constant_y = unit_scaling / model.fy();
    float bad_point = std::numeric_limits<float>::quiet_NaN();
    
    //transform point cloud to lidar frame
    cloud_out = cloud_in;
    cloud_out->header = t_in.header;
    
    auto translation = Eigen::Translation3f(
         static_cast<float>(t_in.transform.translation.x),
         static_cast<float>(t_in.transform.translation.y),
         static_cast<float>(t_in.transform.translation.z)
    );
    auto quaternion = Eigen::Quaternion<float>(
        static_cast<float>(t_in.transform.rotation.w),
        static_cast<float>(t_in.transform.rotation.x),
        static_cast<float>(t_in.transform.rotation.y),
        static_cast<float>(t_in.transform.rotation.z)
    );

    Eigen::Transform<float,3, Eigen::Affine> t = translation * quaternion;
    Eigen::Vector3f point;
    
    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_in, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_in, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_in, "z");

    sensor_msgs::PointCloud2Iterator<float> x_out(*cloud_out, "x");
    sensor_msgs::PointCloud2Iterator<float> y_out(*cloud_out, "y");
    sensor_msgs::PointCloud2Iterator<float> z_out(*cloud_out, "z");

    const T * depth_row = reinterpret_cast<const T *>(&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof(T);
    for (int v = 0; v < static_cast<int>(cloud_in->height); ++v, depth_row += row_step) {
        for (int u = 0; u < static_cast<int>(cloud_in->width); ++u, ++iter_x, ++iter_y, ++iter_z,++x_out, ++y_out, ++z_out) {
        T depth = depth_row[u];

    // Missing points denoted by NaNs
      if (!DepthTraits<T>::valid(depth)) {
        if (range_max != 0.0) {
          depth = DepthTraits<T>::fromMeters(range_max);
        } else {
          *iter_x = *iter_y = *iter_z = bad_point;
          continue;
        }
      }

    // Fill in XYZ
    
    *iter_x = ((u - center_x) * depth * constant_x);
    *iter_y = ((v - center_y) * depth * constant_y);
    *iter_z = (DepthTraits<T>::toMeters(depth));
    point =  t * Eigen::Vector3f(*iter_x, *iter_y, *iter_z);
    *x_out = point.x();
    *y_out = point.y();
    *z_out = point.z();
    }    
  }
}
}

namespace truck::occupancy_grid {

OccupancyGridNode::OccupancyGridNode() : Node("occupancy_grid") {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    RCLCPP_INFO(this->get_logger(), "qos %d", qos);

    params_ = {this->declare_parameter("resolution", 0.1), this->declare_parameter("radius", 20.0), this->declare_parameter("enable_lidar_grid", false), this->declare_parameter("enable_camera_grid", false), this->declare_parameter("enable_camera_cloud", false)};

    RCLCPP_INFO(this->get_logger(), "resolution: %.2fm", params_.resolution);
    RCLCPP_INFO(this->get_logger(), "radius: %.2fm", params_.radius);
    RCLCPP_INFO(this->get_logger(), "enable_camera_cloud %d", params_.enable_camera_cloud);
    RCLCPP_INFO(this->get_logger(), "enable_lidar_grid %d", params_.enable_lidar_grid);
    RCLCPP_INFO(this->get_logger(), "enable_camera_grid %d", params_.enable_camera_grid);

    slot_.image_handle = this->create_subscription<sensor_msgs::msg::Image>("/camera/depth/image_rect_raw", qos, 
        std::bind(&OccupancyGridNode::handleImageScan, this, std::placeholders::_1));

    slot_.camera_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/depth/camera_info", 10,
        std::bind(&OccupancyGridNode::handleCameraInfo, this, std::placeholders::_1));
    
    slot_.camera_handle = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/processed_image", rclcpp::QoS(1).reliability(qos), 
        std::bind(&OccupancyGridNode::handleCameraScan, this, std::placeholders::_1));
    
    slot_.lidar_handle = create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar/scan",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&OccupancyGridNode::handleLaserScan, this, std::placeholders::_1));
        
    signal_.processed_image = this->create_publisher<sensor_msgs::msg::Image>("/camera/processed_image", 10);
        
    signal_.camera_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/pointcloud", 10);
        
    signal_.grid = create_publisher<nav_msgs::msg::OccupancyGrid>("/grid", 10);
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void OccupancyGridNode::handleImageScan(sensor_msgs::msg::Image::ConstSharedPtr image){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat image_raw = cv_ptr->image;
    cv::rectangle(image_raw,cv::Point(0,420),cv::Point(848,480),cv::Scalar(0,0,0),-1);
    cv::rectangle(image_raw,cv::Point(0,0),cv::Point(848,80),cv::Scalar(0,0,0),-1);
    cv_bridge::CvImage image_bridge = cv_bridge::CvImage(image->header,sensor_msgs::image_encodings::TYPE_16UC1,image_raw);
    sensor_msgs::msg::Image processed_image;
    image_bridge.toImageMsg(processed_image);
    signal_.processed_image->publish(processed_image);
}

void OccupancyGridNode::handleCameraInfo(sensor_msgs::msg::CameraInfo::SharedPtr info) {
        state_.camera_info = info;
}

void OccupancyGridNode::handleCameraScan(sensor_msgs::msg::Image::ConstSharedPtr image) {
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(); 
    cloud_msg->header = image->header;
    cloud_msg->height = image->height;
    cloud_msg->width = image->width;
    cloud_msg->is_dense = false;
    cloud_msg->is_bigendian = false;
        
    sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(state_.camera_info);
    
    const auto source = cloud_msg->header.frame_id;
    const auto target = "base";
    const geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(target, source, rclcpp::Time(0));
    
    state_.cloud_msg_output = cloud_msg;
    
    convertDepth<uint16_t>(image, cloud_msg, state_.cloud_msg_output, model, transform);
    
    if(params_.enable_camera_cloud){
        signal_.camera_cloud->publish(*state_.cloud_msg_output);
    }
}




void OccupancyGridNode::handleLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
    nav_msgs::msg::OccupancyGrid grid;

    const auto [resolution, radius, enable_lidar_grid, enable_camera_grid, enable_camera_cloud] = params_;
    const auto cell_radius = ceil<int>(radius / resolution);
    
    const auto cell_num = 2 * cell_radius + 1;
    grid.header = scan->header;
    grid.info.resolution = resolution;
    grid.info.width = cell_num;
    grid.info.height = cell_num;

    grid.info.origin.position.x = -radius;
    grid.info.origin.position.y = -radius;
    grid.info.origin.position.z = 0;

    grid.info.origin.orientation.x = 0;
    grid.info.origin.orientation.y = 0;
    grid.info.origin.orientation.z = 0;
    grid.info.origin.orientation.w = 1;
    grid.data.resize(cell_num * cell_num);

    if (params_.enable_lidar_grid) {

        for (size_t k = 0; k < scan->ranges.size(); ++k) {
            const double range = scan->ranges[k];

            const bool valid_range = (scan->range_min <= range) && (range <= scan->range_max);
            if (!valid_range) {
                continue;
            }

            const double angle = scan->angle_min + k * scan->angle_increment;

            const double x = range * std::cos(angle) + radius;
            const double y = range * std::sin(angle) + radius;

            const auto i = clamp(static_cast<int>(x / resolution), 0, cell_num - 1);
            const auto j = clamp(static_cast<int>(y / resolution), 0, cell_num - 1);

            grid.data[i + j * cell_num] = 100;
        }
    }
    
    if(params_.enable_camera_grid) {    
        sensor_msgs::PointCloud2Iterator<float> iter_x(*state_.cloud_msg_output, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*state_.cloud_msg_output, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*state_.cloud_msg_output, "z");        
        for ( ; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            double hmax = 0.01;     //should change hmax/hmin if we need another vertical view 
            double hmin = -0.05;
            if((*iter_z) > hmin && (*iter_z < hmax)) {    
                double y = *iter_y + radius;
                double x = *iter_x + radius;
                const auto i = clamp(static_cast<int>(y / resolution), 0, cell_num - 1); //x
                const auto j = clamp(static_cast<int>(x / resolution), 0, cell_num - 1); //y        
                if(i < cell_num && j < cell_num) {                       
                    grid.data[ j + i * grid.info.width] = 100;
                }
            } 
        }
    }
    signal_.grid->publish(grid);   
}

}  // namespace truck::occupancy_grid