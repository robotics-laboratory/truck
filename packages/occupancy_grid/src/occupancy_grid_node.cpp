#include "occupancy_grid/occupancy_grid_node.h"

#include "common/math.h"

#include <cstdint>
#include <functional>

#include <limits>
#include <cmath>
#include <vector>
#include <math.h>
#include <chrono>

#include <image_transport/image_transport.hpp>

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
sensor_msgs::msg::PointCloud2::SharedPtr makeCloud(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const image_geometry::PinholeCameraModel & model,
    const geometry_msgs::msg::TransformStamped & from_camera_to_lidar_tf_msg,
    double range_max = 0.0
    )
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
sensor_msgs::msg::PointCloud2::SharedPtr initial_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(); 
initial_cloud_msg->header = from_camera_to_lidar_tf_msg.header;
initial_cloud_msg->height = depth_msg->height;
initial_cloud_msg->width = depth_msg->width;
initial_cloud_msg->is_dense = false;
initial_cloud_msg->is_bigendian = false;
sensor_msgs::PointCloud2Modifier pcd_modifier(*initial_cloud_msg);
pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

sensor_msgs::PointCloud2Iterator<float> iter_x(*initial_cloud_msg, "x");
sensor_msgs::PointCloud2Iterator<float> iter_y(*initial_cloud_msg, "y");
sensor_msgs::PointCloud2Iterator<float> iter_z(*initial_cloud_msg, "z");

tf2::Stamped<tf2::Transform> tf;
tf2::fromMsg(from_camera_to_lidar_tf_msg,tf); 
auto rotation = tf.getRotation();
auto translation = tf.getOrigin();
tf2::Transform transforamtion(rotation, translation);
tf2::Vector3 transformed_point;

const T * depth_row = reinterpret_cast<const T *>(&depth_msg->data[0]);
int row_step = depth_msg->step / sizeof(T);
for (int v = 0; v < static_cast<int>(initial_cloud_msg->height); ++v, depth_row += row_step) {
    for (int u = 0; u < static_cast<int>(initial_cloud_msg->width); ++u, ++iter_x, ++iter_y, ++iter_z) {
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
    
    auto x = ((u - center_x) * depth * constant_x);
    auto y = ((v - center_y) * depth * constant_y);
    auto z = (DepthTraits<T>::toMeters(depth));
    tf2::Vector3 depth_vector(x,y,z);
    transformed_point =  transforamtion * depth_vector;
    *iter_x = transformed_point.x();
    *iter_y = transformed_point.y();
    *iter_z = transformed_point.z();
    }    
}
return initial_cloud_msg;
}
}

namespace truck::occupancy_grid {

OccupancyGridNode::OccupancyGridNode() : Node("occupancy_grid") {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    RCLCPP_INFO(this->get_logger(), "qos %d", qos);

    params_ = {this->declare_parameter("resolution", 0.1), this->declare_parameter("radius", 20.0),
     this->declare_parameter("enable_lidar_grid", false),
     this->declare_parameter("enable_camera_grid", false),
     this->declare_parameter("enable_camera_cloud", false),
     this->declare_parameter("camera_view_hmin", -0.05),
     this->declare_parameter("camera_view_hmax", 0.01)};

    RCLCPP_INFO(this->get_logger(), "resolution: %.2fm", params_.resolution);
    RCLCPP_INFO(this->get_logger(), "radius: %.2fm", params_.radius);
    RCLCPP_INFO(this->get_logger(), "enable_camera_cloud: %d", params_.enable_camera_cloud);
    RCLCPP_INFO(this->get_logger(), "enable_lidar_grid: %d", params_.enable_lidar_grid);
    RCLCPP_INFO(this->get_logger(), "enable_camera_grid: %d", params_.enable_camera_grid);
    RCLCPP_INFO(this->get_logger(), "camera_view_hmin: %.2fm", params_.camera_view_hmin);
    RCLCPP_INFO(this->get_logger(), "camera_view_hmax: %.2fm", params_.camera_view_hmax);


    slot_.camera_info = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/depth/camera_info", 10,
        std::bind(&OccupancyGridNode::handleCameraInfo, this, std::placeholders::_1));
    
    slot_.camera = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth/image_rect_raw", rclcpp::QoS(1).reliability(qos), 
        std::bind(&OccupancyGridNode::handleCameraDepth, this, std::placeholders::_1));
    
    
    slot_.lidar = create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar/scan",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&OccupancyGridNode::handleLaserScan, this, std::placeholders::_1));
                
    signal_.camera_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/pointcloud", 10);
        
    signal_.grid = create_publisher<nav_msgs::msg::OccupancyGrid>("/grid", 10);
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void OccupancyGridNode::handleCameraInfo(sensor_msgs::msg::CameraInfo::SharedPtr info) {
        state_.camera_info = info;
}

void OccupancyGridNode::handleCameraDepth(sensor_msgs::msg::Image::ConstSharedPtr image) {

    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(state_.camera_info);
    
    const auto source = image->header.frame_id;
    const auto target = "base";
    geometry_msgs::msg::TransformStamped transform;
    
    try {
        transform = tf_buffer_->lookupTransform(target, source, rclcpp::Time(0));
    } catch (const tf2::TransformException& ex) {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform. %s", ex.what());
        return;
    }

    state_.cloud_point = makeCloud<uint16_t>(image, model, transform);
    
    if(params_.enable_camera_cloud) {
        signal_.camera_cloud->publish(*state_.cloud_point);
    }
}

void OccupancyGridNode::handleLaserScan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan){
    state_.lidar_point = scan;
    publishOccupancyGrid(); 
}

void OccupancyGridNode::publishOccupancyGrid() {
    nav_msgs::msg::OccupancyGrid grid;

    const auto [resolution, radius, enable_lidar_grid,
     enable_camera_grid, enable_camera_cloud,camera_view_hmin,camera_view_hmax] = params_;
    const auto cell_radius = ceil<int>(radius / resolution);
    const auto cell_num = 2 * cell_radius + 1;
    
    grid.header = state_.lidar_point->header;
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
        for (size_t k = 0; k < state_.lidar_point->ranges.size(); ++k) {
            const double range = state_.lidar_point->ranges[k];

            const bool valid_range = (state_.lidar_point->range_min <= range) && (range <= state_.lidar_point->range_max);
            if (!valid_range) {
                continue;
            }

            const double angle = state_.lidar_point->angle_min + k * state_.lidar_point->angle_increment;

            const double x = range * std::cos(angle) + radius;
            const double y = range * std::sin(angle) + radius;

            const auto i = static_cast<int>(x / resolution);
            if (i < 0 || i >=  cell_num) {
                continue;
            }
            const auto j = static_cast<int>(y / resolution);
            if (j < 0 || j >=  cell_num) {
                continue;
            }
            grid.data[i + j * cell_num] = 100;
        }
    }

    if (params_.enable_camera_grid) {
        sensor_msgs::PointCloud2Iterator<float> iter_x(*state_.cloud_point, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*state_.cloud_point, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*state_.cloud_point, "z");        
        for ( ; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            if((*iter_z) > camera_view_hmin && (*iter_z < camera_view_hmax)) {    
                double y = *iter_y + radius;
                double x = *iter_x + radius;                      
                const auto i = static_cast<int>(x / resolution);
                if (i < 0 || i >=  cell_num) {
                    continue;
                }
                const auto j = static_cast<int>(y / resolution);
                if (j < 0 || j >=  cell_num) {
                    continue;
                }
                grid.data[ i + j * grid.info.width] = 100;
            } 
        }
    }
    signal_.grid->publish(grid);
}

} // namespace truck::occupancy_grid