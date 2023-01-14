#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/impl/utils.h>

#include "include/a_star.h"
#include "include/global_variables.h"

using namespace std;

class Graph : public rclcpp::Node {
    private:
        rclcpp::TimerBase::SharedPtr timer_1_, timer_2_;
        
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_car_odom_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_oc_grid_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_end_point_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_dt_grid_float_;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_graph_vertices_base_, publisher_graph_vertices_accent_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_graph_optimal_path_;

        nav_msgs::msg::OccupancyGrid oc_grid_;
        nav_msgs::msg::Odometry car_odom_;
        geometry_msgs::msg::PointStamped end_point_;
        std_msgs::msg::Float32MultiArray dt_grid_values_arr_;

        void topic_callback_car_odometry(const nav_msgs::msg::Odometry::SharedPtr car_odom) {
            car_odom_ = *car_odom;
            // RCLCPP_INFO(this->get_logger(), "Subscription to topic /car/odometry ...");
        }

        void topic_callback_map_occupancy_grid(const nav_msgs::msg::OccupancyGrid::SharedPtr oc_grid) {
            oc_grid_ = *oc_grid;
            // RCLCPP_INFO(this->get_logger(), "Subscription to topic /map/occupancy_grid ...");
        }

        void topic_callback_clicked_point(const geometry_msgs::msg::PointStamped::SharedPtr end_point) {
            end_point_ = *end_point;
            RCLCPP_INFO(this->get_logger(), "Subscription to topic /clicked_point ...");
        }

        void topic_callback_dt_grid_float(const std_msgs::msg::Float32MultiArray::SharedPtr dt_grid_float_values) {
            dt_grid_values_arr_ = *dt_grid_float_values;
            // RCLCPP_INFO(this->get_logger(), "Subscription to topic /map/distance_transform_grid/shared ...");
        }

        void timer_callback_graph_vertices() {
            sensor_msgs::msg::PointCloud2 pointcloud2_base;
            sensor_msgs::msg::PointCloud2 pointcloud2_accent;

            pcl::PointCloud<pcl::PointXYZRGB> pointcloud_base;
            pcl::PointCloud<pcl::PointXYZRGB> pointcloud_accent;

            for (int i = 0; i < grid().margin_size_cells; i++) {
                for (int j = 0; j < grid().margin_size_cells; j++) {
                    int start_x_index = convert_coord_to_cell_index(car_odom_.pose.pose.position.x);
                    int start_y_index = convert_coord_to_cell_index(car_odom_.pose.pose.position.y);

                    int end_x_index = convert_coord_to_cell_index(end_point_.point.x);
                    int end_y_index = convert_coord_to_cell_index(end_point_.point.y);

                    pcl::PointXYZRGB p;

                    if (((i == start_x_index) && (j == start_y_index)) ||
                        ((i == end_x_index) && (j == end_y_index))) {
                        // draw red vertex (accent)
                        p = pcl::PointXYZRGB(255, 0, 0);
                        p.x = (grid().cell_size_meters() / 2) + (grid().cell_size_meters() * i);
                        p.y = (grid().cell_size_meters() / 2) + (grid().cell_size_meters() * j);
                        p.z = 0.005;

                        pointcloud_accent.points.push_back(p);
                    } else {
                        // draw gray vertex (base)
                        p = pcl::PointXYZRGB(70, 70, 70);
                        p.x = (grid().cell_size_meters() / 2) + (grid().cell_size_meters() * i);
                        p.y = (grid().cell_size_meters() / 2) + (grid().cell_size_meters() * j);
                        p.z = 0.005;

                        pointcloud_base.points.push_back(p);
                    }
                }
            }
                
            pcl::toROSMsg(pointcloud_base, pointcloud2_base);
            pcl::toROSMsg(pointcloud_accent, pointcloud2_accent);

            pointcloud2_base.header.stamp = this->now();
            pointcloud2_base.header.frame_id = "odom_ekf";
            
            pointcloud2_accent.header.stamp = this->now();
            pointcloud2_accent.header.frame_id = "odom_ekf";
            
            publisher_graph_vertices_base_->publish(pointcloud2_base);
            publisher_graph_vertices_accent_->publish(pointcloud2_accent);
            // RCLCPP_INFO(this->get_logger(), "Publishing topic /graph/vertices/base ...");
            // RCLCPP_INFO(this->get_logger(), "Publishing topic /graph/vertices/accent ...");
        }

        void timer_callback_graph_optimal_path() {
            tf2::Quaternion tf2_quat;
            tf2::fromMsg(car_odom_.pose.pose.orientation, tf2_quat);

            double yaw_angle = tf2::impl::getYaw(tf2_quat);

            float start_x = car_odom_.pose.pose.position.x;
            float start_y = car_odom_.pose.pose.position.y;

            Vec start_yaw(
                start_x,
                start_y,
                start_x + 0.1 * cos(yaw_angle),
                start_y + 0.1 * sin(yaw_angle)
            );
            

            int start_x_index = convert_coord_to_cell_index(car_odom_.pose.pose.position.x);
            int start_y_index = convert_coord_to_cell_index(car_odom_.pose.pose.position.y);

            int end_x_index = convert_coord_to_cell_index(end_point_.point.x);
            int end_y_index = convert_coord_to_cell_index(end_point_.point.y);

            nav_msgs::msg::Path optimal_path;
            vector<geometry_msgs::msg::PoseStamped> optimal_points;

            optimal_path.header.frame_id = "odom_ekf";
            optimal_path.header.stamp = this->now();

            // 1 - path found
            // 2 - start or end point is obstacle
            // 3 - path doesn't exist
            int success = 
                a_star(
                    optimal_points,
                    start_x_index, start_y_index,
                    end_x_index, end_y_index,
                    *this,
                    oc_grid_,
                    dt_grid_values_arr_,
                    start_yaw
                );

            if (success == 1) {
                optimal_path.poses = optimal_points;          
                // RCLCPP_INFO(this->get_logger(), "Publishing topic /graph/optimal_path ...");
            } else if (success == 2) {
                RCLCPP_INFO(this->get_logger(), "WARNING: start or end point is an obstacle");
            } else if (success == 3) {
                RCLCPP_INFO(this->get_logger(), "WARNING: path doesn't exist");
            }

            publisher_graph_optimal_path_->publish(optimal_path);
        }

    public:
        Graph() : Node("graph_visualizer") {
            const auto qos = static_cast<rmw_qos_reliability_policy_t>(
                this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

            subscription_car_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/ekf/odometry/filtered",
                rclcpp::QoS(1).reliability(qos),
                bind(&Graph::topic_callback_car_odometry, this, placeholders::_1)
            );

            subscription_end_point_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
                "/clicked_point",
                rclcpp::QoS(1).reliability(qos),
                bind(&Graph::topic_callback_clicked_point, this, placeholders::_1)
            );

            subscription_oc_grid_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map/occupancy_grid",
                10,
                bind(&Graph::topic_callback_map_occupancy_grid, this, placeholders::_1)
            );

            subscription_dt_grid_float_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "/map/distance_transform_grid/shared",
                10,
                bind(&Graph::topic_callback_dt_grid_float, this, placeholders::_1)
            );

            publisher_graph_vertices_base_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/graph/vertices/base", 10);
            publisher_graph_vertices_accent_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/graph/vertices/accent", 10);
            publisher_graph_optimal_path_ = this->create_publisher<nav_msgs::msg::Path>("/motion/path/custom", 10);

            timer_1_ = this->create_wall_timer(250ms, bind(&Graph::timer_callback_graph_vertices, this));
            timer_2_ = this->create_wall_timer(250ms, bind(&Graph::timer_callback_graph_optimal_path, this));
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<Graph>());
    rclcpp::shutdown();
    return 0;
}