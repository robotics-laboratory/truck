#include <cmath>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2/impl/utils.h>

#include "include/global_variables.h"

using namespace cv;
using namespace std;
using namespace std::chrono_literals;

class StaticCollisionChecker : public rclcpp::Node {
    private:
        rclcpp::TimerBase::SharedPtr timer_1_;
        rclcpp::TimerBase::SharedPtr timer_2_;

        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_oc_grid_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_car_odom_;

        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_dt_circles_;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_dt_grid_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_dt_nearest_obstacle_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_dt_grid_float_;

        nav_msgs::msg::OccupancyGrid oc_grid_, dt_grid_;

        nav_msgs::msg::Odometry car_odom_;

        Mat oc_grid_mat_binary_, dt_grid_mat_values_;

        std_msgs::msg::Float32MultiArray dt_grid_values_arr_;

        void topic_callback_map_occupancy_grid(const nav_msgs::msg::OccupancyGrid::SharedPtr oc_grid) {
            // RCLCPP_INFO(this->get_logger(), "Subscription to topic /map/occupancy_grid ...");
            oc_grid_ = *oc_grid;
        }

        void topic_callback_car_odometry(const nav_msgs::msg::Odometry::SharedPtr car_odom) {
            // RCLCPP_INFO(this->get_logger(), "Subscription to topic /car/odometry ...");
            car_odom_ = *car_odom;
        }

        void create_oc_grid_mat() {
            oc_grid_mat_binary_ =
                Mat(grid().margin_size_cells, grid().margin_size_cells, CV_32F);

            for (int i = 0; i < grid().margin_size_cells; i++) {
                for (int j = 0; j < grid().margin_size_cells; j++) {
                    auto oc_grid_cell =
                        int(oc_grid_.data.at(i * grid().margin_size_cells + j));

                    if (oc_grid_cell == grid().free_cell_color) {
                        // cell type: free
                        oc_grid_mat_binary_.at<float>(i, j) = 1;
                    } else if (oc_grid_cell == grid().obstacle_cell_color) {
                        // cell type: obstacle
                        oc_grid_mat_binary_.at<float>(i, j) = 0;
                    }
                }
            }

            oc_grid_mat_binary_.convertTo(oc_grid_mat_binary_, CV_8UC1);
        }

        void dublicate_dt_mat_to_1d_vector() {
            for (int i = 0; i < grid().margin_size_cells; i++) {
                for (int j = 0; j < grid().margin_size_cells; j++) {
                    auto dist = dt_grid_mat_values_.at<float>(i, j);
                    dt_grid_values_arr_.data.push_back(dist);
                }
            }
        }

        void initialize_dist_trans_grid() {
            dt_grid_.header.frame_id = "odom_ekf";
            dt_grid_.header.stamp = this->now();            

            dt_grid_.info.resolution = grid().cell_size_meters();
            dt_grid_.info.height = grid().margin_size_cells;
            dt_grid_.info.width = grid().margin_size_cells;

            dt_grid_.info.origin.position.x = 0;
            dt_grid_.info.origin.position.y = 0;
            dt_grid_.info.origin.position.z = 0;

            dt_grid_.data.clear();
        }

        void assign_colors_from_dist_trans_grid_mat() {
            for (int i = 0; i < grid().margin_size_cells; i++) {
                for (int j = 0; j < grid().margin_size_cells; j++) {
                    auto dist = dt_grid_mat_values_.at<float>(i, j);

                    if (dist == 0.0f) {
                        // cell type: obstacle
                        dt_grid_.data.push_back(grid().obstacle_cell_color);
                    } else if (dist > grid().max_dist_of_interest_cells()) {
                        // cell type: free
                        dt_grid_.data.push_back(grid().free_cell_color);
                    } else {
                        // cell type: free
                        auto cell_color =
                            (grid().max_dist_of_interest_cells() - dt_grid_mat_values_.at<float>(i, j)) *
                            (grid().darkest_free_cell_color / grid().max_dist_of_interest_cells());

                        dt_grid_.data.push_back(cell_color);
                    }
                }
            }
        }

        void timer_callback_map_distance_transform_grid() {
            create_oc_grid_mat();
            distanceTransform(oc_grid_mat_binary_, dt_grid_mat_values_, DIST_L2, DIST_MASK_PRECISE);
            dublicate_dt_mat_to_1d_vector();
            initialize_dist_trans_grid();
            assign_colors_from_dist_trans_grid_mat(); 

            publisher_dt_grid_float_->publish(dt_grid_values_arr_);
            publisher_dt_grid_->publish(dt_grid_);
            // RCLCPP_INFO(this->get_logger(), "Publishing topic /map/distance_transform_grid ...");
            // RCLCPP_INFO(this->get_logger(), "Publishing topic /map/distance_transform_grid/shared ...");
        }

        float distance_to_nearest_obstacle(float x, float y) {
            int i = convert_coord_to_cell_index(y);
            int j = convert_coord_to_cell_index(x);
            auto dist = dt_grid_mat_values_.at<float>(i, j);
            auto dist_filtered = (dist * grid().cell_size_meters()) - (truck().width / 2);

            if (dist_filtered < 0) {
                return 0;
            }

            return dist_filtered;
        }

        void timer_callback_dist_nearest_obstacle() {
            vector<float> distances;
            tf2::Quaternion tf2_quat;
            tf2::fromMsg(car_odom_.pose.pose.orientation, tf2_quat);
            
            double yaw = tf2::impl::getYaw(tf2_quat);

            visualization_msgs::msg::Marker circles;
            circles.header.stamp = now();
            circles.header.frame_id = "odom_ekf";
            circles.id = 1;
            circles.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            circles.action = visualization_msgs::msg::Marker::ADD;
            circles.lifetime = rclcpp::Duration::from_seconds(0);

            circles.scale.x = truck().width;
            circles.scale.y = truck().width;
            circles.scale.z = 1e-5;

            circles.color.a = 1.0;
            circles.color.r = 240;
            circles.color.g = 192;
            circles.color.b = 203;

            geometry_msgs::msg::Point p;

            if (truck().circles_count == 1) {
                p.x = car_odom_.pose.pose.position.x;
                p.y = car_odom_.pose.pose.position.y;
                p.z = 1e-5;

                circles.points.push_back(p);
                distances.push_back(distance_to_nearest_obstacle(p.x, p.y));
            } else {
                for (int i = 0; i < truck().circles_count; i++) {
                    p.x = car_odom_.pose.pose.position.x;
                    p.y = car_odom_.pose.pose.position.y;
                    p.z = 1e-5;

                    float offset =
                        -0.5 * (truck().length - truck().width) +
                        i * ((truck().length - truck().width) / (truck().circles_count - 1));

                    p.x += offset * cos(yaw);
                    p.y += offset * sin(yaw);

                    circles.points.push_back(p);
                    distances.push_back(distance_to_nearest_obstacle(p.x, p.y));
                }
            }

            std_msgs::msg::Float32 min_dist;
            min_dist.data = *std::min_element(distances.begin(), distances.end());

            publisher_dt_circles_->publish(circles);
            publisher_dt_nearest_obstacle_->publish(min_dist);
            // RCLCPP_INFO(this->get_logger(), "Publishing topic /car/circles ...");
            // RCLCPP_INFO(this->get_logger(), "Publishing topic /dist_nearest_obstcle ...");
        }

    public:
        StaticCollisionChecker() : Node("distance_transform") {
            const auto qos = static_cast<rmw_qos_reliability_policy_t>(
                this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

            subscription_oc_grid_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map/occupancy_grid",
                10,
                bind(&StaticCollisionChecker::topic_callback_map_occupancy_grid, this, placeholders::_1)
            );
            
            subscription_car_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/ekf/odometry/filtered",
                rclcpp::QoS(1).reliability(qos),
                bind(&StaticCollisionChecker::topic_callback_car_odometry, this, placeholders::_1)
            );

            publisher_dt_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/distance_transform_grid", 10);

            publisher_dt_grid_float_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/map/distance_transform_grid/shared", 10);

            publisher_dt_nearest_obstacle_ = this->create_publisher<std_msgs::msg::Float32>("/dist_nearest_obstcle", 10);

            publisher_dt_circles_ = this->create_publisher<visualization_msgs::msg::Marker>("/car/circles", 10);

            timer_1_ = this->create_wall_timer(250ms, bind(&StaticCollisionChecker::timer_callback_map_distance_transform_grid, this));
            timer_2_ = this->create_wall_timer(250ms, bind(&StaticCollisionChecker::timer_callback_dist_nearest_obstacle, this));
        }
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<StaticCollisionChecker>());
    rclcpp::shutdown();
    return 0;
}