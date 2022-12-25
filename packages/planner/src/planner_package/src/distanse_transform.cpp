#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float32.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>

using namespace cv;
using namespace std;
using namespace std::chrono_literals;

class StaticCollisionChecker : public rclcpp::Node {
    private:    
        int grid_size_ = 200;
        float darkest_free_cell_color_ = 90.0f;
        // 1 cell size is 5 cm., so 2 m. (200 cm) equals to 40 cells
        float max_cubes_to_highlight_color_ = 40.0f;

        rclcpp::TimerBase::SharedPtr timer_1_;
        rclcpp::TimerBase::SharedPtr timer_2_;

        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_oc_grid_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_car_odom_;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_dist_trans_grid_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_dist_nearest_obstacle_;

        nav_msgs::msg::OccupancyGrid oc_grid_, dist_trans_grid_;
        nav_msgs::msg::Odometry car_odom_;

        Mat oc_grid_mat_binary_, dist_trans_grid_mat_values_;

        void topic_callback_map_occupancy_grid(const nav_msgs::msg::OccupancyGrid::SharedPtr oc_grid) {
            RCLCPP_INFO(this->get_logger(), "Subscription to topic /map/occupancy_grid");
            oc_grid_ = *oc_grid;
        }

        void topic_callback_car_odometry(const nav_msgs::msg::Odometry::SharedPtr car_odom) {
            RCLCPP_INFO(this->get_logger(), "Subscription to topic /car/odometry");
            car_odom_ = *car_odom;
        }

        void create_oc_grid_mat() {
            oc_grid_mat_binary_ = Mat(grid_size_, grid_size_, CV_32F);

            for (int i = 0; i < grid_size_; i++) {
                for (int j = 0; j < grid_size_; j++) {
                    auto oc_grid_cell = int(oc_grid_.data.at(i * grid_size_ + j));

                    if (oc_grid_cell == 0) {
                        // cell type: free
                        oc_grid_mat_binary_.at<float>(i, j) = 1;
                    } else if (oc_grid_cell == 99) {
                        // cell type: obstacle
                        oc_grid_mat_binary_.at<float>(i, j) = 0;
                    }
                }
            }

            oc_grid_mat_binary_.convertTo(oc_grid_mat_binary_, CV_8UC1);
        }

        void initialize_dist_trans_grid() {
            dist_trans_grid_.header.frame_id = "/odom";
            dist_trans_grid_.header.stamp = this->now();            

            // cell size 5 cm.
            dist_trans_grid_.info.resolution = 0.05;
            dist_trans_grid_.info.height = grid_size_;
            dist_trans_grid_.info.width = grid_size_;

            dist_trans_grid_.info.origin.position.x = 0;
            dist_trans_grid_.info.origin.position.y = 0;
            dist_trans_grid_.info.origin.position.z = 0;

            dist_trans_grid_.data.clear();
        }

        void assign_colors_from_dist_trans_grid_mat() {
            for (int i = 0; i < grid_size_; i++) {
                for (int j = 0; j < grid_size_; j++) {
                    if (dist_trans_grid_mat_values_.at<float>(i, j) == 0.0f) {
                        // cell type: obstacle
                        // cell color: 99 (black)
                        dist_trans_grid_.data.push_back(99);
                    } else if (dist_trans_grid_mat_values_.at<float>(i, j) > max_cubes_to_highlight_color_) {
                        // cell type: free (obstacle is further than 2 m.)
                        // cell color: 0 (white)
                        dist_trans_grid_.data.push_back(0);
                    } else {
                        // cell type: free (obstacle is in segment (0 m., 2 m.])
                        // cell color: in segment [0, 90)
                        auto cell_color =
                            (max_cubes_to_highlight_color_ - dist_trans_grid_mat_values_.at<float>(i, j)) *
                            (darkest_free_cell_color_ / max_cubes_to_highlight_color_);

                        dist_trans_grid_.data.push_back(cell_color);
                    }
                }
            }
        }

        void timer_callback_map_distance_transform_grid() {
            create_oc_grid_mat();
            distanceTransform(oc_grid_mat_binary_, dist_trans_grid_mat_values_, DIST_L2, DIST_MASK_PRECISE);
            initialize_dist_trans_grid();
            assign_colors_from_dist_trans_grid_mat(); 
            publisher_dist_trans_grid_->publish(dist_trans_grid_);
            RCLCPP_INFO(this->get_logger(), "Publishing topic /map/distance_transform_grid");
        }

        float dist_to_near_obstacle_in_cubes(float x, float y) {
            int i = int(y * 100 / 5);
            int j = int(x * 100 / 5);

            return dist_trans_grid_mat_values_.at<float>(i, j);
        }

        void timer_callback_dist_nearest_obstacle() {
            /// TODO: Integrate project and create .yaml config
            // int k = model_.truckShape().circles_count_approx_shape;
            // float l = model_.truckShape().length - model_.truckShape().width;
            /// int k = 3;
            /// float l = 0.505 - 0.313;

            std_msgs::msg::Float32 dist_nearest_obstacle;
            dist_nearest_obstacle.data =
                dist_to_near_obstacle_in_cubes(car_odom_.pose.pose.position.x, car_odom_.pose.pose.position.y);

            publisher_dist_nearest_obstacle_->publish(dist_nearest_obstacle);
            RCLCPP_INFO(this->get_logger(), "Publishing topic /dist_nearest_obstcle");
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

            publisher_dist_trans_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/distance_transform_grid", 10);

            publisher_dist_nearest_obstacle_ = this->create_publisher<std_msgs::msg::Float32>("/dist_nearest_obstcle", 10);

            timer_1_ = this->create_wall_timer(1s, bind(&StaticCollisionChecker::timer_callback_map_distance_transform_grid, this));
            timer_2_ = this->create_wall_timer(1s, bind(&StaticCollisionChecker::timer_callback_dist_nearest_obstacle, this));
        }
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<StaticCollisionChecker>());
    rclcpp::shutdown();
    return 0;
}