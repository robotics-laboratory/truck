#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "include/global_variables.h"

using namespace cv;
using namespace std;
using namespace std::chrono_literals;

class OccupancyGridPublisher : public rclcpp::Node {
    private:
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;

        nav_msgs::msg::OccupancyGrid oc_grid_;

        vector<int> oc_grid_mat_to_colors(const Mat &oc_grid_mat) {
            vector<int> oc_grid_colors;

            for (int i = 0; i < grid().margin_size_cells; i++) {
                for (int j = 0; j < grid().margin_size_cells; j++) {
                    auto value = oc_grid_mat.at<uchar>(grid().margin_size_cells - i - 1, j);

                    // black pixel of 1-channel (b&w) image has value 0
                    if (value == 0) {
                        // cell type: obstacle
                        oc_grid_colors.push_back(grid().obstacle_cell_color);
                    } else {
                        // cell type: free
                        oc_grid_colors.push_back(grid().free_cell_color);  
                    }
                }
            }

            return oc_grid_colors;
        }

        void create_occupancy_grid(const vector<int>& colors) {
            oc_grid_.header.frame_id = "odom_ekf";
            oc_grid_.header.stamp = this->now();

            oc_grid_.info.resolution = grid().cell_size_meters();
            oc_grid_.info.height = grid().margin_size_cells;
            oc_grid_.info.width = grid().margin_size_cells;
            
            oc_grid_.info.origin.position.x = 0;
            oc_grid_.info.origin.position.y = 0;
            oc_grid_.info.origin.position.z = 0;
            
            for (int el : colors) {
                oc_grid_.data.push_back(el);
            }
        }

        void load_occupancy_grid() {
            Mat oc_grid_mat = imread("/truck/packages/planner/src/planner_package/src/maps/map_2.jpg", IMREAD_GRAYSCALE);
            vector<int> oc_grid_colors = oc_grid_mat_to_colors(oc_grid_mat);
            create_occupancy_grid(oc_grid_colors);
        }

        void timer_callback() {
            publisher_->publish(oc_grid_);
            // RCLCPP_INFO(this->get_logger(), "Publishing topic /map/occupancy_grid ...");
        }

    public:
        OccupancyGridPublisher(): Node("occupancy_grid_publisher") {
            load_occupancy_grid();
            publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/occupancy_grid", 10);
            timer_ = this->create_wall_timer(250ms, bind(&OccupancyGridPublisher::timer_callback, this));
        }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<OccupancyGridPublisher>());
    rclcpp::shutdown();
    return 0;
}