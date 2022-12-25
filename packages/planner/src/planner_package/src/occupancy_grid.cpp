#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;
using namespace std::chrono_literals;

class OccupancyGridPublisher : public rclcpp::Node {
    private:
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;

        nav_msgs::msg::OccupancyGrid oc_grid_;

        vector<int> oc_grid_mat_to_colors(const Mat &oc_grid_mat, int grid_size) {
            vector<int> oc_grid_colors;

            for (int i = 0; i < grid_size; i++) {
                for (int j = 0; j < grid_size; j++) {
                    if (oc_grid_mat.at<uchar>(grid_size - i - 1, j) == 0) {
                        // cell type: obstacle
                        // color: black [value 99]
                        oc_grid_colors.push_back(99);
                    } else {
                        // cell type: free
                        // color: white [value 0]
                        oc_grid_colors.push_back(0);  
                    }
                }
            }

            return oc_grid_colors;
        }

        void create_occupancy_grid(const vector<int>& colors, int grid_size) {
            oc_grid_.header.frame_id = "/odom";
            oc_grid_.header.stamp = this->now();

            // cell size is 5 cm.
            oc_grid_.info.resolution = 0.05;
            oc_grid_.info.height = grid_size;
            oc_grid_.info.width = grid_size;
            oc_grid_.info.origin.position.x = oc_grid_.info.origin.position.y = oc_grid_.info.origin.position.z = 0;
            
            for (int el : colors) {
                oc_grid_.data.push_back(el);
            }
        }

        void load_occupancy_grid() {
            Mat oc_grid_mat = imread("/truck/packages/planner/src/planner_package/src/maps/map_2", IMREAD_GRAYSCALE);
            int grid_size = oc_grid_mat.size().height;

            vector<int> oc_grid_colors = oc_grid_mat_to_colors(oc_grid_mat, grid_size);
            create_occupancy_grid(oc_grid_colors, grid_size);
        }

        void timer_callback() {
            publisher_->publish(oc_grid_);
            RCLCPP_INFO(this->get_logger(), "Publishing topic /map/occupancy_grid");
        }

    public:
        OccupancyGridPublisher(): Node("occupancy_grid_publisher") {
            load_occupancy_grid();
            publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/occupancy_grid", 10);
            timer_ = this->create_wall_timer(1s, bind(&OccupancyGridPublisher::timer_callback, this));
        }
};


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<OccupancyGridPublisher>());
    rclcpp::shutdown();
    return 0;
}