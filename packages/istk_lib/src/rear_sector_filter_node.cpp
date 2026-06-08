#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

namespace
{

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

struct FilterConfig
{
  double center_angle_deg = 180.0;
  double width_deg = 60.0;
  double range_min = 0.05;
  double range_max = 2.0;
  bool invert = false;
};

class RearSectorFilterNode : public rclcpp::Node
{
public:
  RearSectorFilterNode() : rclcpp::Node("rear_sector_filter")
  {
    declare_parameter<std::string>("input_scan", "/scan");
    declare_parameter<std::string>("output_scan", "/scan_filtered");
    declare_parameter<std::string>("config_path", "");
    declare_parameter<std::string>("control_prefix", "/filter");
    declare_parameter<double>("center_angle_deg", 180.0);
    declare_parameter<double>("width_deg", 60.0);
    declare_parameter<double>("range_min", 0.05);
    declare_parameter<double>("range_max", 2.0);
    declare_parameter<bool>("invert", false);

    input_scan_ = get_parameter("input_scan").as_string();
    output_scan_ = get_parameter("output_scan").as_string();
    config_path_ = get_parameter("config_path").as_string();
    control_prefix_ = get_parameter("control_prefix").as_string();
    if (!control_prefix_.empty() && control_prefix_.back() == '/') {
      control_prefix_.pop_back();
    }

    current_config_ = loadInitialConfig();
    startup_config_ = current_config_;

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_scan_, rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<std_msgs::msg::String>(control_prefix_ + "/status", 10);
    current_pub_ =
      create_publisher<geometry_msgs::msg::Vector3>(control_prefix_ + "/current", 10);

    applyConfig(current_config_);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_scan_, rclcpp::SensorDataQoS(),
      std::bind(&RearSectorFilterNode::onScan, this, std::placeholders::_1));

    adjust_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      control_prefix_ + "/adjust", 10,
      std::bind(&RearSectorFilterNode::onAdjust, this, std::placeholders::_1));
    set_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
      control_prefix_ + "/set", 10,
      std::bind(&RearSectorFilterNode::onSet, this, std::placeholders::_1));
    save_sub_ = create_subscription<std_msgs::msg::Empty>(
      control_prefix_ + "/save", 10,
      std::bind(&RearSectorFilterNode::onSave, this, std::placeholders::_1));
    reload_sub_ = create_subscription<std_msgs::msg::Empty>(
      control_prefix_ + "/reload", 10,
      std::bind(&RearSectorFilterNode::onReload, this, std::placeholders::_1));
    reset_sub_ = create_subscription<std_msgs::msg::Empty>(
      control_prefix_ + "/reset", 10,
      std::bind(&RearSectorFilterNode::onReset, this, std::placeholders::_1));

    publishStatus(
      "RearSectorFilter started: " + input_scan_ + " -> " + output_scan_ +
      ", center=" + toString(current_config_.center_angle_deg) + " deg" +
      ", width=" + toString(current_config_.width_deg) + " deg" +
      ", range_max=" + toString(current_config_.range_max) + " m" +
      ", invert=" + std::string(current_config_.invert ? "true" : "false"));
  }

private:
  FilterConfig loadInitialConfig() const
  {
    FilterConfig config;
    config.center_angle_deg = get_parameter("center_angle_deg").as_double();
    config.width_deg = get_parameter("width_deg").as_double();
    config.range_min = get_parameter("range_min").as_double();
    config.range_max = get_parameter("range_max").as_double();
    config.invert = get_parameter("invert").as_bool();

    if (config_path_.empty()) {
      return config;
    }

    std::ifstream stream(config_path_);
    if (!stream.good()) {
      return config;
    }

    const YAML::Node root = YAML::LoadFile(config_path_);
    const YAML::Node node = root["rear_sector_filter"];
    if (!node) {
      return config;
    }

    if (node["center_angle_deg"]) {
      config.center_angle_deg = node["center_angle_deg"].as<double>();
    }
    if (node["width_deg"]) {
      config.width_deg = node["width_deg"].as<double>();
    }
    if (node["range_min"]) {
      config.range_min = node["range_min"].as<double>();
    }
    if (node["range_max"]) {
      config.range_max = node["range_max"].as<double>();
    }
    if (node["invert"]) {
      config.invert = node["invert"].as<bool>();
    }

    return config;
  }

  void saveConfig() const
  {
    if (config_path_.empty()) {
      return;
    }

    YAML::Node root;
    root["rear_sector_filter"]["center_angle_deg"] = current_config_.center_angle_deg;
    root["rear_sector_filter"]["width_deg"] = current_config_.width_deg;
    root["rear_sector_filter"]["range_min"] = current_config_.range_min;
    root["rear_sector_filter"]["range_max"] = current_config_.range_max;
    root["rear_sector_filter"]["invert"] = current_config_.invert;

    std::ofstream out(config_path_);
    out << root;
  }

  void applyConfig(const FilterConfig & config)
  {
    current_config_ = config;
    center_angle_rad_ = current_config_.center_angle_deg * M_PI / 180.0;
    half_width_rad_ = current_config_.width_deg * M_PI / 360.0;
    publishCurrent();
  }

  void publishCurrent() const
  {
    if (!current_pub_) {
      return;
    }
    geometry_msgs::msg::Vector3 msg;
    msg.x = current_config_.center_angle_deg;
    msg.y = current_config_.width_deg;
    msg.z = current_config_.range_max;
    current_pub_->publish(msg);
  }

  void publishStatus(const std::string & text) const
  {
    if (!status_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
    publishCurrent();
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }

  void onAdjust(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    FilterConfig config = current_config_;
    config.center_angle_deg += msg->x;
    config.width_deg = std::max(1.0, config.width_deg + msg->y);
    config.range_max = std::max(config.range_min, config.range_max + msg->z);
    applyConfig(config);
    publishStatus(
      "Adjusted sector: d_center=" + toString(msg->x) + " deg, d_width=" + toString(msg->y) +
      " deg, d_range=" + toString(msg->z) + " m");
  }

  void onSet(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    FilterConfig config = current_config_;
    config.center_angle_deg = msg->x;
    config.width_deg = std::max(1.0, msg->y);
    config.range_max = std::max(config.range_min, msg->z);
    applyConfig(config);
    publishStatus(
      "Set sector: center=" + toString(msg->x) + " deg, width=" + toString(msg->y) +
      " deg, range=" + toString(msg->z) + " m");
  }

  void onSave(const std_msgs::msg::Empty::SharedPtr)
  {
    saveConfig();
    publishStatus("Saved " + config_path_);
  }

  void onReload(const std_msgs::msg::Empty::SharedPtr)
  {
    applyConfig(loadInitialConfig());
    publishStatus("Reloaded " + config_path_);
  }

  void onReset(const std_msgs::msg::Empty::SharedPtr)
  {
    applyConfig(startup_config_);
    publishStatus("Reset to startup sector");
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto out = *msg;
    double angle = msg->angle_min;

    for (std::size_t i = 0; i < out.ranges.size(); ++i) {
      const float range = out.ranges[i];
      const bool inside_sector = std::abs(normalizeAngle(angle - center_angle_rad_)) <= half_width_rad_;
      const bool inside_range =
        std::isfinite(range) && range >= current_config_.range_min && range <= current_config_.range_max;

      bool should_remove = inside_sector;
      if (current_config_.invert) {
        should_remove = !should_remove;
      }

      if (inside_range && should_remove) {
        out.ranges[i] = std::numeric_limits<float>::infinity();
        if (!out.intensities.empty() && i < out.intensities.size()) {
          out.intensities[i] = 0.0f;
        }
      }

      angle += msg->angle_increment;
    }

    scan_pub_->publish(out);
  }

  static std::string toString(double value)
  {
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(2);
    stream << value;
    return stream.str();
  }

  std::string input_scan_;
  std::string output_scan_;
  std::string config_path_;
  std::string control_prefix_;

  FilterConfig current_config_;
  FilterConfig startup_config_;
  double center_angle_rad_ = M_PI;
  double half_width_rad_ = M_PI / 6.0;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr current_pub_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr adjust_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr set_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr save_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reload_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RearSectorFilterNode>());
  rclcpp::shutdown();
  return 0;
}
