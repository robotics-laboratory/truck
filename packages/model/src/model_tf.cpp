#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <memory>

namespace truck::model {

using namespace std::literals;

geometry_msgs::msg::Vector3 toVector3(const YAML::Node& v) {
    geometry_msgs::msg::Vector3 msg;

    msg.x = v['x'].as<double>();
    msg.y = v['y'].as<double>();
    msg.z = v['z'].as<double>();

    return msg;
}

geometry_msgs::msg::Quaternion toQuaternion(const YAML::Node& q) {
    geometry_msgs::msg::Quaternion msg;

    msg.x = q['x'].as<double>();
    msg.y = q['y'].as<double>();
    msg.z = q['z'].as<double>();
    msg.w = q['w'].as<double>();

    return msg;
}

tf2_msgs::msg::TFMessage loadTf(const std::string& path) {
    tf2_msgs::msg::TFMessage result;

    const auto node = YAML::LoadFile(path);
    for (const auto& tf : node) {
        geometry_msgs::msg::TransformStamped msg;

        msg.header.frame_id = tf["frame_id"].as<std::string>();
        msg.child_frame_id = tf["child_frame_id"].as<std::string>();

        msg.transform.translation = toVector3(tf["translation"]);
        msg.transform.rotation = toQuaternion(tf["rotation"]);

        result.transforms.push_back(msg);
    }

    return result;
}

class ModelTfNode : public rclcpp::Node {
  public:
    ModelTfNode() : Node("ModelTfNode") {
        const auto period =
            std::chrono::milliseconds(this->declare_parameter<long int>("period", 1000));

        const auto tf_path = this->declare_parameter<std::string>("tf_path", "tf_static.yaml");

        timer_ = this->create_wall_timer(period, std::bind(&ModelTfNode::publish, this));
        tf_signal_ = Node::create_publisher<tf2_msgs::msg::TFMessage>("/tf", 1);

        tf_ = loadTf(tf_path);

        RCLCPP_INFO(
            this->get_logger(),
            "load %zu transforms  from %s",
            tf_.transforms.size(),
            tf_path.c_str());
    }

  private:
    void publish() {
        const auto stamp = now();

        auto msg = tf_;
        for (auto&& tf: msg.transforms) {
            tf.header.stamp = stamp;
        }

        tf_signal_->publish(msg);
    }

    tf2_msgs::msg::TFMessage tf_;

    // output
    rclcpp::TimerBase::SharedPtr timer_ = nullptr;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_signal_ = nullptr;
};

}  // namespace truck::model

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck::model::ModelTfNode>());
    rclcpp::shutdown();
    return 0;
}