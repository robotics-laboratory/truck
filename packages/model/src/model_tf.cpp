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
        const auto tf_path = this->declare_parameter<std::string>("tf_path", "tf_static.yaml");
        const auto static_qos = rclcpp::QoS(1).transient_local();

        static_tf_signal_ = Node::create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", static_qos);

        auto static_tf = loadTf(tf_path);

        RCLCPP_INFO(
            this->get_logger(),
            "Load %zu static transforms from %s",
            static_tf.transforms.size(),
            tf_path.c_str());

        const auto stamp = now();
        for (auto&& tf : static_tf.transforms) {
            tf.header.stamp = stamp;
        }

        static_tf_signal_->publish(static_tf);
    }

  private:

    // output
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr static_tf_signal_ = nullptr;
};

}  // namespace truck::model

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<truck::model::ModelTfNode>());
    rclcpp::shutdown();
    return 0;
}