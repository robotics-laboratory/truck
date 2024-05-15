#include "model/model.h"

#include <chrono>

namespace truck::model {

using namespace std::literals;

class ModelTfNode : public rclcpp::Node {
  public:
    ModelTfNode() : Node("ModelTfNode") {
        const auto model_path = this->declare_parameter<std::string>("model_path", "model.yaml");
        const auto static_qos = rclcpp::QoS(1).transient_local();

        model_ = std::make_unique<model::Model>(model::load(this->get_logger(), model_path));

        static_tf_signal_ =
            Node::create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", static_qos);

        auto static_tf = model_->getTfStaticMsg();

        RCLCPP_INFO(
            this->get_logger(),
            "Load %zu static transforms from %s",
            static_tf.transforms.size(),
            model_path.c_str());

        const auto stamp = now();
        for (auto&& tf : static_tf.transforms) {
            tf.header.stamp = stamp;
        }

        static_tf_signal_->publish(static_tf);
    }

  private:
    std::unique_ptr<model::Model> model_ = nullptr;

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
