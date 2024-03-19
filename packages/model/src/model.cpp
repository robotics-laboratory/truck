#include "model/model.h"

#include <cmath>

namespace truck::model {

namespace {

double getBaseToRearRatio(double base_curvature, double base_to_rear) {
    return std::sqrt(1 - squared(base_curvature * base_to_rear));
}

double getRearToBaseRatio(double rear_curvature, double base_to_rear) {
    return std::sqrt(1 + squared(rear_curvature * base_to_rear));
}

double rearToBaseCurvature(double rear_curvature, double base_to_rear) {
    const double ratio = getRearToBaseRatio(rear_curvature, base_to_rear);
    return rear_curvature / ratio;
}

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

    const auto node = YAML::LoadFile(path)["tf_static"];
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

} // namespace

Model::Model(const std::string& config_path) : params_(config_path) {
    cache_.width_half = params_.wheel_base.width / 2;

    {
        const double tan_inner = tan(abs(params_.limits.steering.inner));
        const double tan_outer = tan(abs(params_.limits.steering.outer));

        const double max_abs_rear_curvature = std::min(
            tan_inner / (params_.wheel_base.length - cache_.width_half * tan_inner),
            tan_outer / (params_.wheel_base.length + cache_.width_half * tan_outer));

        cache_.max_abs_curvature =
            std::min(rearToBaseCurvature(max_abs_rear_curvature, params_.wheel_base.base_to_rear), 
            params_.limits.max_abs_curvature);

        const double steering_limit 
            = std::atan2(max_abs_rear_curvature, params_.wheel_base.length);
        cache_.middle_steering_limits = {-steering_limit, steering_limit};

        cache_.base_curvature_limits = {-cache_.max_abs_curvature, cache_.max_abs_curvature};

        const auto clock = std::make_shared<rclcpp::Clock>();
        cache_.tf_static_buffer = std::make_shared<tf2_ros::Buffer>(clock);
        cache_.tf_static_msg = loadTf(config_path);
        for (const auto& transform : cache_.tf_static_msg.transforms) {
            cache_.tf_static_buffer->setTransform(transform, "", true);
        }
    }
}

Twist Model::baseToRearTwist(Twist twist) const {
    const double ratio = getBaseToRearRatio(twist.curvature, params_.wheel_base.base_to_rear);
    return {twist.curvature / ratio, twist.velocity * ratio};
}

Twist Model::rearToBaseTwist(Twist twist) const {
    const double ratio = getRearToBaseRatio(twist.curvature, params_.wheel_base.base_to_rear);
    return {twist.curvature / ratio, twist.velocity * ratio};
}

double Model::baseToRearAcceleration(double acceleration, double base_curvature) const {
    const double ratio = getBaseToRearRatio(base_curvature, params_.wheel_base.base_to_rear);
    return acceleration * ratio;
}

ServoAngles Model::servoHomeAngles() const { return params_.servo_home_angles; }

double Model::baseMaxAbsCurvature() const { return cache_.max_abs_curvature; }

double Model::steeringVelocity() const { return params_.limits.steering_velocity; }

double Model::baseMaxAcceleration() const { return params_.limits.max_acceleration; }

double Model::baseMaxDeceleration() const { return params_.limits.max_deceleration; }

Limits<double> Model::baseVelocityLimits() const { return params_.limits.velocity; }

Limits<double> Model::baseCurvatureLimits() const { return cache_.base_curvature_limits; }

Limits<geom::Angle> Model::leftSteeringLimits() const {
    return {-params_.limits.steering.inner, params_.limits.steering.outer};
}

Limits<geom::Angle> Model::rightSteeringLimits() const {

    return {-params_.limits.steering.outer, params_.limits.steering.inner};
}

Limits<double> Model::middleSteeringLimits() const {
    return cache_.middle_steering_limits;
}

Steering Model::rearCurvatureToSteering(double curvature) const {
    const double first = curvature * params_.wheel_base.length;
    const double second = curvature * cache_.width_half;

    return Steering {
        geom::Angle::fromRadians(std::atan2(first, 1)),
        geom::Angle::fromRadians(std::atan2(first, 1 - second)),
        geom::Angle::fromRadians(std::atan2(first, 1 + second))
    };
}

double Model::middleSteeringToRearCurvature(double steering) const {
    return tan(steering) / params_.wheel_base.length;
}

Steering Model::rearTwistToSteering(Twist twist) const {
    return rearCurvatureToSteering(twist.curvature);
}

WheelVelocity Model::rearTwistToWheelVelocity(Twist twist) const {
    const double ratio = twist.curvature * cache_.width_half;

    return WheelVelocity {
        geom::Angle{(1 - ratio) * twist.velocity / params_.wheel.radius},
        geom::Angle{(1 + ratio) * twist.velocity / params_.wheel.radius}
    };
}

double Model::linearVelocityToMotorRPS(double velocity) const {
    return velocity / params_.wheel.radius / M_PI / params_.gear_ratio / 2;
}

double Model::motorRPStoLinearVelocity(double rps) const {
    return rps * params_.wheel.radius * M_PI * params_.gear_ratio * 2;
}

double Model::gearRatio() const { return params_.gear_ratio; }

const Shape& Model::shape() const { return params_.shape; }

const WheelBase& Model::wheelBase() const { return params_.wheel_base; }

const Wheel& Model::wheel() const { return params_.wheel; }

const Lidar& Model::lidar() const { return params_.lidar; }

tf2_msgs::msg::TFMessage Model::getTfStaticMsg() const { return cache_.tf_static_msg; }

tf2::Transform Model::getLatestTranform(const std::string& source, 
    const std::string& target) const {
    try {
        const auto tf = cache_.tf_static_buffer->lookupTransform(
            target, source, rclcpp::Time(0)).transform;
        tf2::Transform transform;
        tf2::fromMsg(tf, transform);
        return transform;
    } catch (const tf2::TransformException& ex) {
        return tf2::Transform::getIdentity();
    }
}

}  // namespace truck::model
