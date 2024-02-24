#include "simulator_2d/simulator_engine.h"

#include "map/map.h"
#include "geom/distance.h"
#include "geom/swap.h"
#include "geom/intersection.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace truck::simulator {

SimulatorEngine::SimulatorEngine(std::unique_ptr<model::Model> model,
    double integration_step, double precision) {
        
    model_ = std::move(model);

    params_.integration_step = integration_step;
    params_.precision = precision;

    cache_.integration_step_2 = integration_step / 2;
    cache_.integration_step_6 = integration_step / 6;
    cache_.inverse_integration_step = 1 / integration_step;
    cache_.inverse_wheelbase_length = 1 / model_->wheelBase().length;
    cache_.base_to_lidar = model_->lidar().from_base;

    const double angle_min_rad = model_->lidar().angle_min.radians();
    const double angle_max_rad = model_->lidar().angle_max.radians();
    const double angle_increment_rad = model_->lidar().angle_increment.radians();

    cache_.lidar_rays_number = (angle_max_rad - angle_min_rad) / angle_increment_rad;
    cache_.lidar_angle_min = geom::AngleVec2(model_->lidar().angle_min);
    cache_.lidar_angle_increment = model_->lidar().angle_increment;

    resetRear();
}

void SimulatorEngine::resetRear(double x, double y, double yaw,
    double steering, double linear_velocity) {

    rear_ax_state_ = (SimulatorEngine::State() 
        << x, y, yaw, steering, linear_velocity)
        .finished();
}

void SimulatorEngine::resetRear() {
    resetRear(-model_->wheelBase().base_to_rear, 0, 0, 0, 0);
}

void SimulatorEngine::resetBase(const geom::Pose& pose,
    double middle_steering, double linear_velocity) {
    
    const auto [rear_x, rear_y] = geom::Vec2{pose.pos.x, pose.pos.y} 
        - model_->wheelBase().base_to_rear * pose.dir;

    const double yaw = pose.dir.angle().radians();

    const double base_curvature = std::tan(middle_steering) * cache_.inverse_wheelbase_length;
    const auto base_twist = model::Twist {base_curvature, linear_velocity};
    const auto rear_twist = model_->baseToRearTwist(base_twist);

    resetRear(rear_x, rear_y, yaw, middle_steering, rear_twist.velocity);
}

void SimulatorEngine::resetMap(const std::string& path) {
    obstacles_.clear();
    const auto map = map::Map::fromGeoJson(path);
    const auto polygons = map.polygons();
    for (const auto &polygon: polygons) {
        auto segments = polygon.segments();
        obstacles_.insert(obstacles_.end(), 
            std::make_move_iterator(segments.begin()), 
            std::make_move_iterator(segments.end()));
    }
}

void SimulatorEngine::eraseMap() {
    obstacles_.clear();
}

geom::Pose SimulatorEngine::getOdomBasePose() const {
    const double x = rear_ax_state_[StateIndex::kX];
    const double y = rear_ax_state_[StateIndex::kY];
    const double yaw = rear_ax_state_[StateIndex::kYaw];

    geom::Pose pose;
    pose.dir = geom::AngleVec2(geom::Angle::fromRadians(yaw));
    pose.pos = geom::Vec2{x, y} + model_->wheelBase().base_to_rear * pose.dir;
    return pose;
}

model::Steering SimulatorEngine::getCurrentSteering(double rear_curvature) const {
    return model_->rearCurvatureToSteering(rear_curvature);
}

model::Steering SimulatorEngine::getTargetSteering() const {
    return model_->rearCurvatureToSteering(control_.curvature);
}

model::Twist SimulatorEngine::rearToOdomBaseTwist(double rear_curvature) const {
    const double linear_velocity = rear_ax_state_[StateIndex::kLinearVelocity];
    const auto twist = model::Twist {
        rear_curvature,
        linear_velocity
    };

    return model_->rearToBaseTwist(twist);
}

geom::Vec2 SimulatorEngine::rearToOdomBaseLinearVelocity(
    truck::geom::AngleVec2 dir, double base_velocity) const {

    return dir * base_velocity;
}

double SimulatorEngine::rearToBaseAngularVelocity(
    double base_velocity, double rear_curvature) const {

    return base_velocity * rear_curvature;
}

namespace {

int softSign(double number, double precision) {
    if (number > precision) {
        return 1;
    }
    
    if (number < -precision) {
        return -1;
    }

    return 0;
}

int mod(int number, int divider) {
    return (number % divider + divider) % divider;
}

geom::Vec2 getLidarOrigin(const geom::Pose& odom_base_pose, const geom::Vec2& from_base) {
    const auto dir_vec = odom_base_pose.dir.vec();
    const auto x = odom_base_pose.pos.x 
        + from_base.x * dir_vec.x - from_base.y * dir_vec.y;
    const auto y = odom_base_pose.pos.y
        + from_base.y * dir_vec.x + from_base.x * dir_vec.y;
    return geom::Vec2(x, y);
}

geom::Angle getOrientedAngle(const geom::Vec2& origin_to_point, 
    const geom::Vec2& dir) {

    auto origin_to_point_angle = geom::angleBetween(dir, origin_to_point);
    return origin_to_point_angle._0_2PI();
}

double ceil_with_precision(double number, double precision) {
    return std::ceil(number / precision) * precision;
}

geom::AngleVec2 getRayDir(const geom::AngleVec2& zero_dir, 
    const geom::Angle increment, const int index) {
    
    const auto mult_increment = geom::AngleVec2(index * increment);
    return zero_dir + mult_increment;
}

float getIntersectionDistance(const geom::Ray& ray, 
    const geom::Segment& segment, double precision) {

    const auto intersection = geom::intersect(ray, segment, precision);
    if (!intersection) {
        return std::numeric_limits<float>::max();
    }

    const auto distance = geom::distance(*intersection, ray.origin);
    return static_cast<float>(distance);
}

} // namespace

std::vector<float> SimulatorEngine::getLidarRanges(const geom::Pose& odom_base_pose) const {
    std::vector<float> ranges(cache_.lidar_rays_number, std::numeric_limits<float>::max());
    
    const auto origin = getLidarOrigin(odom_base_pose, cache_.base_to_lidar);
    const auto dir = (odom_base_pose.dir + cache_.lidar_angle_min);
    const auto dir_vector = dir.vec();
    const auto increment_rad = cache_.lidar_angle_increment.radians();

    for (const auto& segment : obstacles_) {
        const auto origin_begin = segment.begin - origin;
        const auto origin_end = segment.end - origin;

        auto begin_oriented_angle = getOrientedAngle(origin_begin, dir_vector);
        auto end_oriented_angle = getOrientedAngle(origin_end, dir_vector);

        const int sign = geom::angleBetween(origin_begin, origin_end).radians() > 0
            ? 1 : -1;
        int begin_index, end_index;

        if (sign > 0) {
            begin_index = ceil_with_precision(
                begin_oriented_angle.radians() / increment_rad, 
                params_.precision);
            end_index = end_oriented_angle.radians() / increment_rad;
        } else {
            begin_index = begin_oriented_angle.radians() / increment_rad;
            end_index = ceil_with_precision(
                end_oriented_angle.radians() / increment_rad, 
                params_.precision);
        }

        if (begin_index >= cache_.lidar_rays_number
            && end_index >= cache_.lidar_rays_number) {
            continue;
        }

        begin_index = std::min(begin_index, cache_.lidar_rays_number - 1);
        end_index = std::min(end_index, cache_.lidar_rays_number - 1);

        geom::Ray current_ray(origin, 
            getRayDir(dir, cache_.lidar_angle_increment, begin_index));
        const auto increment = geom::AngleVec2(sign * cache_.lidar_angle_increment);
        int index = begin_index - sign;
        
        do {
            index = mod(index + sign, cache_.lidar_rays_number);
            const auto distance = getIntersectionDistance(current_ray, segment, params_.precision);
            /*
            if (index == 0) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("simulator_engine"), 
                    "distance = " + std::to_string(distance));
            }
            */
            
            ranges[index] = std::min(ranges[index], distance);
            current_ray.dir += increment;
        } while (index != end_index);
    }

    return ranges;
}

TruckState SimulatorEngine::getTruckState() const {
    const double steering = rear_ax_state_[StateIndex::kSteering];

    const auto pose = getOdomBasePose();
    const double rear_curvature = model_->middleSteeringToRearCurvature(steering);
    const auto current_steering = getCurrentSteering(rear_curvature);
    const auto target_steering = getTargetSteering();
    const auto twist = rearToOdomBaseTwist(rear_curvature);
    const auto linear_velocity = rearToOdomBaseLinearVelocity(pose.dir, twist.velocity);
    const auto angular_velocity = rearToBaseAngularVelocity(twist.velocity, rear_curvature);
    auto lidar_ranges = getLidarRanges(pose);
    
    return TruckState()
        .time(time_)
        .odomBasePose(pose)
        .currentSteering(current_steering)
        .targetSteering(target_steering)
        .baseTwist(twist)
        .odomBaseLinearVelocity(linear_velocity)
        .baseAngularVelocity(angular_velocity)
        .lidarRanges(std::move(lidar_ranges));
}

namespace {

/**
 * @param desired_velocity The velocity to strive for.
 * @param velocity Current (initial) velocity.
 * @param precision Precision of calculations.
 * 
 * @return A pair of values:
 * first - the maneuver sign of the model;
 * second - the target speed.
 * 
 * If the sign is positive, it is necessary to accelerate. 
 * If the sign is negative, it is necessary to decelerate.
 * 
 * If the current and desired velocities are of different signs,
 * the target speed will be 0 (the model must first stop,
 * and then start moving in the opposite direction).
 * Otherwise, the target and desired velocities are the same.
 */
std::pair<int, double> actionSign(double desired_velocity, double velocity, double precision) {
    const bool need_stop = (softSign(desired_velocity, precision)
        * softSign(velocity, precision)) < 0;
    const double target_velocity = need_stop ? 0 : desired_velocity;

    return {softSign(abs(target_velocity) - abs(velocity), precision), target_velocity};
}

} // namespace

void SimulatorEngine::setBaseControl(
    double velocity, double acceleration, double curvature) {

    VERIFY(acceleration >= 0);

    curvature = model_->baseCurvatureLimits().clamp(curvature);
    velocity = model_->baseVelocityLimits().clamp(velocity);

    const auto base_twist = model::Twist {curvature, velocity};
    const auto rear_twist = model_->baseToRearTwist(base_twist);

    const auto [action_sign, _] = actionSign(rear_twist.velocity,
        rear_ax_state_[StateIndex::kLinearVelocity], params_.precision);
    if (action_sign == 1) {
        acceleration = std::max(acceleration, model_->baseMaxAcceleration());
    }
    else if (action_sign == -1) {
        acceleration = std::max(acceleration, model_->baseMaxDeceleration());
    }

    control_.curvature = rear_twist.curvature;
    control_.velocity = rear_twist.velocity;
    control_.acceleration 
        = model_->baseToRearAcceleration(acceleration, curvature);
}

void SimulatorEngine::setBaseControl(double velocity, double curvature) {
    const auto base_twist = model::Twist {curvature, velocity};
    const auto rear_twist = model_->baseToRearTwist(base_twist);

    const int action_sign = actionSign(rear_twist.velocity,
        rear_ax_state_[StateIndex::kLinearVelocity], params_.precision).first;
    double acceleration = 0;
    if (action_sign == 1) {
        acceleration = model_->baseMaxAcceleration();
    } else if (action_sign == -1) {
        acceleration = model_->baseMaxDeceleration();
    }

    setBaseControl(velocity, acceleration, curvature);
}

namespace {

double getOptionalValue(const std::optional<double>& opt, double max) {
    if (!opt) {
        return max;
    }

    return std::min(*opt, max);
}

} // namespace

double SimulatorEngine::getCurrentAcceleration() const {
    const double velocity = rear_ax_state_[StateIndex::kLinearVelocity];
    
    const auto [action_sign, target_velocity] 
        = actionSign(control_.velocity, velocity, params_.precision);

    const int acceleration_sign = softSign(target_velocity 
        - velocity, params_.precision);

    double current_acceleration = 0;
    if (action_sign == 1) {
        // Acceleration.
        current_acceleration = acceleration_sign 
            * getOptionalValue(control_.acceleration, model_->baseMaxAcceleration());
    } else if (action_sign == -1) {
        // Deceleration.
        current_acceleration = acceleration_sign
            * getOptionalValue(control_.acceleration, model_->baseMaxDeceleration());
    }

    const double velocity_delta = current_acceleration * params_.integration_step;
    const double new_velocity = velocity + velocity_delta;
    const bool target_velocity_achieved 
        = (acceleration_sign > 0 && (new_velocity + params_.precision > target_velocity))
        || (acceleration_sign < 0 && (new_velocity - params_.precision < target_velocity));

    if (target_velocity_achieved) {
        current_acceleration = (target_velocity 
            - velocity) * cache_.inverse_integration_step;
    }

    return current_acceleration;
}

double SimulatorEngine::getCurrentSteeringVelocity() const {
    const double steering = rear_ax_state_[StateIndex::kSteering];
    const double target_steering = getTargetSteering().middle.radians();
    const int velocity_sign = softSign(target_steering - steering, params_.precision);
    double current_velocity = velocity_sign * model_->steeringVelocity();

    const double steering_delta = current_velocity * params_.integration_step;
    const double new_steering = steering + steering_delta;
    const bool target_steering_achieved
        = (velocity_sign > 0 && (new_steering + params_.precision > target_steering))
        || (velocity_sign < 0 && (new_steering - params_.precision < target_steering));

    if (target_steering_achieved) {
        current_velocity = (target_steering 
            - steering) * cache_.inverse_integration_step;
    }

    return current_velocity;
}

SimulatorEngine::State SimulatorEngine::calculateStateDerivative(
    const SimulatorEngine::State &state, double acceleration, double steering_velocity) const {
    
    const double yaw = state[StateIndex::kYaw];
    const double velocity = state[StateIndex::kLinearVelocity];
    const double steering = state[StateIndex::kSteering];

    SimulatorEngine::State deriv;
    deriv.setZero();
    deriv[StateIndex::kX] = cos(yaw) * velocity;
    deriv[StateIndex::kY] = sin(yaw) * velocity;
    deriv[StateIndex::kYaw] = tan(steering) * velocity * cache_.inverse_wheelbase_length;
    deriv[StateIndex::kLinearVelocity] = acceleration;
    deriv[StateIndex::kSteering] = steering_velocity;

    return deriv;
}

SimulatorEngine::State SimulatorEngine::calculateRK4(
    double acceleration, double steering_velocity) const {

    const auto k1 = calculateStateDerivative(rear_ax_state_, acceleration, steering_velocity);
    const auto k2 = calculateStateDerivative(
        rear_ax_state_ + k1 * cache_.integration_step_2, acceleration, steering_velocity);
    const auto k3 = calculateStateDerivative(
        rear_ax_state_ + k2 * cache_.integration_step_2, acceleration, steering_velocity);
    const auto k4 = calculateStateDerivative(
        rear_ax_state_ + k3 * params_.integration_step, acceleration, steering_velocity);

    return (k1 + 2 * k2 + 2 * k3 + k4) * cache_.integration_step_6;
}

namespace {

rclcpp::Duration convertFromSecondsToDuration(double seconds) {
    auto int_seconds = int(seconds);
    auto nanoseconds = (seconds - int_seconds) * 1e9;
    return rclcpp::Duration(int_seconds, int(nanoseconds));
}

} // namespace

void SimulatorEngine::advance(double seconds) {
    time_ += convertFromSecondsToDuration(seconds);

    const int integration_steps = seconds * cache_.inverse_integration_step;

    for (int i = 0; i < integration_steps; ++i) {
        const double current_acceleration = getCurrentAcceleration();
        const double current_steering_velocity = getCurrentSteeringVelocity();
        rear_ax_state_ += calculateRK4(current_acceleration, current_steering_velocity);
    }
}

}  // namespace truck::simulator
