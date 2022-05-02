#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include "transform.hpp"

namespace robolab {
namespace aruco {
namespace math {

Transform::Transform(const tf2::Quaternion &rotation, const tf2::Vector3 &translation)
    : rotation(rotation), translation(translation) {}

tf2::Vector3 Transform::apply(const tf2::Vector3 &v) const {
    auto res = tf2::quatRotate(rotation, v);
    res += translation;
    return res;
}

tf2::Vector3 Transform::operator()(const tf2::Vector3 &v) const {
    return apply(v);
}

Transform Transform::operator*(const Transform &other) const {
    return Transform(rotation * other.rotation, (*this)(other.translation));
}

Transform Transform::inverse() const {
    auto inv_rotation = rotation.inverse();
    return Transform(inv_rotation, tf2::quatRotate(inv_rotation, -translation));
}

const tf2::Quaternion& Transform::getRotation() const {
    return rotation;
}

}
}
}