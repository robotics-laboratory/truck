#include "transform.hpp"

namespace rosaruco {

Transform::Transform(const tf2::Quaternion& rotation, const tf2::Vector3& translation) :
    rotation_(rotation.normalized()), translation_(translation) {}

tf2::Vector3 Transform::apply(const tf2::Vector3& v) const {
    auto res = tf2::quatRotate(rotation_, v);
    res += translation_;
    return res;
}

tf2::Vector3 Transform::operator()(const tf2::Vector3& v) const { return apply(v); }

Transform Transform::operator*(const Transform& other) const {
    return Transform((rotation_ * other.rotation_).normalized(), (*this)(other.translation_));
}

Transform Transform::inverse() const {
    auto inv_rotation = rotation_.inverse();
    return Transform(inv_rotation, tf2::quatRotate(inv_rotation, -translation_));
}

const tf2::Quaternion& Transform::getRotation() const { return rotation_; }

const tf2::Vector3& Transform::getTranslation() const { return translation_; }

void Transform::setRotation(const tf2::Quaternion& r) { rotation_ = r; }

void Transform::setTranslation(const tf2::Vector3& t) { translation_ = t; }

}  // namespace rosaruco
