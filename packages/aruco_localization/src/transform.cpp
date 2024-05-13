#include "transform.hpp"

namespace rosaruco {

Transform::Transform(const tf2::Quaternion &rotation, const tf2::Vector3 &translation)
    : rotation_(rotation.normalized()), translation_(translation) {}

tf2::Vector3 Transform::Apply(const tf2::Vector3 &v) const {
    auto res = tf2::quatRotate(rotation_, v);
    res += translation_;
    return res;
}

tf2::Vector3 Transform::operator()(const tf2::Vector3 &v) const { return Apply(v); }

Transform Transform::operator*(const Transform &other) const {
    return Transform((rotation_ * other.rotation_).normalized(), (*this)(other.translation_));
}

Transform Transform::Inverse() const {
    auto inv_rotation = rotation_.inverse();
    return Transform(inv_rotation, tf2::quatRotate(inv_rotation, -translation_));
}

const tf2::Quaternion &Transform::GetRotation() const { return rotation_; }

const tf2::Vector3 &Transform::GetTranslation() const { return translation_; }

void Transform::SetRotation(const tf2::Quaternion &r) { rotation_ = r; }

void Transform::SetTranslation(const tf2::Vector3 &t) { translation_ = t; }

}  // namespace rosaruco
