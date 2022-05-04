#pragma once

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace rosaruco {

class Transform {
public:
    tf2::Quaternion rotation;
    tf2::Vector3 translation;

    Transform(const tf2::Quaternion &rotation, const tf2::Vector3 &translation);
    
    tf2::Vector3 apply(const tf2::Vector3 &v) const;

    tf2::Vector3 operator()(const tf2::Vector3 &v) const;

    const tf2::Quaternion& getRotation() const;

    Transform operator*(const Transform &other) const;

    Transform inverse() const;
};

}
