#pragma once

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

namespace rosaruco {

class Transform {
private:

    tf2::Quaternion rotation_;
    tf2::Vector3 translation_;

public:   

    Transform(const tf2::Quaternion &rotation, const tf2::Vector3 &translation);
    
    tf2::Vector3 Apply(const tf2::Vector3 &v) const;

    tf2::Vector3 operator()(const tf2::Vector3 &v) const;

    const tf2::Quaternion& GetRotation() const;

    const tf2::Vector3& GetTranslation() const;

    void SetRotation(const tf2::Quaternion& r);

    void SetTranslation(const tf2::Vector3& t);

    Transform operator*(const Transform &other) const;

    Transform Inverse() const;
};

} // namespace rosaruco
