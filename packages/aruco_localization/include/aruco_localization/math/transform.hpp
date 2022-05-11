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
    
    tf2::Vector3 apply(const tf2::Vector3 &v) const;

    tf2::Vector3 operator()(const tf2::Vector3 &v) const;

    const tf2::Quaternion& getRotation() const;

    const tf2::Vector3& getTranslation() const;

    void setRotation(const tf2::Quaternion& r);

    void setTranslation(const tf2::Vector3& t);

    Transform operator*(const Transform &other) const;

    Transform inverse() const;
};

}
