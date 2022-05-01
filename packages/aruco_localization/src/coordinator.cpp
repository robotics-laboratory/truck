#include "coordinator.hpp"
#include "math_helpers.hpp"

#include <iostream>
#include <rclcpp/rclcpp.hpp>


namespace robolab {
namespace aruco {

Coordinator::Coordinator(int marker_count) {
    to_anchor_transform_.resize(marker_count);
    to_anchor_transform_[0] = tf2::Transform(tf2::Quaternion(0, 0, 0, 1), {0, 0, 0});
}

void Coordinator::update(const std::vector<int> &ids, const std::vector<cv::Vec3d> &rvecs, const std::vector<cv::Vec3d> &tvecs) {
    std::vector<tf2::Transform> transforms;
    transforms.reserve(rvecs.size());

    for (size_t i = 0; i < ids.size(); i++) {
        transforms.push_back(math_helpers::GetTransform(rvecs[i], tvecs[i]));
    }

    update(ids, transforms);
}


void Coordinator::update(const std::vector<int> &ids, const std::vector<tf2::Transform> &transforms) {

    for (size_t i = 0; i < ids.size(); i++) {
        for (size_t j = 0; j < ids.size(); j++) {
            if (!to_anchor_transform_[ids[i]] && to_anchor_transform_[ids[j]]) {
                to_anchor_transform_[ids[i]] = *to_anchor_transform_[ids[j]] * transforms[j].inverse() * transforms[i];
            }
        }
    }

    for (size_t i = 0; i < ids.size(); i++) {
        if (to_anchor_transform_[ids[i]]) {
            auto from_cam_to_anchor = transforms[i].inverse() * *to_anchor_transform_[ids[i]];
            current_pose_.point = from_cam_to_anchor({0, 0, 0});
            current_pose_.orientation = from_cam_to_anchor.getRotation()
                * tf2::Quaternion(tf2::Vector3(0, 1, 0), -M_PI / 2) 
                * tf2::Quaternion(tf2::Vector3(1, 0, 0), M_PI / 2);
            break;
        }
    }
}

Pose Coordinator::get_pose() {
    return current_pose_;
}

}
}