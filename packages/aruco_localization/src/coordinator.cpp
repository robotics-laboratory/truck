#include "coordinator.hpp"
#include "graph.hpp"
#include "math/math_helpers.hpp"

#include <functional>
#include <rclcpp/rclcpp.hpp>

namespace rosaruco {

const static double kDefaultEdgeError = 1e9;

Coordinator::Coordinator(int nodes_count) : nodes_count_(nodes_count){
    edges_.resize(nodes_count, std::vector<Edge>(nodes_count));
}

void Coordinator::Update(const std::vector<int> &ids, const std::vector<cv::Vec3d> &rvecs, const std::vector<cv::Vec3d> &tvecs) {
    std::vector<Transform> transforms;
    transforms.reserve(rvecs.size());

    for (size_t i = 0; i < ids.size(); i++) {
        transforms.push_back(GetTransform(rvecs[i], tvecs[i]));
    }

    Update(ids, transforms);
}

void Coordinator::Update(const std::vector<int> &ids, const std::vector<Transform> &from_marker_to_cam) {
    if (ids.empty()) {
        return;
    }

    for (size_t i = 0; i < ids.size(); i++) {
        for (size_t j = 0; j < ids.size(); j++) {
            if (i != j) {
                edges_[ids[i]][ids[j]].AddTransform(from_marker_to_cam[j].inverse() * from_marker_to_cam[i]);
            }
        }
    }

    std::vector<double> distance;
    std::vector<int> last_node;

    Dijkstra(nodes_count_, anchor_id_, distance, last_node, 
        std::function<double(int, int)>([this](int x, int y){
            return edges_[x][y].GetError();
        })
    );

    for (size_t i = 0; i < ids.size(); i++) {
        to_anchor_transform_[i] = distance[ids[i]];
    }

    size_t best = 0;

    for (size_t i = 1; i < ids.size(); i++) {
        if (distance[ids[best]] > distance[ids[i]]) {
            best = i;
        }
    }

    int node = ids[best];

    Transform to_anchor({0, 0, 0, 1}, {0, 0, 0});

    while (node != anchor_id_) {
        to_anchor = to_anchor * edges_[node][last_node[node]].GetAverage();
    }

    to_anchor = to_anchor * from_marker_to_cam[best].inverse();

    Pose new_pose;
    new_pose.orientation = to_anchor.getRotation() * tf2::Quaternion(tf2::Vector3(0, 1, 0), -M_PI / 2) 
                * tf2::Quaternion(tf2::Vector3(1, 0, 0), M_PI / 2);
    new_pose.point = to_anchor({0, 0, 0});

    current_pose_ = new_pose;
}

const std::optional<Transform>& Coordinator::GetTransformToAnchor(int from_id) const {
    return to_anchor_transform_[from_id];
}

Pose Coordinator::GetPose() {
    return current_pose_;
}


void Coordinator::SetAnchorId(int id) {
    assert(anchor_id_ == -1);
    anchor_id_ = id;
}

Coordinator::Edge::Edge() :
    quaternion_sum_(4, 4, CV_64F), 
    average_translation_({0, 0, 0}),
    transforms_count_(0),
    average_transform_({0, 0, 0, 0}, {0, 0, 0}),
    error_(kDefaultEdgeError) {}

const Transform& Coordinator::Edge::GetAverage() const {
    return average_transform_;   
}

double Coordinator::Edge::GetError() const {
    return error_;
}

void Coordinator::Edge::AddTransform(const Transform& t) {
    const auto &rotation = t.getRotation();
    cv::Mat quat_vec(4, 1, CV_64F);
    quat_vec.at<double>(0, 0) = rotation.x();
    quat_vec.at<double>(1, 0) = rotation.y();
    quat_vec.at<double>(2, 0) = rotation.z();
    quat_vec.at<double>(3, 0) = rotation.w();

    quaternion_sum_ = quaternion_sum_ * transforms_count_ / (transforms_count_ + 1) 
        + quat_vec * quat_vec.t() / (transforms_count_ + 1);

    average_translation_ = average_translation_ * transforms_count_  / (transforms_count_ + 1) 
        + t.getTranslation() / (transforms_count_ + 1);

    transforms_count_++;

    cv::Mat w, u, vt;
    cv::SVD::compute(quaternion_sum_, w, u, vt);
    
    average_transform_.setRotation(tf2::Quaternion(
        vt.at<double>(0, 0), 
        vt.at<double>(0, 1),
        vt.at<double>(0, 2),
        vt.at<double>(0, 3)
    ));

    average_transform_.setTranslation(average_translation_);

    w.at<double>(0) = 0;
    error_ = cv::norm(w);
}

bool Coordinator::Edge::Empty() const {
    return transforms_count_ > 0;
}

}
