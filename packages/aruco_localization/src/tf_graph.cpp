#include "graph_algorithms.hpp"
#include "math_helpers.hpp"
#include "tf_graph.hpp"

namespace rosaruco {

const static double kDefaultEdgeError = 1e9;

TfGraph::TfGraph(int nodes_count) : nodes_count_(nodes_count) {
    edges_.resize(nodes_count);
    for (auto &row : edges_) {
        row.resize(nodes_count);
    }
}

void TfGraph::AddTransform(int x, int y, const Transform& t) {
    edges_[x][y].AddTransform(t);
}

std::function<double(int, int)> TfGraph::GetWeights() {
    return std::function<double(int, int)>(
        [this](int x, int y){
            return edges_[x][y].GetError();
        }
    );
}

const Transform& TfGraph::GetTransform(int x, int y) {
    return edges_[x][y].GetAverage();
}

void TfGraph::GetBestTransformFromStartNode(int start_node, const std::vector<int> &finish_nodes,
        std::vector<Transform>& best_transforms, std::vector<double>& errors) {
    std::vector<double> distance;
    std::vector<int> prev_node;

    Dijkstra(nodes_count_, start_node, distance, prev_node, GetWeights());

    best_transforms.clear();
    best_transforms.reserve(finish_nodes.size());

    errors.clear();
    errors.reserve(finish_nodes.size());

    for (const int &finish_node : finish_nodes) {

        if (std::isinf(distance[finish_node])) {
            continue;   
        }

        errors.push_back(distance[finish_node]);

        int current_node = finish_node;

        Transform from_finish_to_current({0, 0, 0, 1}, {0, 0, 0});
        while (current_node != start_node) {
            from_finish_to_current = 
                GetTransform(current_node, prev_node[current_node]) * from_finish_to_current;
            current_node = prev_node[current_node];
        }

        best_transforms.push_back(from_finish_to_current);
    }
}

TfGraph::Edge::Edge() :
    average_translation_({0, 0, 0}),
    transforms_count_(0),
    average_transform_({0, 0, 0, 0}, {0, 0, 0}),
    error_(kDefaultEdgeError) {
        
    quaternion_sum_ = cv::Mat::zeros(4, 4, CV_64F);
}

const Transform& TfGraph::Edge::GetAverage() const {
    return average_transform_;   
}

double TfGraph::Edge::GetError() const {
    if (Empty()) {
        return std::numeric_limits<double>::infinity();
    }
    return error_;
}

void TfGraph::Edge::AddTransform(const Transform& t) {
    const auto &rotation = t.GetRotation();
    cv::Mat quat_vec(4, 1, CV_64F);
    quat_vec.at<double>(0, 0) = rotation.x();
    quat_vec.at<double>(1, 0) = rotation.y();
    quat_vec.at<double>(2, 0) = rotation.z();
    quat_vec.at<double>(3, 0) = rotation.w();

    quaternion_sum_ = quaternion_sum_ * transforms_count_ / (transforms_count_ + 1) 
        + quat_vec * quat_vec.t() / (transforms_count_ + 1);

    average_translation_ = average_translation_ * transforms_count_  / (transforms_count_ + 1) 
        + t.GetTranslation() / (transforms_count_ + 1);

    auto translation_square = ElementWiseMul(t.GetTranslation(), t.GetTranslation());

    average_translation_square_ = average_translation_square_ * transforms_count_  / (transforms_count_ + 1) 
        + translation_square / (transforms_count_ + 1);

    transforms_count_++;

    auto dispersion = average_translation_square_ - ElementWiseMul(average_translation_, average_translation_);

    translation_error_square_ = 0;

    for (size_t i = 0; i < 3; i++) {
        translation_error_square_ += dispersion[i];
    }

    cv::Mat w, u, vt;
    cv::SVD::compute(quaternion_sum_, w, u, vt);

    average_transform_.SetRotation(tf2::Quaternion(
        vt.at<double>(0, 0), 
        vt.at<double>(0, 1),
        vt.at<double>(0, 2),
        vt.at<double>(0, 3)
    ));

    average_transform_.SetTranslation(average_translation_);

    quaternion_error_square_ = 0;
    for (size_t i = 1; i < 4; i++) {
        quaternion_error_square_ += w.at<double>(i) * w.at<double>(i);
    }

    error_ = sqrt(translation_error_square_ + quaternion_error_square_);
}

bool TfGraph::Edge::Empty() const {
    return transforms_count_ == 0;
}

} // namespace rosaruco
