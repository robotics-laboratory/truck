#include <functional>
#include <opencv2/core.hpp>
#include <vector>

#include "transform.hpp"

namespace rosaruco {

class TfGraph {
  public:
    TfGraph(int nodes_count);

    void addTransform(int x, int y, const Transform& t);

    void getBestTransformFromStartNode(
        int start_node, const std::vector<int>& finish_nodes,
        std::vector<Transform>& best_transforms, std::vector<double>& errors);

  private:
    const Transform& getTransform(int x, int y);

    std::function<double(int, int)> getWeights();

    class Edge {
      public:
        Edge();
        const Transform& getAverage() const;
        double getError() const;
        void addTransform(const Transform& t);
        bool empty() const;

      private:
        cv::Mat quaternion_sum_;
        tf2::Vector3 average_translation_;
        tf2::Vector3 average_translation_square_;
        int transforms_count_;

        Transform average_transform_;

        double quaternion_error_square_;
        double translation_error_square_;
        double error_;
    };

    int nodes_count_;
    std::vector<std::vector<Edge>> edges_;
};

}  // namespace rosaruco
