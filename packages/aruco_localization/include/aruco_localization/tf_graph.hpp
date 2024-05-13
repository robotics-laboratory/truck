#include <functional>
#include <opencv2/core.hpp>
#include <vector>

#include "transform.hpp"

namespace rosaruco {

class TfGraph {
  public:
    TfGraph(int nodes_count);

    void AddTransform(int x, int y, const Transform& t);

    void GetBestTransformFromStartNode(
        int start_node, const std::vector<int>& finish_nodes,
        std::vector<Transform>& best_transforms, std::vector<double>& errors);

  private:
    const Transform& GetTransform(int x, int y);

    std::function<double(int, int)> GetWeights();

    class Edge {
      public:
        Edge();
        const Transform& GetAverage() const;
        double GetError() const;
        void AddTransform(const Transform& t);
        bool Empty() const;

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
