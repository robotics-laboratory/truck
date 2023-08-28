#include "common/array_as_queue.h"
#include "common/exception.h"
#include "common/math.h"
#include "fastgrid/manhattan_distance.h"
#include "geom/vector.h"

#include <limits>
#include <memory>
#include <vector>

namespace truck::fastgrid {

void ManhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps, int* queue_buf,
    F32Grid& manhattan_distance) {
    const int resolution = distance_transform.resolution;
    const auto [width, height] = distance_transform.size;
    const float unreachable = std::numeric_limits<float>::max();
    const int origin_index = distance_transform.GetIndex(source);

    std::fill(
        manhattan_distance.data, manhattan_distance.data + manhattan_distance.size(), unreachable);

    if (distance_transform.data[origin_index] <= eps) {
        return;
    }

    manhattan_distance.data[origin_index] = 0;

    ArrayAsQueue<int> queue(queue_buf);
    queue.Push(origin_index);

    while (!queue.Empty()) {
        const int cur_index = queue.Extract();
        const int i = cur_index / width;
        const int j = cur_index % width;

        for (int dt : {-1, 1}) {
            const int h_next_index = cur_index + dt;
            if ((0 <= j + dt) && (j + dt < width) &&
                (distance_transform.data[h_next_index] > eps) &&
                (manhattan_distance.data[h_next_index] == unreachable)) {
                manhattan_distance.data[h_next_index] =
                    manhattan_distance.data[cur_index] + resolution;
                queue.Push(h_next_index);
            }

            const int v_next_index = cur_index + dt * width;
            if ((0 <= i + dt) && (i + dt < height) &&
                (distance_transform.data[v_next_index] > eps) &&
                (manhattan_distance.data[v_next_index] == unreachable)) {
                manhattan_distance.data[v_next_index] =
                    manhattan_distance.data[cur_index] + resolution;
                queue.Push(v_next_index);
            }
        }
    }
}

void ManhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps,
    F32Grid& manhattan_distance) {
    std::vector<int> queue_buf(distance_transform.size());
    ManhattanDistance(distance_transform, source, eps, queue_buf.data(), manhattan_distance);
}

F32GridHolder ManhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps) {
    F32GridHolder result = MakeGridLike<float>(distance_transform);
    ManhattanDistance(distance_transform, source, eps, *result);
    return result;
}

}  // namespace truck::fastgrid
