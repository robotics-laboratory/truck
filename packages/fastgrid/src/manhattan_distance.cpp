#include "common/exception.h"
#include "common/math.h"
#include "fastgrid/manhattan_distance.h"
#include "geom/vector.h"

#include <limits>

namespace truck::fastgrid {

void ManhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps, Queue<int>& queue_buf,
    F32Grid& manhattan_distance) {
    const int resolution = distance_transform.resolution;
    const int width = distance_transform.size.width;
    const int height = distance_transform.size.height;
    const int grid_size = distance_transform.size();
    const float unreachable = std::numeric_limits<float>::max();

    const int row = (source.y - distance_transform.origin->pos.y) / resolution;
    const int col = (source.x - distance_transform.origin->pos.x) / resolution;
    const int origin_index = row * width + col;

    if (row < 0 || row >= height || col < 0 || col >= width) {
        throw std::domain_error("Point is not inside the grid");
    }

    std::fill(
        manhattan_distance.data, manhattan_distance.data + manhattan_distance.size(), unreachable);

    if (distance_transform.data[origin_index] <= eps) {
        return;
    }

    manhattan_distance.data[origin_index] = 0;
    queue_buf.Push(origin_index);

    while (!queue_buf.Empty()) {
        const int cur_index = queue_buf.Front();
        queue_buf.Pop();

        for (int dt : {-1, 1}) {
            const int h_next_index = cur_index + dt;
            if ((0 <= h_next_index) && (h_next_index / width == cur_index / width) &&
                (distance_transform.data[h_next_index] > eps) &&
                (manhattan_distance.data[h_next_index] == unreachable)) {
                manhattan_distance.data[h_next_index] =
                    manhattan_distance.data[cur_index] + resolution;
                queue_buf.Push(h_next_index);
            }

            const int v_next_index = cur_index + dt * width;
            if ((0 <= v_next_index) && (v_next_index < grid_size) &&
                (distance_transform.data[v_next_index] > eps) &&
                (manhattan_distance.data[v_next_index] == unreachable)) {
                manhattan_distance.data[v_next_index] =
                    manhattan_distance.data[cur_index] + resolution;
                queue_buf.Push(v_next_index);
            }
        }
    }
}

void ManhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps,
    F32Grid& manhattan_distance) {
    Queue<int> queue_buf(distance_transform.size());
    ManhattanDistance(distance_transform, source, eps, queue_buf, manhattan_distance);
}

F32GridHolder ManhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps) {
    F32GridHolder result = MakeGridLike<float>(distance_transform);
    ManhattanDistance(distance_transform, source, eps, *result);
    return result;
}
}  // namespace truck::fastgrid
