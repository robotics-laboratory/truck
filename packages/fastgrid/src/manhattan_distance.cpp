#include "common/exception.h"
#include "common/math.h"
#include "fastgrid/manhattan_distance.h"
#include "geom/vector.h"

#include <limits>

namespace truck::fastgrid {

void ManhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps,
    std::vector<int>& queue_buf, F32Grid& manhattan_distance) {
    const int resolution = distance_transform.resolution;
    const int width = distance_transform.size.width;
    const int height = distance_transform.size.height;
    const float unreachable = std::numeric_limits<float>::max();
    const int row = clamp(static_cast<int>(source.y / resolution), 0, height - 1);
    const int col = clamp(static_cast<int>(source.x / resolution), 0, width - 1);

    std::fill(
        manhattan_distance.data, manhattan_distance.data + manhattan_distance.size(), unreachable);

    if (distance_transform[row][col] <= eps) {
        return;
    }

    manhattan_distance[row][col] = 0;

    int queue_buf_front = 0;
    int queue_buf_back = 1;
    queue_buf[queue_buf_front] = row * distance_transform.size.width + col;

    while (queue_buf_front < queue_buf_back) {
        const int cur_row = queue_buf[queue_buf_front] / width;
        const int cur_col = queue_buf[queue_buf_front] % width;
        ++queue_buf_front;

        for (int dt : {-1, 1}) {
            if ((0 <= cur_row + dt) && (cur_row + dt < width) &&
                (distance_transform[cur_row + dt][cur_col] > eps) &&
                (manhattan_distance[cur_row + dt][cur_col] == unreachable)) {
                manhattan_distance[cur_row + dt][cur_col] =
                    manhattan_distance[cur_row][cur_col] + resolution;
                queue_buf[queue_buf_back++] =
                    (cur_row + dt) * distance_transform.size.width + cur_col;
            }

            if ((0 <= cur_col + dt) && (cur_col + dt < height) &&
                (distance_transform[cur_row][cur_col + dt] > eps) &&
                (manhattan_distance[cur_row][cur_col + dt] == unreachable)) {
                manhattan_distance[cur_row][cur_col + dt] =
                    manhattan_distance[cur_row][cur_col] + resolution;
                queue_buf[queue_buf_back++] =
                    cur_row * distance_transform.size.width + (cur_col + dt);
            }
        }
    }
}

void ManhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps,
    F32Grid& manhattan_distance) {
    std::vector<int> queue_buf(distance_transform.size());
    ManhattanDistance(distance_transform, source, eps, queue_buf, manhattan_distance);
}

F32GridHolder ManhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps) {
    F32Grid manhattan_distance(
        distance_transform.size, distance_transform.resolution, distance_transform.origin);
    F32GridDataPtr manhattan_distance_ptr = Allocate<float>(distance_transform.size);
    manhattan_distance.Reset(manhattan_distance_ptr.get());
    ManhattanDistance(distance_transform, source, eps, manhattan_distance);
    return F32GridHolder(std::move(manhattan_distance), std::move(manhattan_distance_ptr));
}
}  // namespace truck::fastgrid
