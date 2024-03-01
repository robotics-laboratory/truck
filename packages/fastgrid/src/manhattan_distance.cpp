#include "fastgrid/manhattan_distance.h"
#include "fastgrid/misc.h"

#include "common/exception.h"
#include "common/math.h"

#include <limits>
#include <vector>

namespace truck::fastgrid {

void manhattanDistance(
    const F32Grid& distance_transform, ArrayAsQueue<int>& queue, float eps,
    F32Grid& manhattan_distance) {
    const int resolution = distance_transform.resolution;
    const auto [width, height] = distance_transform.size;
    const float unreachable = std::numeric_limits<float>::max();

    std::fill(
        manhattan_distance.data, manhattan_distance.data + manhattan_distance.size(), unreachable);

    for (size_t i = 0; i < queue.size(); ++i) {
        const int cur_index = queue.data()[i];
        if (distance_transform.data[cur_index] <= eps) {
            continue;
        }
        manhattan_distance.data[cur_index] = 0;
    }

    while (!queue.empty()) {
        const int cur_index = queue.pop();

        if (manhattan_distance.data[cur_index] == unreachable) {
            continue;
        }

        const float next_weight = manhattan_distance.data[cur_index] + resolution;
        const auto [x, y] = distance_transform.toIndex(cur_index);

        if (0 < x) {
            const int left = cur_index - 1;
            if (distance_transform.data[left] > eps &&
                manhattan_distance.data[left] == unreachable) {
                manhattan_distance.data[left] = next_weight;
                queue.push(left);
            }
        }

        if (x < width - 1) {
            const int right = cur_index + 1;
            if (distance_transform.data[right] > eps &&
                manhattan_distance.data[right] == unreachable) {
                manhattan_distance.data[right] = next_weight;
                queue.push(right);
            }
        }

        if (0 < y) {
            const int up = cur_index - width;
            if (distance_transform.data[up] > eps && manhattan_distance.data[up] == unreachable) {
                manhattan_distance.data[up] = next_weight;
                queue.push(up);
            }
        }

        if (y < height - 1) {
            const int down = cur_index + width;
            if (down < manhattan_distance.size() && distance_transform.data[down] > eps &&
                manhattan_distance.data[down] == unreachable) {
                manhattan_distance.data[down] = next_weight;
                queue.push(down);
            }
        }
    }
}

F32GridHolder manhattanDistance(
    const F32Grid& distance_transform, ArrayAsQueue<int>& queue, float eps) {
    F32GridHolder result = makeGridLike<float>(distance_transform);
    manhattanDistance(distance_transform, queue, eps, *result);
    return result;
}

void manhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps, int* queue_buf,
    F32Grid& manhattan_distance) {
    ArrayAsQueue<int> queue(queue_buf);
    const auto origin_index = *VERIFY(distance_transform.tryGetPlainIndex(source));
    queue.push(origin_index);
    manhattanDistance(distance_transform, queue, eps, manhattan_distance);
}

void manhattanDistance(
    const F32Grid& distance_transform, const std::vector<geom::Vec2>& sources, float eps,
    int* queue_buf, F32Grid& manhattan_distance) {
    ArrayAsQueue<int> queue(queue_buf);
    for (const auto& source : sources) {
        const auto origin_index = *VERIFY(distance_transform.tryGetPlainIndex(source));
        queue.push(origin_index);
    }
    manhattanDistance(distance_transform, eps, queue_buf, queue.size(), manhattan_distance);
}

void manhattanDistance(
    const F32Grid& distance_transform, const std::vector<geom::Vec2>& sources, float eps,
    F32Grid& manhattan_distance) {
    std::vector<int> queue_buf(distance_transform.size());
    manhattanDistance(distance_transform, sources, eps, queue_buf.data(), manhattan_distance);
}

F32GridHolder manhattanDistance(
    const F32Grid& distance_transform, const std::vector<geom::Vec2>& sources, float eps) {
    F32GridHolder result = makeGridLike<float>(distance_transform);
    manhattanDistance(distance_transform, sources, eps, *result);
    return result;
}

void manhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps, int* queue_buf,
    F32Grid& manhattan_distance) {
    manhattanDistance(
        distance_transform, std::vector<geom::Vec2>{source}, eps, queue_buf, manhattan_distance);
}

void manhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps,
    F32Grid& manhattan_distance) {
    manhattanDistance(distance_transform, std::vector<geom::Vec2>{source}, eps, manhattan_distance);
}

F32GridHolder manhattanDistance(
    const F32Grid& distance_transform, const geom::Vec2& source, float eps) {
    return manhattanDistance(distance_transform, std::vector<geom::Vec2>{source}, eps);
}

}  // namespace truck::fastgrid
