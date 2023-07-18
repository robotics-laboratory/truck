#include "fastgrid/distance_transform.h"

#include <limits>

namespace truck::fastgrid {

void DistanceTransformApprox3(const U8Grid& input, S32Grid& buf, F32Grid& out) noexcept {
    const int32_t int_unreachable = std::numeric_limits<int32_t>::max();

    std::fill(buf.data, buf.data + buf.size(), int_unreachable);

    for (int distance = 0; distance < input.size.width + input.size.height - 1; ++distance) {
        int cur_row = std::min(distance, input.size.height - 1);
        int cur_col = distance - cur_row;
        while (cur_row >= 0 && cur_col < input.size.width) {
            if (input[cur_row][cur_col] == 0) {
                buf[cur_row + 1][cur_col + 1] = 0;
                --cur_row;
                ++cur_col;
                continue;
            }
            if (buf[cur_row + 1][cur_col] != int_unreachable &&
                buf[cur_row + 1][cur_col] + 3 < buf[cur_row + 1][cur_col + 1]) {
                buf[cur_row + 1][cur_col + 1] = buf[cur_row + 1][cur_col] + 3;
            }
            if (buf[cur_row][cur_col] != int_unreachable &&
                buf[cur_row][cur_col] + 4 < buf[cur_row + 1][cur_col + 1]) {
                buf[cur_row + 1][cur_col + 1] = buf[cur_row][cur_col] + 4;
            }
            if (buf[cur_row][cur_col + 1] != int_unreachable &&
                buf[cur_row][cur_col + 1] + 3 < buf[cur_row + 1][cur_col + 1]) {
                buf[cur_row + 1][cur_col + 1] = buf[cur_row][cur_col + 1] + 3;
            }
            if (buf[cur_row + 2][cur_col] != int_unreachable &&
                buf[cur_row + 2][cur_col] + 4 < buf[cur_row + 1][cur_col + 1]) {
                buf[cur_row + 1][cur_col + 1] = buf[cur_row + 2][cur_col] + 4;
            }
            --cur_row;
            ++cur_col;
        }
    }

    for (int distance = input.size.width + input.size.height - 2; distance >= 0; --distance) {
        int cur_col = std::min(distance, input.size.width - 1);
        int cur_row = distance - cur_col;
        while (cur_row < input.size.height && cur_col >= 0) {
            if (input[cur_row][cur_col] == 0) {
                buf[cur_row + 1][cur_col + 1] = 0;
                ++cur_row;
                --cur_col;
                continue;
            }
            if (buf[cur_row][cur_col + 2] != int_unreachable &&
                buf[cur_row][cur_col + 2] + 4 < buf[cur_row + 1][cur_col + 1]) {
                buf[cur_row + 1][cur_col + 1] = buf[cur_row][cur_col + 2] + 4;
            }
            if (buf[cur_row + 1][cur_col + 2] != int_unreachable &&
                buf[cur_row + 1][cur_col + 2] + 3 < buf[cur_row + 1][cur_col + 1]) {
                buf[cur_row + 1][cur_col + 1] = buf[cur_row + 1][cur_col + 2] + 3;
            }
            if (buf[cur_row + 2][cur_col + 2] != int_unreachable &&
                buf[cur_row + 2][cur_col + 2] + 4 < buf[cur_row + 1][cur_col + 1]) {
                buf[cur_row + 1][cur_col + 1] = buf[cur_row + 2][cur_col + 2] + 4;
            }
            if (buf[cur_row + 2][cur_col + 1] != int_unreachable &&
                buf[cur_row + 2][cur_col + 1] + 3 < buf[cur_row + 1][cur_col + 1]) {
                buf[cur_row + 1][cur_col + 1] = buf[cur_row + 2][cur_col + 1] + 3;
            }
            ++cur_row;
            --cur_col;
        }
    }

    const float float_unreachable = std::numeric_limits<float>::max();

    std::fill(out.data, out.data + out.size(), float_unreachable);

    for (int i = 0; i < input.size.height; ++i) {
        for (int j = 0; j < input.size.width; ++j) {
            if (buf[i + 1][j + 1] == int_unreachable) {
                continue;
            }
            out[i][j] = static_cast<float>(buf[i + 1][j + 1]) * out.resolution / 3;
        }
    }
}

void DistanceTransformApprox3(const U8Grid& input, F32Grid& out) noexcept {
    Size buf_size = {.width = input.size.width + 2, .height = input.size.height + 2};
    S32Grid buf(buf_size, input.resolution, input.origin);
    S32GridDataPtr buf_data = Allocate<int32_t>(buf_size);
    buf.Reset(buf_data.get());
    DistanceTransformApprox3(input, buf, out);
}

F32GridHolder DistanceTransformApprox3(const U8Grid& input) noexcept {
    F32Grid out(input.size, input.resolution, input.origin);
    F32GridDataPtr out_data = Allocate<float>(input.size);
    out.Reset(out_data.get());
    DistanceTransformApprox3(input, out);
    return F32GridHolder(std::move(out), std::move(out_data));
}

void DistanceTransformApprox5(const U8Grid& input, S32Grid& buf, F32Grid& out) noexcept {
    const int32_t int_unreachable = std::numeric_limits<int32_t>::max();

    std::fill(buf.data, buf.data + buf.size(), int_unreachable);

    for (int distance = 0; distance < input.size.width + input.size.height - 1; ++distance) {
        int cur_row = std::min(distance, input.size.height - 1);
        int cur_col = distance - cur_row;
        while (cur_row >= 0 && cur_col < input.size.width) {
            if (input[cur_row][cur_col] == 0) {
                buf[cur_row + 2][cur_col + 2] = 0;
                --cur_row;
                ++cur_col;
                continue;
            }
            if (buf[cur_row][cur_col + 3] != int_unreachable &&
                buf[cur_row][cur_col + 3] + 11 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row][cur_col + 3] + 11;
            }
            if (buf[cur_row][cur_col + 2] != int_unreachable &&
                buf[cur_row][cur_col + 2] + 10 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row][cur_col + 2] + 10;
            }
            if (buf[cur_row][cur_col + 1] != int_unreachable &&
                buf[cur_row][cur_col + 1] + 11 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row][cur_col + 1] + 11;
            }
            if (buf[cur_row][cur_col] != int_unreachable &&
                buf[cur_row][cur_col] + 14 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row][cur_col] + 14;
            }
            if (buf[cur_row + 1][cur_col + 2] != int_unreachable &&
                buf[cur_row + 1][cur_col + 2] + 5 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row + 1][cur_col + 2] + 5;
            }
            if (buf[cur_row + 1][cur_col + 1] != int_unreachable &&
                buf[cur_row + 1][cur_col + 1] + 7 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row + 1][cur_col + 1] + 7;
            }
            if (buf[cur_row + 1][cur_col] != int_unreachable &&
                buf[cur_row + 1][cur_col] + 11 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row + 1][cur_col] + 11;
            }
            if (buf[cur_row + 2][cur_col + 1] != int_unreachable &&
                buf[cur_row + 2][cur_col + 1] + 5 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row + 2][cur_col + 1] + 5;
            }
            if (buf[cur_row + 2][cur_col] != int_unreachable &&
                buf[cur_row + 2][cur_col] + 10 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row + 2][cur_col] + 10;
            }
            if (buf[cur_row + 3][cur_col + 1] != int_unreachable &&
                buf[cur_row + 3][cur_col + 1] + 7 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row + 3][cur_col + 1] + 7;
            }
            if (buf[cur_row + 3][cur_col] != int_unreachable &&
                buf[cur_row + 3][cur_col] + 11 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row + 3][cur_col] + 11;
            }
            if (buf[cur_row + 4][cur_col] != int_unreachable &&
                buf[cur_row + 4][cur_col] + 14 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row + 4][cur_col] + 14;
            }
            --cur_row;
            ++cur_col;
        }
    }

    for (int distance = input.size.width + input.size.height - 2; distance >= 0; --distance) {
        int cur_col = std::min(distance, input.size.width - 1);
        int cur_row = distance - cur_col;
        while (cur_row < input.size.height && cur_col >= 0) {
            if (input[cur_row][cur_col] == 0) {
                buf[cur_row + 2][cur_col + 2] = 0;
                ++cur_row;
                --cur_col;
                continue;
            }
            if (buf[cur_row][cur_col + 4] != int_unreachable &&
                buf[cur_row][cur_col + 4] + 14 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row][cur_col + 4] + 14;
            }
            if (buf[cur_row + 1][cur_col + 4] != int_unreachable &&
                buf[cur_row + 1][cur_col + 4] + 11 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row + 1][cur_col + 4] + 11;
            }
            if (buf[cur_row + 1][cur_col + 3] != int_unreachable &&
                buf[cur_row + 1][cur_col + 3] + 7 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row + 1][cur_col + 3] + 7;
            }
            if (buf[cur_row + 2][cur_col + 4] != int_unreachable &&
                buf[cur_row + 2][cur_col + 4] + 10 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row + 2][cur_col + 4] + 10;
            }
            if (buf[cur_row + 2][cur_col + 3] != int_unreachable &&
                buf[cur_row + 2][cur_col + 3] + 5 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row + 2][cur_col + 3] + 5;
            }
            if (buf[cur_row + 3][cur_col + 4] != int_unreachable &&
                buf[cur_row + 3][cur_col + 4] + 11 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row + 3][cur_col + 4] + 11;
            }
            if (buf[cur_row + 3][cur_col + 3] != int_unreachable &&
                buf[cur_row + 3][cur_col + 3] + 7 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row + 3][cur_col + 3] + 7;
            }
            if (buf[cur_row + 3][cur_col + 2] != int_unreachable &&
                buf[cur_row + 3][cur_col + 2] + 5 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row + 3][cur_col + 2] + 5;
            }
            if (buf[cur_row + 4][cur_col + 4] != int_unreachable &&
                buf[cur_row + 4][cur_col + 4] + 14 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row + 4][cur_col + 4] + 14;
            }
            if (buf[cur_row + 4][cur_col + 3] != int_unreachable &&
                buf[cur_row + 4][cur_col + 3] + 11 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row + 4][cur_col + 3] + 11;
            }
            if (buf[cur_row + 4][cur_col + 2] != int_unreachable &&
                buf[cur_row + 4][cur_col + 2] + 10 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row + 4][cur_col + 2] + 10;
            }
            if (buf[cur_row + 4][cur_col + 1] != int_unreachable &&
                buf[cur_row + 4][cur_col + 1] + 11 < buf[cur_row + 2][cur_col + 2]) {
                buf[cur_row + 2][cur_col + 2] = buf[cur_row + 4][cur_col + 1] + 11;
            }
            ++cur_row;
            --cur_col;
        }
    }

    for (int i = 0; i < input.size.height; ++i) {
        for (int j = 0; j < input.size.width; ++j) {
            out[i][j] = static_cast<float>(buf[i + 2][j + 2]) * out.resolution / 5;
        }
    }
}

void DistanceTransformApprox5(const U8Grid& input, F32Grid& out) noexcept {
    Size buf_size = {.width = input.size.width + 4, .height = input.size.height + 4};
    S32Grid buf(buf_size, input.resolution, input.origin);
    S32GridDataPtr buf_data = Allocate<int32_t>(buf_size);
    buf.Reset(buf_data.get());
    DistanceTransformApprox5(input, buf, out);
}

F32GridHolder DistanceTransformApprox5(const U8Grid& input) noexcept {
    F32Grid out(input.size, input.resolution, input.origin);
    F32GridDataPtr out_data = Allocate<float>(input.size);
    out.Reset(out_data.get());
    DistanceTransformApprox5(input, out);
    return F32GridHolder(std::move(out), std::move(out_data));
}

}  // namespace truck::fastgrid
