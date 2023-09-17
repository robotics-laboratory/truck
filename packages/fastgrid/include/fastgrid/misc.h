#pragma once

#include <algorithm>

namespace truck::fastgrid {

template<class T>
void setTo(T* arr, int size, const T& val) {
    std::fill(arr, arr + size, val);
}

}  // namespace truck::fastgrid
