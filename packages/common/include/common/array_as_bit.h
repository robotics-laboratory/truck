#pragma once

#include <cstddef>
#include <type_traits>

namespace truck {

/** Binary Indexed Tree (Fenwick Tree)
 *
 * See https://en.wikipedia.org/wiki/Fenwick_tree
 */
template<typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
class ArrayAsBinaryIndexedTree {
  public:
    ArrayAsBinaryIndexedTree() = delete;

    ArrayAsBinaryIndexedTree(T* data, size_t size) : data_(data), size_(size) {
        for (size_t k = 1; k <= size_; ++k) {
            size_t parent_k = k + (k & -k);
            if (parent_k <= size_) {
                *(data_ + (parent_k - 1)) += *(data_ + (k - 1));
            }
        }
    }

    void Add(size_t i, const T& val) noexcept {
        for (size_t k = i + 1; k <= size_; k += k & -k) {
            *(data_ + (k - 1)) += val;
        }
    }

    T Sum(size_t r) const noexcept {
        T sum = 0;
        for (; r > 0; r -= r & -r) {
            sum += *(data_ + (r - 1));
        }
        return sum;
    }

    T Sum(size_t l, size_t r) const noexcept { return Sum(r) - Sum(l); }

    size_t LowerBound(T sum) const noexcept {
        size_t k = 0;
        for (size_t b = std::__lg(size_) + 1; b > 0; --b) {
            size_t next_k = k + (1 << (b - 1));
            if (next_k <= size_ && *(data_ + (next_k - 1)) < sum) {
                k = next_k;
                sum -= *(data_ + (next_k - 1));
            }
        }
        return k;
    }

    const T* Data() const noexcept { return data_; }

    T* Data() noexcept { return data_; }

    size_t Size() const noexcept { return size_; }

  private:
    T* data_ = nullptr;
    size_t size_ = 0;
};

}  // namespace truck