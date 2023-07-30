#pragma once

#include <cstddef>
#include <utility>

namespace truck {

template<typename T>
class ArrayAsQueue {
  public:
    ArrayAsQueue() : head_(nullptr), tail_(nullptr) {}

    ArrayAsQueue(T* arr) : head_(arr), tail_(arr) {}

    void Push(const T& value) noexcept {
        *tail_ = value;
        ++tail_;
    }

    void Push(T&& value) noexcept {
        *tail_ = std::move(value);
        ++tail_;
    }

    const T& Front() const noexcept { return *head_; }

    T& Front() noexcept { return *head_; }

    T Extract() noexcept { return *(head_++); }

    void Pop() noexcept { ++head_; }

    size_t Size() const noexcept { return tail_ - head_; }

    bool Empty() const noexcept { return head_ == tail_; }

  private:
    T* head_;
    T* tail_;
};

}  // namespace truck