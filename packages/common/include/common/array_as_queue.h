#pragma once

#include <cstddef>
#include <utility>

namespace truck {

template<typename T>
class ArrayAsQueue {
  public:
    ArrayAsQueue() : head_(nullptr), tail_(nullptr) {}

    ArrayAsQueue(T* arr) : head_(arr), tail_(arr) {}

    void push(const T& value) noexcept {
        *tail_ = value;
        ++tail_;
    }

    void push(T&& value) noexcept {
        *tail_ = std::move(value);
        ++tail_;
    }

    const T& front() const noexcept { return *head_; }

    T& front() noexcept { return *head_; }

    T pop() noexcept { return *(head_++); }

    size_t size() const noexcept { return tail_ - head_; }

    bool empty() const noexcept { return head_ == tail_; }

    T* data() noexcept { return head_; }

  private:
    T* head_;
    T* tail_;
};

}  // namespace truck
