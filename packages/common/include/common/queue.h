#pragma once

#include <memory>

namespace truck {

template<typename T>
class Queue {
  private:
    using SizeType = size_t;

  public:
    Queue() : data_(nullptr), capacity_(0), size_(0), begin_(0) {}

    Queue(const Queue& other) noexcept : Queue() {
        Reallocate(other.capacity_);
        capacity_ = other.capacity_;
        size_ = other.size_;
        std::copy(
            other.data_ + other.begin_,
            other.data_ + other.begin_ + other.size_,
            data_.get() + other.begin_);
        begin_ = other.begin_;
    }

    Queue(SizeType size) noexcept : Queue() { Reallocate(size); }

    Queue(Queue&& other) noexcept : Queue() { Swap(other); }

    ~Queue() = default;

    Queue& operator=(const Queue& other) & noexcept {
        Queue object(other);
        Swap(object);
        return *this;
    }

    Queue& operator=(Queue&& other) & noexcept {
        Swap(other);
        return *this;
    }

    void Push(const T& value) noexcept {
        if (size_ == capacity_) {
            Reallocate((capacity_ == 0 ? 1 : 2 * capacity_));
        }
        *(data_.get() + (begin_ + size_) % capacity_) = value;
        ++size_;
    }

    void Push(T&& value) noexcept {
        if (size_ == capacity_) {
            Reallocate((capacity_ == 0 ? 1 : 2 * capacity_));
        }
        *(data_.get() + (begin_ + size_) % capacity_) = std::move(value);
        ++size_;
    }

    void Pop() noexcept {
        --size_;
        begin_ = (begin_ + 1) % capacity_;
    }

    const T& Front() const noexcept { return *(data_.get() + begin_); }

    T& Front() noexcept { return *(data_.get() + begin_); }

    void Clear() noexcept {
        size_ = 0;
        begin_ = 0;
    }

    SizeType Size() const noexcept { return size_; }

    bool Empty() noexcept { return Size() == 0; }

  private:
    std::unique_ptr<T[]> data_;
    SizeType capacity_;
    SizeType size_;
    SizeType begin_;

    void Reallocate(SizeType new_capacity) noexcept {
        if (new_capacity <= capacity_) {
            return;
        }
        std::unique_ptr<T[]> new_data = std::make_unique<T[]>(new_capacity);
        std::copy(
            data_.get() + begin_,
            data_.get() + std::min(begin_ + size_, capacity_),
            new_data.get());
        std::copy(
            data_.get(),
            data_.get() + std::max(static_cast<SizeType>(0), begin_ + size_ - capacity_),
            new_data.get() + capacity_ - begin_);
        std::swap(data_, new_data);
        capacity_ = new_capacity;
        begin_ = 0;
    }

    void Swap(Queue& other) noexcept {
        data_.swap(other.data_);
        std::swap(capacity_, other.capacity_);
        std::swap(size_, other.size_);
        std::swap(begin_, other.begin_);
    }
};

}  // namespace truck