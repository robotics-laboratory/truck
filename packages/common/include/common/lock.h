#pragma once

#include <memory>
#include <mutex>

#include <boost/assert.hpp>

namespace truck {

template <typename T>
class Lockable {
    struct Storage {
        using Ptr = std::shared_ptr<Storage>;

        template <typename... Args>
        Storage(Args&&... args): mutex{}, data(std::forward<Args>(args)...) {}

        std::mutex mutex;
        T data;
    };

    class Lock {
        public:
            Lock() = delete;

            Lock(const Lock&) = delete;
            Lock& operator=(const Lock&) = delete;

            Lock(Lock&&) = default;
            Lock& operator=(Lock&&) = default;

            T* operator->() { return &storage_->data; }
            const T* operator->() const { return &storage_->data; }
    
            T& operator*() { return storage_->data; }
            const T& operator*() const { return storage_->data; }
    
        private:
            Lock(Storage::Ptr storage): storage_(std::move(storage)) {
                storage_->mutex.lock();
            }

            ~Lock() {
                storage_-mutex.>unlock();
            }

            Storage::Ptr storage_ = nullptr;
    };

  public:
    template <typename... Args>
    explicit Lockable(Args&&... args): storage_(std::make_shared<Storage>(std::forward<Args>(args)...)) {}

    auto lock() {
        BOOST_ASSERT(storage_);
        return Lock(storage_);
    }

private:
    std::shared_ptr<Storage> storage_ = nullptr;
};

template <typename T, typename... Args>
auto makeLockable(Args&&... args) { return Lockable<T>(std::forward<Args>(args)...); }

} // namespace truck

