#pragma once

#include "fastgrid/grid.h"

#include <cstdint>
#include <memory>

namespace truck::fastgrid {

template<typename T>
using GridDataPtr = std::unique_ptr<T[]>;

using U8GridDataPtr = GridDataPtr<uint8_t>;
using S32GridDataPtr = GridDataPtr<int32_t>;
using U32GridDataPtr = GridDataPtr<uint32_t>;
using F32GridDataPtr = GridDataPtr<float>;

template<typename T>
GridDataPtr<T> Allocate(const Size& size) noexcept {
    return std::make_unique<T[]>(size());
}

template<typename T>
struct GridHolder {
    GridHolder() noexcept {}

    GridHolder(Grid<T>&& grid, GridDataPtr<T>&& ptr) noexcept {
        std::swap(this->grid, grid);
        std::swap(this->ptr, ptr);
    }

    GridHolder(GridHolder&& other) noexcept
        : GridHolder(std::move(other.grid), std::move(other.ptr)) {}

    GridHolder(const GridHolder& other) = delete;

    GridHolder& operator=(GridHolder&& other) & {
        std::swap(grid, other.grid);
        std::swap(ptr, other.ptr);
        return *this;
    }

    GridHolder& operator=(const GridHolder& other) = delete;

    Grid<T>& operator*() noexcept { return grid; }
    const Grid<T>& operator*() const noexcept { return grid; }

    Grid<T>* operator->() { return &grid; }
    const Grid<T>* operator->() const { return &grid; }

    Grid<T> grid = {};
    GridDataPtr<T> ptr = nullptr;
};

using U8GridHolder = GridHolder<uint8_t>;
using S32GridHolder = GridHolder<int32_t>;
using U32GridHolder = GridHolder<uint32_t>;
using F32GridHolder = GridHolder<float>;

template<typename T, typename... Args>
GridHolder<T> MakeGrid(Args&&... args) {
    auto grid = Grid<T>(std::forward<Args>(args)...);
    auto ptr = Allocate<T>(grid.size);
    grid.Reset(ptr.get());
    return {std::move(grid), std::move(ptr)};
}

template<typename T, typename U>
GridHolder<T> MakeGridLike(const Grid<U>& other) noexcept {
    GridHolder<T> holder(MakeGrid<T>(other.size, other.resolution, other.origin));
    std::copy(other.data, other.data + other.size(), holder.ptr.get());
    return holder;
}

template<typename T, typename U>
GridHolder<T> MakeGridLike(const GridHolder<U>& other) noexcept {
    return MakeGridLike<T>(other.grid);
}

}  // namespace truck::fastgrid