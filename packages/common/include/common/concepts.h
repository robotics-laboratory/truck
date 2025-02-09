#pragma once
#include <type_traits>

namespace truck {
namespace {
template<class T, class U>
concept SameHelper = std::is_same_v<T, U>;
}

template<class T, class U>
concept same_as = SameHelper<T, U> && SameHelper<U, T>;
}  // namespace truck