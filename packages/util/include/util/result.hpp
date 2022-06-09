#pragma once

#include <type_traits>
#include <variant>
#include <utility>

namespace util {

template<class Value, class Error>
class Result {
    static_assert(!std::is_same_v<Value, Error>, "Value type can not match error type");
private:
    std::variant<Value, Error, std::monostate> state;
public:
    Result(): state(std::in_place_index_t<2>{}) {}
    Result(Value value): state(std::move(value)) {}
    Result(Error error): state(std::move(error)) {}

    operator bool() const {
        return state.index() == 0;
    }

    bool empty() const {
        return state.index() == 2;
    }

    const auto& get() const& {
        return std::get<0>(state);
    }
    auto get() && {
        return std::move(std::get<0>(state));
    }

    const auto& operator*() const {
        return get();
    }
    const auto *operator->() const {
        return &get();
    }

    const auto& error() const& {
        return std::get<1>(state);
    }

    auto error() && {
        return std::move(std::get<1>(state));
    }

    template<class... Args>
    static Result ok(Args&&... args) {
        return Value(std::forward<Args>(args)...);
    }
    template<class... Args>
    static Result fail(Args&&... args) {
        return Error(std::forward<Args>(args)...);
    }
};

};