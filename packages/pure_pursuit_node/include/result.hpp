#pragma once

#include <type_traits>
#include <variant>
#include <utility>

namespace pure_pursuit {

template<class Value, class Error>
class Result {
    static_assert(!std::is_same_v<Value, Error>, "Value type can not match error type");
private:
    std::variant<Value, Error> state;
public:
    Result(Value value): state(std::move(value)) {}
    Result(Error error): state(std::move(error)) {}

    operator bool() const {
        return state.index() == 0;
    }

    const auto& get_value() const& {
        return std::get<0>(state);
    }
    auto get_value() && {
        return std::move(std::get<0>(state));
    }

    const auto& operator*() const {
        return get_value();
    }
    const auto *operator->() const {
        return &get_value();
    }

    const auto& get_error() const& {
        return std::get<1>(state);
    }

    auto get_error() && {
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