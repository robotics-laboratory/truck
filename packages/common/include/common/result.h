#pragma once

#include <utility>
#include <variant>

namespace truck::common {

template<class Value, class Error>
class Result {
    static_assert(!std::is_same_v<Value, Error>, "Value type can not match error type");

  private:
    std::variant<std::monostate, Value, Error> state;

  public:
    Result() : state(std::in_place_index_t<0>{}) {}
    Result(Value value) : state(std::move(value)) {}
    Result(Error error) : state(std::move(error)) {}

    operator bool() const { return state.index() == 1; }

    bool empty() const { return state.index() == 0; }

    const auto& get() const& { return std::get<1>(state); }
    auto get() && { return std::move(std::get<1>(state)); }

    const auto& operator*() const { return get(); }
    const auto* operator->() const { return &get(); }

    const auto& error() const& { return std::get<2>(state); }

    auto error() && { return std::move(std::get<2>(state)); }

    template <class... Args>
    static Result ok(Args&&... args) {
        return Value(std::forward<Args>(args)...);
    }
    template <class... Args>
    static Result fail(Args&&... args) {
        return Error(std::forward<Args>(args)...);
    }
};

}  // namespace truck::common