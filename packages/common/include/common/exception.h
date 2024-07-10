#pragma once

#include <exception>
#include <sstream>
#include <utility>

#include "common/format.h"

namespace truck {

class Exception : public std::exception {
  public:
    Exception() = default;

    Exception(std::string message) : message_(std::move(message)) {}

    Exception(const char* message) : message_(message) {}

    template<typename... Args>
    Exception(const char* f, const Args&... args) : message_(fmt(f, args...)) {}

    template<typename T>
    Exception& operator<<(const T& value) {
        std::ostringstream stream;
        stream << value;
        message_ += stream.str();
        return *this;
    }

    const char* what() const noexcept override { return message_.c_str(); }

  private:
    std::string message_;
};

#define THROW_EXCEPTION() throw Exception() << __FILE__ << ":" << __LINE__

#define VERIFY(expr)                             \
    [&](auto&& arg) -> decltype(arg) {           \
        if (!static_cast<bool>(arg)) {           \
            THROW_EXCEPTION() << ": '" << #expr; \
        }                                        \
        return std::forward<decltype(arg)>(arg); \
    }(expr)

#define VERIFY_STREAM(condition, messages)                           \
    if (!(condition)) {                                              \
        THROW_EXCEPTION() << ": " << #condition << "\n" << messages; \
    }

#define VERIFY_FMT(condition, ...)                                          \
    if (!(condition)) {                                                     \
        THROW_EXCEPTION() << ":" << #condition << "\n" << fmt(__VA_ARGS__); \
    }

#define FALL(...) THROW_EXCEPTION() << ": " << fmt(__VA_ARGS__);

}  // namespace truck
