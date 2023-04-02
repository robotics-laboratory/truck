#include "common/format.h"

#include <cstdarg>
#include <cstdio>

namespace truck {

std::string fmt(const char* s, ...) {
    va_list args;
    va_start(args, s);

    char buffer[4096];
    vsnprintf(buffer, sizeof(buffer), s, args);

    va_end(args);

    return std::string(buffer);
}

}  // namespace truck