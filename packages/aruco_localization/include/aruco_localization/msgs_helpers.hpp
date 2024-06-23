#pragma once

namespace rosaruco {

template<typename V, typename M>
void setPoint(const V& v, M& msg) {
    msg.x = v[0];
    msg.y = v[1];
    msg.z = v[2];
}

}  // namespace rosaruco
