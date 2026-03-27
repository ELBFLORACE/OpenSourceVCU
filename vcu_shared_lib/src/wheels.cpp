/**
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * @file wheels.cpp
 *
 * @brief Implementation of the wheels operations for the vcu_shared_lib
 *
 * @author Marvin Jacob <marvin.jacob@elbflorace.de> 2025-2026
 * @author Laurin Hesslich <laurin.hesslich@elbflorace.de> 2026
 *
 * @copyright MIT
 *
 * @version 1.0.0
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */
#include "vcu_shared_lib/wheels.hpp"
#include <cstdint>

template <typename T>
Wheels<T>::Wheels()
    : fl {}
    , fr {}
    , rl {}
    , rr {}
{
}

template <typename T>
Wheels<T>::Wheels(const ix_msgs::msg::Wheels msg)
    : fl(static_cast<T>(msg.fl))
    , fr(static_cast<T>(msg.fr))
    , rl(static_cast<T>(msg.rl))
    , rr(static_cast<T>(msg.rr))
{
}

template <typename T>
Wheels<T>::Wheels(T fl_, T fr_, T rl_, T rr_)
    : fl(fl_)
    , fr(fr_)
    , rl(rl_)
    , rr(rr_)
{
}

template <typename T> T Wheels<T>::sum() const { return fl + fr + rl + rr; }

template <typename T> T Wheels<T>::avg() const { return sum() / static_cast<T>(4); }

template <typename T> T Wheels<T>::min() const { return std::min({ fl, fr, rl, rr }); }

template <typename T> T Wheels<T>::max() const { return std::max({ fl, fr, rl, rr }); }

template <typename T> bool Wheels<T>::isPositiveOrZero() const { return min() >= static_cast<T>(0); }

template <typename T> bool Wheels<T>::isNegativeOrZero() const { return max() <= static_cast<T>(0); }

template <typename T> bool Wheels<T>::isZero() const
{
    auto z = static_cast<T>(0);
    return min() == z && max() == z;
}

template <typename T> void Wheels<T>::clamp(const Wheels<T>& lower, const Wheels<T>& upper)
{
    fl = std::clamp(fl, lower.fl, upper.fl);
    fr = std::clamp(fr, lower.fr, upper.fr);
    rl = std::clamp(rl, lower.rl, upper.rl);
    rr = std::clamp(rr, lower.rr, upper.rr);
}

template <typename T> void Wheels<T>::clamp(T lower, T upper)
{
    fl = std::clamp(fl, lower, upper);
    fr = std::clamp(fr, lower, upper);
    rl = std::clamp(rl, lower, upper);
    rr = std::clamp(rr, lower, upper);
}

template <typename T> void Wheels<T>::scale(T factor)
{
    fl *= factor;
    fr *= factor;
    rl *= factor;
    rr *= factor;
}

template <typename T> void Wheels<T>::div(T factor)
{
    fl /= factor;
    fr /= factor;
    rl /= factor;
    rr /= factor;
}

template <typename T> void Wheels<T>::add(const Wheels<T>& s)
{
    fl += s.fl;
    fr += s.fr;
    rl += s.rl;
    rr += s.rr;
}

template <typename T> void Wheels<T>::add(T s)
{
    fl += s;
    fr += s;
    rl += s;
    rr += s;
}

template <typename T> void Wheels<T>::sub(const Wheels<T>& s)
{
    fl -= s.fl;
    fr -= s.fr;
    rl -= s.rl;
    rr -= s.rr;
}

template <typename T> void Wheels<T>::sub(T s)
{
    fl -= s;
    fr -= s;
    rl -= s;
    rr -= s;
}

template <typename T> Wheels<T>& Wheels<T>::operator*=(const T& rhs)
{
    scale(rhs);
    return *this;
}

template <typename T> Wheels<T>& Wheels<T>::operator/=(const T& rhs)
{
    div(rhs);
    return *this;
}

template <typename T> Wheels<T>& Wheels<T>::operator+=(const Wheels& rhs)
{
    add(rhs);
    return *this;
}

template <typename T> Wheels<T>& Wheels<T>::operator+=(const T& rhs)
{
    add(rhs);
    return *this;
}

template <typename T> Wheels<T>& Wheels<T>::operator-=(const Wheels& rhs)
{
    sub(rhs);
    return *this;
}

template <typename T> Wheels<T>& Wheels<T>::operator-=(const T& rhs)
{
    sub(rhs);
    return *this;
}

template <typename T> ix_msgs::msg::Wheels Wheels<T>::to_msg() const
{
    ix_msgs::msg::Wheels m;
    m.fl = static_cast<float>(fl);
    m.fr = static_cast<float>(fr);
    m.rl = static_cast<float>(rl);
    m.rr = static_cast<float>(rr);
    return m;
}

template <typename T> bool operator==(const Wheels<T>& lhs, const Wheels<T>& rhs)
{
    return (lhs.fr == rhs.fr) && (lhs.fl == rhs.fl) && (lhs.rr == rhs.rr) && (lhs.rl == rhs.rl);
}

template <typename T> bool operator!=(const Wheels<T>& lhs, const Wheels<T>& rhs) { return !(lhs == rhs); }

// explicit instantiations to save compile time for most common types
#define INSTANTIATE_WHEELS(T)                                                                                          \
    template struct Wheels<T>;                                                                                         \
    template bool operator== <T>(const Wheels<T>&, const Wheels<T>&);                                                  \
    template bool operator!= <T>(const Wheels<T>&, const Wheels<T>&);

INSTANTIATE_WHEELS(float)
INSTANTIATE_WHEELS(double)

INSTANTIATE_WHEELS(bool)

INSTANTIATE_WHEELS(int8_t)
INSTANTIATE_WHEELS(uint8_t)

INSTANTIATE_WHEELS(int16_t)
INSTANTIATE_WHEELS(uint16_t)

INSTANTIATE_WHEELS(int32_t)
INSTANTIATE_WHEELS(uint32_t)

INSTANTIATE_WHEELS(int64_t)
INSTANTIATE_WHEELS(uint64_t)

#undef INSTANTIATE_WHEELS
