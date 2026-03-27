/**
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * @file wheels.hpp
 *
 * @brief Defining the wheels for the vcu_shared_lib
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
#pragma once

#ifndef vcu_shared_lib_WHEELS_HPP
#define vcu_shared_lib_WHEELS_HPP

#include "ix_msgs/msg/wheels.hpp"
#include <algorithm>

template <typename T> struct Wheels
{
    T fl, fr, rl, rr;
    Wheels();
    Wheels(const ix_msgs::msg::Wheels msg);
    Wheels(T fl, T fr, T rl, T rr);

    T sum() const;
    T avg() const;
    T min() const;
    T max() const;
    bool isPositiveOrZero() const;
    bool isNegativeOrZero() const;
    bool isZero() const;

    void clamp(const Wheels<T>& lower, const Wheels<T>& upper);
    void clamp(T lower, T upper);
    void scale(T factor);
    void div(T factor);
    void add(const Wheels<T>& summand);
    void add(T summand);
    void sub(const Wheels<T>& subtrahend);
    void sub(T subtrahend);

    Wheels& operator*=(const T& rhs);
    Wheels& operator/=(const T& rhs);
    Wheels& operator+=(const Wheels& rhs);
    Wheels& operator+=(const T& rhs);
    Wheels& operator-=(const Wheels& rhs);
    Wheels& operator-=(const T& rhs);

    friend Wheels operator*(Wheels lhs, const T& rhs)
    {
        lhs *= rhs;
        return lhs;
    }
    friend Wheels operator/(Wheels lhs, const T& rhs)
    {
        lhs /= rhs;
        return lhs;
    }
    friend Wheels operator+(Wheels lhs, const Wheels& rhs)
    {
        lhs += rhs;
        return lhs;
    }
    friend Wheels operator+(Wheels lhs, const T& rhs)
    {
        lhs += rhs;
        return lhs;
    }
    friend Wheels operator-(Wheels lhs, const Wheels& rhs)
    {
        lhs -= rhs;
        return lhs;
    }
    friend Wheels operator-(Wheels lhs, const T& rhs)
    {
        lhs -= rhs;
        return lhs;
    }

    ix_msgs::msg::Wheels to_msg() const;
};

// equality operators
template <typename T> bool operator==(const Wheels<T>& lhs, const Wheels<T>& rhs);
template <typename T> bool operator!=(const Wheels<T>& lhs, const Wheels<T>& rhs);

// explicit instantiations

#define INSTANTIATE_WHEELS(T)                                                                                          \
    extern template struct Wheels<T>;                                                                                  \
    extern template bool operator== <T>(const Wheels<T>&, const Wheels<T>&);                                           \
    extern template bool operator!= <T>(const Wheels<T>&, const Wheels<T>&);

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

#endif
