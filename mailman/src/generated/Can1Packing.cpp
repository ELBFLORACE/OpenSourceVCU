/**
 * @file Can1
 *
 * @copyright Copyright (c) 2018-2019 Erik Moqvist
 *
 * @par License
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <errno.h>

#include "generated/Can1.hpp"



static inline uint8_t pack_left_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_right_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint16_t unpack_left_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) << shift);
}

static inline uint16_t unpack_right_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) >> shift);
}

int can1_0x12_bps_ebs_pressure_t::pack(
    uint8_t *dst_p,
    // const struct can1_0x12_bps_ebs_pressure_t *src_p,
    size_t size)
{
    auto src_p = this;
    if (size < 6u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 6);

    dst_p[0] |= pack_right_shift_u16(src_p->ebs_pressure1, 4u, 0xffu);
    dst_p[1] |= pack_left_shift_u16(src_p->ebs_pressure1, 4u, 0xf0u);
    dst_p[1] |= pack_right_shift_u16(src_p->ebs_pressure2, 8u, 0x0fu);
    dst_p[2] |= pack_left_shift_u16(src_p->ebs_pressure2, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(src_p->brake_pressure_front, 4u, 0xffu);
    dst_p[4] |= pack_left_shift_u16(src_p->brake_pressure_front, 4u, 0xf0u);
    dst_p[4] |= pack_right_shift_u16(src_p->brake_pressure_rear, 8u, 0x0fu);
    dst_p[5] |= pack_left_shift_u16(src_p->brake_pressure_rear, 0u, 0xffu);

    return (6);
}

int can1_0x12_bps_ebs_pressure_t::unpack(
    struct can1_0x12_bps_ebs_pressure_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 6u) {
        return (-EINVAL);
    }

    dst_p->ebs_pressure1 = unpack_left_shift_u16(src_p[0], 4u, 0xffu);
    dst_p->ebs_pressure1 |= unpack_right_shift_u16(src_p[1], 4u, 0xf0u);
    dst_p->ebs_pressure2 = unpack_left_shift_u16(src_p[1], 8u, 0x0fu);
    dst_p->ebs_pressure2 |= unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    dst_p->brake_pressure_front = unpack_left_shift_u16(src_p[3], 4u, 0xffu);
    dst_p->brake_pressure_front |= unpack_right_shift_u16(src_p[4], 4u, 0xf0u);
    dst_p->brake_pressure_rear = unpack_left_shift_u16(src_p[4], 8u, 0x0fu);
    dst_p->brake_pressure_rear |= unpack_right_shift_u16(src_p[5], 0u, 0xffu);

    return (0);
}

int can1_0x12_bps_ebs_pressure_init(struct can1_0x12_bps_ebs_pressure_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct can1_0x12_bps_ebs_pressure_t));
    msg_p->brake_pressure_front = 312;
    msg_p->brake_pressure_rear = 312;

    return 0;
}

uint16_t Can1_0x12_bps_ebs_pressure_ebs_pressure1_encode(float value)
{
    return (uint16_t)(value / 0.025f);
}

float Can1_0x12_bps_ebs_pressure_ebs_pressure1_decode(uint16_t value)
{
    return ((float)value * 0.025f);
}

bool Can1_0x12_bps_ebs_pressure_ebs_pressure1_is_in_range(uint16_t value)
{
    return (value <= 4095u);
}

uint16_t Can1_0x12_bps_ebs_pressure_ebs_pressure2_encode(float value)
{
    return (uint16_t)(value / 0.025f);
}

float Can1_0x12_bps_ebs_pressure_ebs_pressure2_decode(uint16_t value)
{
    return ((float)value * 0.025f);
}

bool Can1_0x12_bps_ebs_pressure_ebs_pressure2_is_in_range(uint16_t value)
{
    return (value <= 4095u);
}

uint16_t Can1_0x12_bps_ebs_pressure_brake_pressure_front_encode(float value)
{
    return (uint16_t)(value / 0.05f);
}

float Can1_0x12_bps_ebs_pressure_brake_pressure_front_decode(uint16_t value)
{
    return ((float)value * 0.05f);
}

bool Can1_0x12_bps_ebs_pressure_brake_pressure_front_is_in_range(uint16_t value)
{
    return (value <= 4095u);
}

uint16_t Can1_0x12_bps_ebs_pressure_brake_pressure_rear_encode(float value)
{
    return (uint16_t)(value / 0.05f);
}

float Can1_0x12_bps_ebs_pressure_brake_pressure_rear_decode(uint16_t value)
{
    return ((float)value * 0.05f);
}

bool Can1_0x12_bps_ebs_pressure_brake_pressure_rear_is_in_range(uint16_t value)
{
    return (value <= 4095u);
}

int can1_0x11_apps_bse_steering_t::pack(
    uint8_t *dst_p,
    // const struct can1_0x11_apps_bse_steering_t *src_p,
    size_t size)
{
    auto src_p = this;
    if (size < 7u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 7);

    dst_p[0] |= pack_right_shift_u16(src_p->brakeforce, 4u, 0xffu);
    dst_p[1] |= pack_left_shift_u16(src_p->brakeforce, 4u, 0xf0u);
    dst_p[1] |= pack_right_shift_u16(src_p->steering_angle, 8u, 0x0fu);
    dst_p[2] |= pack_left_shift_u16(src_p->steering_angle, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(src_p->apps_left, 4u, 0xffu);
    dst_p[4] |= pack_left_shift_u16(src_p->apps_left, 4u, 0xf0u);
    dst_p[4] |= pack_right_shift_u16(src_p->apps_right, 8u, 0x0fu);
    dst_p[5] |= pack_left_shift_u16(src_p->apps_right, 0u, 0xffu);

    return (7);
}

int can1_0x11_apps_bse_steering_t::unpack(
    struct can1_0x11_apps_bse_steering_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 7u) {
        return (-EINVAL);
    }

    dst_p->brakeforce = unpack_left_shift_u16(src_p[0], 4u, 0xffu);
    dst_p->brakeforce |= unpack_right_shift_u16(src_p[1], 4u, 0xf0u);
    dst_p->steering_angle = unpack_left_shift_u16(src_p[1], 8u, 0x0fu);
    dst_p->steering_angle |= unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    dst_p->apps_left = unpack_left_shift_u16(src_p[3], 4u, 0xffu);
    dst_p->apps_left |= unpack_right_shift_u16(src_p[4], 4u, 0xf0u);
    dst_p->apps_right = unpack_left_shift_u16(src_p[4], 8u, 0x0fu);
    dst_p->apps_right |= unpack_right_shift_u16(src_p[5], 0u, 0xffu);

    return (0);
}

int can1_0x11_apps_bse_steering_init(struct can1_0x11_apps_bse_steering_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct can1_0x11_apps_bse_steering_t));
    msg_p->steering_angle = 2000;

    return 0;
}

uint16_t Can1_0x11_apps_bse_steering_brakeforce_encode(float value)
{
    return (uint16_t)(value / 0.5f);
}

float Can1_0x11_apps_bse_steering_brakeforce_decode(uint16_t value)
{
    return ((float)value * 0.5f);
}

bool Can1_0x11_apps_bse_steering_brakeforce_is_in_range(uint16_t value)
{
    return (value <= 4096u);
}

uint16_t Can1_0x11_apps_bse_steering_steering_angle_encode(float value)
{
    return (uint16_t)(value / 0.09f);
}

float Can1_0x11_apps_bse_steering_steering_angle_decode(uint16_t value)
{
    return ((float)value * 0.09f);
}

bool Can1_0x11_apps_bse_steering_steering_angle_is_in_range(uint16_t value)
{
    return (value <= 4096u);
}

uint16_t Can1_0x11_apps_bse_steering_apps_left_encode(float value)
{
    return (uint16_t)(value / 0.05f);
}

float Can1_0x11_apps_bse_steering_apps_left_decode(uint16_t value)
{
    return ((float)value * 0.05f);
}

bool Can1_0x11_apps_bse_steering_apps_left_is_in_range(uint16_t value)
{
    return (value <= 3686u);
}

uint16_t Can1_0x11_apps_bse_steering_apps_right_encode(float value)
{
    return (uint16_t)(value / 0.05f);
}

float Can1_0x11_apps_bse_steering_apps_right_decode(uint16_t value)
{
    return ((float)value * 0.05f);
}

bool Can1_0x11_apps_bse_steering_apps_right_is_in_range(uint16_t value)
{
    return (value <= 3686u);
}
