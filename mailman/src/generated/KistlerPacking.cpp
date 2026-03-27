/**
 * @file Kistler
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

#include "generated/Kistler.hpp"



static inline uint8_t pack_left_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_left_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value << shift) & mask);
}

static inline uint8_t pack_left_shift_u32(
    uint32_t value,
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

static inline uint8_t pack_right_shift_u32(
    uint32_t value,
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

static inline uint32_t unpack_left_shift_u32(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint32_t)((uint32_t)(value & mask) << shift);
}

static inline uint8_t unpack_right_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value & mask) >> shift);
}

static inline uint16_t unpack_right_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) >> shift);
}

static inline uint32_t unpack_right_shift_u32(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint32_t)((uint32_t)(value & mask) >> shift);
}

int kistler_0x7e0_vel_angle_t::pack(
    uint8_t *dst_p,
    // const struct kistler_0x7e0_vel_angle_t *src_p,
    size_t size)
{
    auto src_p = this;
    uint16_t angle;
    uint16_t vel;
    uint16_t vel_x;
    uint16_t vel_y;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    vel_x = (uint16_t)src_p->vel_x;
    dst_p[0] |= pack_right_shift_u16(vel_x, 8u, 0xffu);
    dst_p[1] |= pack_left_shift_u16(vel_x, 0u, 0xffu);
    vel_y = (uint16_t)src_p->vel_y;
    dst_p[2] |= pack_right_shift_u16(vel_y, 8u, 0xffu);
    dst_p[3] |= pack_left_shift_u16(vel_y, 0u, 0xffu);
    vel = (uint16_t)src_p->vel;
    dst_p[4] |= pack_right_shift_u16(vel, 8u, 0xffu);
    dst_p[5] |= pack_left_shift_u16(vel, 0u, 0xffu);
    angle = (uint16_t)src_p->angle;
    dst_p[6] |= pack_right_shift_u16(angle, 8u, 0xffu);
    dst_p[7] |= pack_left_shift_u16(angle, 0u, 0xffu);

    return (8);
}

int kistler_0x7e0_vel_angle_t::unpack(
    struct kistler_0x7e0_vel_angle_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t angle;
    uint16_t vel;
    uint16_t vel_x;
    uint16_t vel_y;

    if (size < 8u) {
        return (-EINVAL);
    }

    vel_x = unpack_left_shift_u16(src_p[0], 8u, 0xffu);
    vel_x |= unpack_right_shift_u16(src_p[1], 0u, 0xffu);
    dst_p->vel_x = (int16_t)vel_x;
    vel_y = unpack_left_shift_u16(src_p[2], 8u, 0xffu);
    vel_y |= unpack_right_shift_u16(src_p[3], 0u, 0xffu);
    dst_p->vel_y = (int16_t)vel_y;
    vel = unpack_left_shift_u16(src_p[4], 8u, 0xffu);
    vel |= unpack_right_shift_u16(src_p[5], 0u, 0xffu);
    dst_p->vel = (int16_t)vel;
    angle = unpack_left_shift_u16(src_p[6], 8u, 0xffu);
    angle |= unpack_right_shift_u16(src_p[7], 0u, 0xffu);
    dst_p->angle = (int16_t)angle;

    return (0);
}

int kistler_0x7e0_vel_angle_init(struct kistler_0x7e0_vel_angle_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kistler_0x7e0_vel_angle_t));

    return 0;
}

int16_t Kistler_0x7e0_vel_angle_vel_x_encode(float value)
{
    return (int16_t)(value / 0.036f);
}

float Kistler_0x7e0_vel_angle_vel_x_decode(int16_t value)
{
    return ((float)value * 0.036f);
}

bool Kistler_0x7e0_vel_angle_vel_x_is_in_range(int16_t value)
{
    return ((value >= -11111) && (value <= 11111));
}

int16_t Kistler_0x7e0_vel_angle_vel_y_encode(float value)
{
    return (int16_t)(value / 0.036f);
}

float Kistler_0x7e0_vel_angle_vel_y_decode(int16_t value)
{
    return ((float)value * 0.036f);
}

bool Kistler_0x7e0_vel_angle_vel_y_is_in_range(int16_t value)
{
    return ((value >= -2778) && (value <= 2778));
}

int16_t Kistler_0x7e0_vel_angle_vel_encode(float value)
{
    return (int16_t)(value / 0.036f);
}

float Kistler_0x7e0_vel_angle_vel_decode(int16_t value)
{
    return ((float)value * 0.036f);
}

bool Kistler_0x7e0_vel_angle_vel_is_in_range(int16_t value)
{
    return ((value >= -11111) && (value <= 11111));
}

int16_t Kistler_0x7e0_vel_angle_angle_encode(float value)
{
    return (int16_t)(value / 0.01f);
}

float Kistler_0x7e0_vel_angle_angle_decode(int16_t value)
{
    return ((float)value * 0.01f);
}

bool Kistler_0x7e0_vel_angle_angle_is_in_range(int16_t value)
{
    return ((value >= -4500) && (value <= 4500));
}

int kistler_0x7e1_distance_t::pack(
    uint8_t *dst_p,
    // const struct kistler_0x7e1_distance_t *src_p,
    size_t size)
{
    auto src_p = this;
    uint32_t distance;

    if (size < 4u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 4);

    distance = (uint32_t)src_p->distance;
    dst_p[0] |= pack_right_shift_u32(distance, 24u, 0xffu);
    dst_p[1] |= pack_right_shift_u32(distance, 16u, 0xffu);
    dst_p[2] |= pack_right_shift_u32(distance, 8u, 0xffu);
    dst_p[3] |= pack_left_shift_u32(distance, 0u, 0xffu);

    return (4);
}

int kistler_0x7e1_distance_t::unpack(
    struct kistler_0x7e1_distance_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint32_t distance;

    if (size < 4u) {
        return (-EINVAL);
    }

    distance = unpack_left_shift_u32(src_p[0], 24u, 0xffu);
    distance |= unpack_left_shift_u32(src_p[1], 16u, 0xffu);
    distance |= unpack_left_shift_u32(src_p[2], 8u, 0xffu);
    distance |= unpack_right_shift_u32(src_p[3], 0u, 0xffu);
    dst_p->distance = (int32_t)distance;

    return (0);
}

int kistler_0x7e1_distance_init(struct kistler_0x7e1_distance_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kistler_0x7e1_distance_t));

    return 0;
}

int32_t Kistler_0x7e1_distance_distance_encode(float value)
{
    return (int32_t)(value / 0.001f);
}

float Kistler_0x7e1_distance_distance_decode(int32_t value)
{
    return ((float)value * 0.001f);
}

bool Kistler_0x7e1_distance_distance_is_in_range(int32_t value)
{
    return ((value >= -2147480000) && (value <= 2147480000));
}

int kistler_0x7e2_pitch_roll_t::pack(
    uint8_t *dst_p,
    // const struct kistler_0x7e2_pitch_roll_t *src_p,
    size_t size)
{
    auto src_p = this;
    uint16_t pitch;
    uint16_t roll;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    pitch = (uint16_t)src_p->pitch;
    dst_p[0] |= pack_right_shift_u16(pitch, 8u, 0xffu);
    dst_p[1] |= pack_left_shift_u16(pitch, 0u, 0xffu);
    roll = (uint16_t)src_p->roll;
    dst_p[2] |= pack_right_shift_u16(roll, 8u, 0xffu);
    dst_p[3] |= pack_left_shift_u16(roll, 0u, 0xffu);
    dst_p[4] |= pack_right_shift_u16(src_p->timestamp, 8u, 0xffu);
    dst_p[5] |= pack_left_shift_u16(src_p->timestamp, 0u, 0xffu);
    dst_p[6] |= pack_right_shift_u16(src_p->radius, 8u, 0xffu);
    dst_p[7] |= pack_left_shift_u16(src_p->radius, 0u, 0xffu);

    return (8);
}

int kistler_0x7e2_pitch_roll_t::unpack(
    struct kistler_0x7e2_pitch_roll_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t pitch;
    uint16_t roll;

    if (size < 8u) {
        return (-EINVAL);
    }

    pitch = unpack_left_shift_u16(src_p[0], 8u, 0xffu);
    pitch |= unpack_right_shift_u16(src_p[1], 0u, 0xffu);
    dst_p->pitch = (int16_t)pitch;
    roll = unpack_left_shift_u16(src_p[2], 8u, 0xffu);
    roll |= unpack_right_shift_u16(src_p[3], 0u, 0xffu);
    dst_p->roll = (int16_t)roll;
    dst_p->timestamp = unpack_left_shift_u16(src_p[4], 8u, 0xffu);
    dst_p->timestamp |= unpack_right_shift_u16(src_p[5], 0u, 0xffu);
    dst_p->radius = unpack_left_shift_u16(src_p[6], 8u, 0xffu);
    dst_p->radius |= unpack_right_shift_u16(src_p[7], 0u, 0xffu);

    return (0);
}

int kistler_0x7e2_pitch_roll_init(struct kistler_0x7e2_pitch_roll_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kistler_0x7e2_pitch_roll_t));

    return 0;
}

int16_t Kistler_0x7e2_pitch_roll_pitch_encode(float value)
{
    return (int16_t)(value / 0.01f);
}

float Kistler_0x7e2_pitch_roll_pitch_decode(int16_t value)
{
    return ((float)value * 0.01f);
}

bool Kistler_0x7e2_pitch_roll_pitch_is_in_range(int16_t value)
{
    return ((value >= -4500) && (value <= 4500));
}

int16_t Kistler_0x7e2_pitch_roll_roll_encode(float value)
{
    return (int16_t)(value / 0.01f);
}

float Kistler_0x7e2_pitch_roll_roll_decode(int16_t value)
{
    return ((float)value * 0.01f);
}

bool Kistler_0x7e2_pitch_roll_roll_is_in_range(int16_t value)
{
    return ((value >= -4500) && (value <= 4500));
}

uint16_t Kistler_0x7e2_pitch_roll_timestamp_encode(float value)
{
    return (uint16_t)(value);
}

float Kistler_0x7e2_pitch_roll_timestamp_decode(uint16_t value)
{
    return ((float)value);
}

bool Kistler_0x7e2_pitch_roll_timestamp_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

uint16_t Kistler_0x7e2_pitch_roll_radius_encode(float value)
{
    return (uint16_t)(value / 0.01f);
}

float Kistler_0x7e2_pitch_roll_radius_decode(uint16_t value)
{
    return ((float)value * 0.01f);
}

bool Kistler_0x7e2_pitch_roll_radius_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

int kistler_0x7e3_acc_hor_acc_c_body_t::pack(
    uint8_t *dst_p,
    // const struct kistler_0x7e3_acc_hor_acc_c_body_t *src_p,
    size_t size)
{
    auto src_p = this;
    uint16_t acc_c_body;
    uint16_t acc_x_hor;
    uint16_t acc_y_hor;
    uint16_t acc_z_hor;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    acc_x_hor = (uint16_t)src_p->acc_x_hor;
    dst_p[0] |= pack_right_shift_u16(acc_x_hor, 8u, 0xffu);
    dst_p[1] |= pack_left_shift_u16(acc_x_hor, 0u, 0xffu);
    acc_y_hor = (uint16_t)src_p->acc_y_hor;
    dst_p[2] |= pack_right_shift_u16(acc_y_hor, 8u, 0xffu);
    dst_p[3] |= pack_left_shift_u16(acc_y_hor, 0u, 0xffu);
    acc_z_hor = (uint16_t)src_p->acc_z_hor;
    dst_p[4] |= pack_right_shift_u16(acc_z_hor, 8u, 0xffu);
    dst_p[5] |= pack_left_shift_u16(acc_z_hor, 0u, 0xffu);
    acc_c_body = (uint16_t)src_p->acc_c_body;
    dst_p[6] |= pack_right_shift_u16(acc_c_body, 8u, 0xffu);
    dst_p[7] |= pack_left_shift_u16(acc_c_body, 0u, 0xffu);

    return (8);
}

int kistler_0x7e3_acc_hor_acc_c_body_t::unpack(
    struct kistler_0x7e3_acc_hor_acc_c_body_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t acc_c_body;
    uint16_t acc_x_hor;
    uint16_t acc_y_hor;
    uint16_t acc_z_hor;

    if (size < 8u) {
        return (-EINVAL);
    }

    acc_x_hor = unpack_left_shift_u16(src_p[0], 8u, 0xffu);
    acc_x_hor |= unpack_right_shift_u16(src_p[1], 0u, 0xffu);
    dst_p->acc_x_hor = (int16_t)acc_x_hor;
    acc_y_hor = unpack_left_shift_u16(src_p[2], 8u, 0xffu);
    acc_y_hor |= unpack_right_shift_u16(src_p[3], 0u, 0xffu);
    dst_p->acc_y_hor = (int16_t)acc_y_hor;
    acc_z_hor = unpack_left_shift_u16(src_p[4], 8u, 0xffu);
    acc_z_hor |= unpack_right_shift_u16(src_p[5], 0u, 0xffu);
    dst_p->acc_z_hor = (int16_t)acc_z_hor;
    acc_c_body = unpack_left_shift_u16(src_p[6], 8u, 0xffu);
    acc_c_body |= unpack_right_shift_u16(src_p[7], 0u, 0xffu);
    dst_p->acc_c_body = (int16_t)acc_c_body;

    return (0);
}

int kistler_0x7e3_acc_hor_acc_c_body_init(struct kistler_0x7e3_acc_hor_acc_c_body_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kistler_0x7e3_acc_hor_acc_c_body_t));

    return 0;
}

int16_t Kistler_0x7e3_acc_hor_acc_c_body_acc_x_hor_encode(float value)
{
    return (int16_t)(value / 0.01f);
}

float Kistler_0x7e3_acc_hor_acc_c_body_acc_x_hor_decode(int16_t value)
{
    return ((float)value * 0.01f);
}

bool Kistler_0x7e3_acc_hor_acc_c_body_acc_x_hor_is_in_range(int16_t value)
{
    return ((value >= -18000) && (value <= 18000));
}

int16_t Kistler_0x7e3_acc_hor_acc_c_body_acc_y_hor_encode(float value)
{
    return (int16_t)(value / 0.01f);
}

float Kistler_0x7e3_acc_hor_acc_c_body_acc_y_hor_decode(int16_t value)
{
    return ((float)value * 0.01f);
}

bool Kistler_0x7e3_acc_hor_acc_c_body_acc_y_hor_is_in_range(int16_t value)
{
    return ((value >= -18000) && (value <= 18000));
}

int16_t Kistler_0x7e3_acc_hor_acc_c_body_acc_z_hor_encode(float value)
{
    return (int16_t)(value / 0.01f);
}

float Kistler_0x7e3_acc_hor_acc_c_body_acc_z_hor_decode(int16_t value)
{
    return ((float)value * 0.01f);
}

bool Kistler_0x7e3_acc_hor_acc_c_body_acc_z_hor_is_in_range(int16_t value)
{
    return ((value >= -18000) && (value <= 18000));
}

int16_t Kistler_0x7e3_acc_hor_acc_c_body_acc_c_body_encode(float value)
{
    return (int16_t)(value / 0.01f);
}

float Kistler_0x7e3_acc_hor_acc_c_body_acc_c_body_decode(int16_t value)
{
    return ((float)value * 0.01f);
}

bool Kistler_0x7e3_acc_hor_acc_c_body_acc_c_body_is_in_range(int16_t value)
{
    return ((value >= -18000) && (value <= 18000));
}

int kistler_0x7e4_ang_vel_hor_t::pack(
    uint8_t *dst_p,
    // const struct kistler_0x7e4_ang_vel_hor_t *src_p,
    size_t size)
{
    auto src_p = this;
    uint16_t ang_vel_x_hor;
    uint16_t ang_vel_y_hor;
    uint16_t ang_vel_z_hor;

    if (size < 6u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 6);

    ang_vel_x_hor = (uint16_t)src_p->ang_vel_x_hor;
    dst_p[0] |= pack_right_shift_u16(ang_vel_x_hor, 8u, 0xffu);
    dst_p[1] |= pack_left_shift_u16(ang_vel_x_hor, 0u, 0xffu);
    ang_vel_y_hor = (uint16_t)src_p->ang_vel_y_hor;
    dst_p[2] |= pack_right_shift_u16(ang_vel_y_hor, 8u, 0xffu);
    dst_p[3] |= pack_left_shift_u16(ang_vel_y_hor, 0u, 0xffu);
    ang_vel_z_hor = (uint16_t)src_p->ang_vel_z_hor;
    dst_p[4] |= pack_right_shift_u16(ang_vel_z_hor, 8u, 0xffu);
    dst_p[5] |= pack_left_shift_u16(ang_vel_z_hor, 0u, 0xffu);

    return (6);
}

int kistler_0x7e4_ang_vel_hor_t::unpack(
    struct kistler_0x7e4_ang_vel_hor_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t ang_vel_x_hor;
    uint16_t ang_vel_y_hor;
    uint16_t ang_vel_z_hor;

    if (size < 6u) {
        return (-EINVAL);
    }

    ang_vel_x_hor = unpack_left_shift_u16(src_p[0], 8u, 0xffu);
    ang_vel_x_hor |= unpack_right_shift_u16(src_p[1], 0u, 0xffu);
    dst_p->ang_vel_x_hor = (int16_t)ang_vel_x_hor;
    ang_vel_y_hor = unpack_left_shift_u16(src_p[2], 8u, 0xffu);
    ang_vel_y_hor |= unpack_right_shift_u16(src_p[3], 0u, 0xffu);
    dst_p->ang_vel_y_hor = (int16_t)ang_vel_y_hor;
    ang_vel_z_hor = unpack_left_shift_u16(src_p[4], 8u, 0xffu);
    ang_vel_z_hor |= unpack_right_shift_u16(src_p[5], 0u, 0xffu);
    dst_p->ang_vel_z_hor = (int16_t)ang_vel_z_hor;

    return (0);
}

int kistler_0x7e4_ang_vel_hor_init(struct kistler_0x7e4_ang_vel_hor_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kistler_0x7e4_ang_vel_hor_t));

    return 0;
}

int16_t Kistler_0x7e4_ang_vel_hor_ang_vel_x_hor_encode(float value)
{
    return (int16_t)(value / 0.02f);
}

float Kistler_0x7e4_ang_vel_hor_ang_vel_x_hor_decode(int16_t value)
{
    return ((float)value * 0.02f);
}

bool Kistler_0x7e4_ang_vel_hor_ang_vel_x_hor_is_in_range(int16_t value)
{
    return ((value >= -15000) && (value <= 15000));
}

int16_t Kistler_0x7e4_ang_vel_hor_ang_vel_y_hor_encode(float value)
{
    return (int16_t)(value / 0.02f);
}

float Kistler_0x7e4_ang_vel_hor_ang_vel_y_hor_decode(int16_t value)
{
    return ((float)value * 0.02f);
}

bool Kistler_0x7e4_ang_vel_hor_ang_vel_y_hor_is_in_range(int16_t value)
{
    return ((value >= -15000) && (value <= 15000));
}

int16_t Kistler_0x7e4_ang_vel_hor_ang_vel_z_hor_encode(float value)
{
    return (int16_t)(value / 0.02f);
}

float Kistler_0x7e4_ang_vel_hor_ang_vel_z_hor_decode(int16_t value)
{
    return ((float)value * 0.02f);
}

bool Kistler_0x7e4_ang_vel_hor_ang_vel_z_hor_is_in_range(int16_t value)
{
    return ((value >= -15000) && (value <= 15000));
}

int kistler_0x7e5_correvit_t::pack(
    uint8_t *dst_p,
    // const struct kistler_0x7e5_correvit_t *src_p,
    size_t size)
{
    auto src_p = this;
    uint16_t angle_cor;
    uint16_t vel_cor;
    uint16_t vel_x_cor;
    uint16_t vel_y_cor;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    vel_x_cor = (uint16_t)src_p->vel_x_cor;
    dst_p[0] |= pack_right_shift_u16(vel_x_cor, 8u, 0xffu);
    dst_p[1] |= pack_left_shift_u16(vel_x_cor, 0u, 0xffu);
    vel_y_cor = (uint16_t)src_p->vel_y_cor;
    dst_p[2] |= pack_right_shift_u16(vel_y_cor, 8u, 0xffu);
    dst_p[3] |= pack_left_shift_u16(vel_y_cor, 0u, 0xffu);
    vel_cor = (uint16_t)src_p->vel_cor;
    dst_p[4] |= pack_right_shift_u16(vel_cor, 8u, 0xffu);
    dst_p[5] |= pack_left_shift_u16(vel_cor, 0u, 0xffu);
    angle_cor = (uint16_t)src_p->angle_cor;
    dst_p[6] |= pack_right_shift_u16(angle_cor, 8u, 0xffu);
    dst_p[7] |= pack_left_shift_u16(angle_cor, 0u, 0xffu);

    return (8);
}

int kistler_0x7e5_correvit_t::unpack(
    struct kistler_0x7e5_correvit_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t angle_cor;
    uint16_t vel_cor;
    uint16_t vel_x_cor;
    uint16_t vel_y_cor;

    if (size < 8u) {
        return (-EINVAL);
    }

    vel_x_cor = unpack_left_shift_u16(src_p[0], 8u, 0xffu);
    vel_x_cor |= unpack_right_shift_u16(src_p[1], 0u, 0xffu);
    dst_p->vel_x_cor = (int16_t)vel_x_cor;
    vel_y_cor = unpack_left_shift_u16(src_p[2], 8u, 0xffu);
    vel_y_cor |= unpack_right_shift_u16(src_p[3], 0u, 0xffu);
    dst_p->vel_y_cor = (int16_t)vel_y_cor;
    vel_cor = unpack_left_shift_u16(src_p[4], 8u, 0xffu);
    vel_cor |= unpack_right_shift_u16(src_p[5], 0u, 0xffu);
    dst_p->vel_cor = (int16_t)vel_cor;
    angle_cor = unpack_left_shift_u16(src_p[6], 8u, 0xffu);
    angle_cor |= unpack_right_shift_u16(src_p[7], 0u, 0xffu);
    dst_p->angle_cor = (int16_t)angle_cor;

    return (0);
}

int kistler_0x7e5_correvit_init(struct kistler_0x7e5_correvit_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kistler_0x7e5_correvit_t));

    return 0;
}

int16_t Kistler_0x7e5_correvit_vel_x_cor_encode(float value)
{
    return (int16_t)(value / 0.036f);
}

float Kistler_0x7e5_correvit_vel_x_cor_decode(int16_t value)
{
    return ((float)value * 0.036f);
}

bool Kistler_0x7e5_correvit_vel_x_cor_is_in_range(int16_t value)
{
    return ((value >= -11111) && (value <= 11111));
}

int16_t Kistler_0x7e5_correvit_vel_y_cor_encode(float value)
{
    return (int16_t)(value / 0.036f);
}

float Kistler_0x7e5_correvit_vel_y_cor_decode(int16_t value)
{
    return ((float)value * 0.036f);
}

bool Kistler_0x7e5_correvit_vel_y_cor_is_in_range(int16_t value)
{
    return ((value >= -2778) && (value <= 2778));
}

int16_t Kistler_0x7e5_correvit_vel_cor_encode(float value)
{
    return (int16_t)(value / 0.036f);
}

float Kistler_0x7e5_correvit_vel_cor_decode(int16_t value)
{
    return ((float)value * 0.036f);
}

bool Kistler_0x7e5_correvit_vel_cor_is_in_range(int16_t value)
{
    return ((value >= -11111) && (value <= 11111));
}

int16_t Kistler_0x7e5_correvit_angle_cor_encode(float value)
{
    return (int16_t)(value / 0.01f);
}

float Kistler_0x7e5_correvit_angle_cor_decode(int16_t value)
{
    return ((float)value * 0.01f);
}

bool Kistler_0x7e5_correvit_angle_cor_is_in_range(int16_t value)
{
    return ((value >= -4500) && (value <= 4500));
}

int kistler_0x7e6_acc_body_t::pack(
    uint8_t *dst_p,
    // const struct kistler_0x7e6_acc_body_t *src_p,
    size_t size)
{
    auto src_p = this;
    uint16_t acc_x_body;
    uint16_t acc_y_body;
    uint16_t acc_z_body;

    if (size < 6u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 6);

    acc_x_body = (uint16_t)src_p->acc_x_body;
    dst_p[0] |= pack_right_shift_u16(acc_x_body, 8u, 0xffu);
    dst_p[1] |= pack_left_shift_u16(acc_x_body, 0u, 0xffu);
    acc_y_body = (uint16_t)src_p->acc_y_body;
    dst_p[2] |= pack_right_shift_u16(acc_y_body, 8u, 0xffu);
    dst_p[3] |= pack_left_shift_u16(acc_y_body, 0u, 0xffu);
    acc_z_body = (uint16_t)src_p->acc_z_body;
    dst_p[4] |= pack_right_shift_u16(acc_z_body, 8u, 0xffu);
    dst_p[5] |= pack_left_shift_u16(acc_z_body, 0u, 0xffu);

    return (6);
}

int kistler_0x7e6_acc_body_t::unpack(
    struct kistler_0x7e6_acc_body_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t acc_x_body;
    uint16_t acc_y_body;
    uint16_t acc_z_body;

    if (size < 6u) {
        return (-EINVAL);
    }

    acc_x_body = unpack_left_shift_u16(src_p[0], 8u, 0xffu);
    acc_x_body |= unpack_right_shift_u16(src_p[1], 0u, 0xffu);
    dst_p->acc_x_body = (int16_t)acc_x_body;
    acc_y_body = unpack_left_shift_u16(src_p[2], 8u, 0xffu);
    acc_y_body |= unpack_right_shift_u16(src_p[3], 0u, 0xffu);
    dst_p->acc_y_body = (int16_t)acc_y_body;
    acc_z_body = unpack_left_shift_u16(src_p[4], 8u, 0xffu);
    acc_z_body |= unpack_right_shift_u16(src_p[5], 0u, 0xffu);
    dst_p->acc_z_body = (int16_t)acc_z_body;

    return (0);
}

int kistler_0x7e6_acc_body_init(struct kistler_0x7e6_acc_body_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kistler_0x7e6_acc_body_t));

    return 0;
}

int16_t Kistler_0x7e6_acc_body_acc_x_body_encode(float value)
{
    return (int16_t)(value / 0.01f);
}

float Kistler_0x7e6_acc_body_acc_x_body_decode(int16_t value)
{
    return ((float)value * 0.01f);
}

bool Kistler_0x7e6_acc_body_acc_x_body_is_in_range(int16_t value)
{
    return ((value >= -18000) && (value <= 18000));
}

int16_t Kistler_0x7e6_acc_body_acc_y_body_encode(float value)
{
    return (int16_t)(value / 0.01f);
}

float Kistler_0x7e6_acc_body_acc_y_body_decode(int16_t value)
{
    return ((float)value * 0.01f);
}

bool Kistler_0x7e6_acc_body_acc_y_body_is_in_range(int16_t value)
{
    return ((value >= -18000) && (value <= 18000));
}

int16_t Kistler_0x7e6_acc_body_acc_z_body_encode(float value)
{
    return (int16_t)(value / 0.01f);
}

float Kistler_0x7e6_acc_body_acc_z_body_decode(int16_t value)
{
    return ((float)value * 0.01f);
}

bool Kistler_0x7e6_acc_body_acc_z_body_is_in_range(int16_t value)
{
    return ((value >= -18000) && (value <= 18000));
}

int kistler_0x7e7_ang_vel_body_t::pack(
    uint8_t *dst_p,
    // const struct kistler_0x7e7_ang_vel_body_t *src_p,
    size_t size)
{
    auto src_p = this;
    uint16_t ang_vel_x_body;
    uint16_t ang_vel_y_body;
    uint16_t ang_vel_z_body;

    if (size < 6u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 6);

    ang_vel_x_body = (uint16_t)src_p->ang_vel_x_body;
    dst_p[0] |= pack_right_shift_u16(ang_vel_x_body, 8u, 0xffu);
    dst_p[1] |= pack_left_shift_u16(ang_vel_x_body, 0u, 0xffu);
    ang_vel_y_body = (uint16_t)src_p->ang_vel_y_body;
    dst_p[2] |= pack_right_shift_u16(ang_vel_y_body, 8u, 0xffu);
    dst_p[3] |= pack_left_shift_u16(ang_vel_y_body, 0u, 0xffu);
    ang_vel_z_body = (uint16_t)src_p->ang_vel_z_body;
    dst_p[4] |= pack_right_shift_u16(ang_vel_z_body, 8u, 0xffu);
    dst_p[5] |= pack_left_shift_u16(ang_vel_z_body, 0u, 0xffu);

    return (6);
}

int kistler_0x7e7_ang_vel_body_t::unpack(
    struct kistler_0x7e7_ang_vel_body_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t ang_vel_x_body;
    uint16_t ang_vel_y_body;
    uint16_t ang_vel_z_body;

    if (size < 6u) {
        return (-EINVAL);
    }

    ang_vel_x_body = unpack_left_shift_u16(src_p[0], 8u, 0xffu);
    ang_vel_x_body |= unpack_right_shift_u16(src_p[1], 0u, 0xffu);
    dst_p->ang_vel_x_body = (int16_t)ang_vel_x_body;
    ang_vel_y_body = unpack_left_shift_u16(src_p[2], 8u, 0xffu);
    ang_vel_y_body |= unpack_right_shift_u16(src_p[3], 0u, 0xffu);
    dst_p->ang_vel_y_body = (int16_t)ang_vel_y_body;
    ang_vel_z_body = unpack_left_shift_u16(src_p[4], 8u, 0xffu);
    ang_vel_z_body |= unpack_right_shift_u16(src_p[5], 0u, 0xffu);
    dst_p->ang_vel_z_body = (int16_t)ang_vel_z_body;

    return (0);
}

int kistler_0x7e7_ang_vel_body_init(struct kistler_0x7e7_ang_vel_body_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kistler_0x7e7_ang_vel_body_t));

    return 0;
}

int16_t Kistler_0x7e7_ang_vel_body_ang_vel_x_body_encode(float value)
{
    return (int16_t)(value / 0.02f);
}

float Kistler_0x7e7_ang_vel_body_ang_vel_x_body_decode(int16_t value)
{
    return ((float)value * 0.02f);
}

bool Kistler_0x7e7_ang_vel_body_ang_vel_x_body_is_in_range(int16_t value)
{
    return ((value >= -15000) && (value <= 15000));
}

int16_t Kistler_0x7e7_ang_vel_body_ang_vel_y_body_encode(float value)
{
    return (int16_t)(value / 0.02f);
}

float Kistler_0x7e7_ang_vel_body_ang_vel_y_body_decode(int16_t value)
{
    return ((float)value * 0.02f);
}

bool Kistler_0x7e7_ang_vel_body_ang_vel_y_body_is_in_range(int16_t value)
{
    return ((value >= -15000) && (value <= 15000));
}

int16_t Kistler_0x7e7_ang_vel_body_ang_vel_z_body_encode(float value)
{
    return (int16_t)(value / 0.02f);
}

float Kistler_0x7e7_ang_vel_body_ang_vel_z_body_decode(int16_t value)
{
    return ((float)value * 0.02f);
}

bool Kistler_0x7e7_ang_vel_body_ang_vel_z_body_is_in_range(int16_t value)
{
    return ((value >= -15000) && (value <= 15000));
}

int kistler_0x7e8_status_t::pack(
    uint8_t *dst_p,
    // const struct kistler_0x7e8_status_t *src_p,
    size_t size)
{
    auto src_p = this;
    uint8_t temperature;

    if (size < 7u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 7);

    dst_p[0] |= pack_right_shift_u16(src_p->sensor_id, 8u, 0xffu);
    dst_p[1] |= pack_left_shift_u16(src_p->sensor_id, 0u, 0xffu);
    temperature = (uint8_t)src_p->temperature;
    dst_p[2] |= pack_left_shift_u8(temperature, 0u, 0xffu);
    dst_p[3] |= pack_left_shift_u8(src_p->lamp_current, 0u, 0xffu);
    dst_p[4] |= pack_left_shift_u8(src_p->filter_setting, 0u, 0xffu);
    dst_p[6] |= pack_left_shift_u8(src_p->direction, 7u, 0x80u);
    dst_p[6] |= pack_left_shift_u8(src_p->angle_switched_off, 6u, 0x40u);
    dst_p[6] |= pack_left_shift_u8(src_p->head_status, 4u, 0x30u);
    dst_p[6] |= pack_left_shift_u8(src_p->temperature_ok, 3u, 0x08u);
    dst_p[6] |= pack_left_shift_u8(src_p->lamp_current_control, 2u, 0x04u);
    dst_p[6] |= pack_left_shift_u8(src_p->filter_off_on, 1u, 0x02u);
    dst_p[6] |= pack_left_shift_u8(src_p->stst, 0u, 0x01u);
    dst_p[7] |= pack_left_shift_u8(src_p->direction_head, 4u, 0x10u);
    dst_p[7] |= pack_left_shift_u8(src_p->direction_head_is_valid, 3u, 0x08u);
    dst_p[7] |= pack_left_shift_u8(src_p->direction_mounting, 2u, 0x04u);
    dst_p[7] |= pack_left_shift_u8(src_p->direction_motion, 1u, 0x02u);
    dst_p[7] |= pack_left_shift_u8(src_p->ang_vel_correction, 0u, 0x01u);

    return (7);
}

int kistler_0x7e8_status_t::unpack(
    struct kistler_0x7e8_status_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint8_t temperature;

    if (size < 7u) {
        return (-EINVAL);
    }

    dst_p->sensor_id = unpack_left_shift_u16(src_p[0], 8u, 0xffu);
    dst_p->sensor_id |= unpack_right_shift_u16(src_p[1], 0u, 0xffu);
    temperature = unpack_right_shift_u8(src_p[2], 0u, 0xffu);
    dst_p->temperature = (int8_t)temperature;
    dst_p->lamp_current = unpack_right_shift_u8(src_p[3], 0u, 0xffu);
    dst_p->filter_setting = unpack_right_shift_u8(src_p[4], 0u, 0xffu);
    dst_p->direction = unpack_right_shift_u8(src_p[6], 7u, 0x80u);
    dst_p->angle_switched_off = unpack_right_shift_u8(src_p[6], 6u, 0x40u);
    dst_p->head_status = unpack_right_shift_u8(src_p[6], 4u, 0x30u);
    dst_p->temperature_ok = unpack_right_shift_u8(src_p[6], 3u, 0x08u);
    dst_p->lamp_current_control = unpack_right_shift_u8(src_p[6], 2u, 0x04u);
    dst_p->filter_off_on = unpack_right_shift_u8(src_p[6], 1u, 0x02u);
    dst_p->stst = unpack_right_shift_u8(src_p[6], 0u, 0x01u);
    dst_p->direction_head = unpack_right_shift_u8(src_p[7], 4u, 0x10u);
    dst_p->direction_head_is_valid = unpack_right_shift_u8(src_p[7], 3u, 0x08u);
    dst_p->direction_mounting = unpack_right_shift_u8(src_p[7], 2u, 0x04u);
    dst_p->direction_motion = unpack_right_shift_u8(src_p[7], 1u, 0x02u);
    dst_p->ang_vel_correction = unpack_right_shift_u8(src_p[7], 0u, 0x01u);

    return (0);
}

int kistler_0x7e8_status_init(struct kistler_0x7e8_status_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct kistler_0x7e8_status_t));

    return 0;
}

uint16_t Kistler_0x7e8_status_sensor_id_encode(float value)
{
    return (uint16_t)(value);
}

float Kistler_0x7e8_status_sensor_id_decode(uint16_t value)
{
    return ((float)value);
}

bool Kistler_0x7e8_status_sensor_id_is_in_range(uint16_t value)
{
    return (value <= 9999u);
}

int8_t Kistler_0x7e8_status_temperature_encode(float value)
{
    return (int8_t)(value);
}

float Kistler_0x7e8_status_temperature_decode(int8_t value)
{
    return ((float)value);
}

bool Kistler_0x7e8_status_temperature_is_in_range(int8_t value)
{
    (void)value;

    return (true);
}

uint8_t Kistler_0x7e8_status_lamp_current_encode(float value)
{
    return (uint8_t)(value / 0.01f);
}

float Kistler_0x7e8_status_lamp_current_decode(uint8_t value)
{
    return ((float)value * 0.01f);
}

bool Kistler_0x7e8_status_lamp_current_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t Kistler_0x7e8_status_filter_setting_encode(float value)
{
    return (uint8_t)(value);
}

float Kistler_0x7e8_status_filter_setting_decode(uint8_t value)
{
    return ((float)value);
}

bool Kistler_0x7e8_status_filter_setting_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t Kistler_0x7e8_status_direction_encode(float value)
{
    return (uint8_t)(value);
}

float Kistler_0x7e8_status_direction_decode(uint8_t value)
{
    return ((float)value);
}

bool Kistler_0x7e8_status_direction_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t Kistler_0x7e8_status_angle_switched_off_encode(float value)
{
    return (uint8_t)(value);
}

float Kistler_0x7e8_status_angle_switched_off_decode(uint8_t value)
{
    return ((float)value);
}

bool Kistler_0x7e8_status_angle_switched_off_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t Kistler_0x7e8_status_head_status_encode(float value)
{
    return (uint8_t)(value);
}

float Kistler_0x7e8_status_head_status_decode(uint8_t value)
{
    return ((float)value);
}

bool Kistler_0x7e8_status_head_status_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t Kistler_0x7e8_status_temperature_ok_encode(float value)
{
    return (uint8_t)(value);
}

float Kistler_0x7e8_status_temperature_ok_decode(uint8_t value)
{
    return ((float)value);
}

bool Kistler_0x7e8_status_temperature_ok_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t Kistler_0x7e8_status_lamp_current_control_encode(float value)
{
    return (uint8_t)(value);
}

float Kistler_0x7e8_status_lamp_current_control_decode(uint8_t value)
{
    return ((float)value);
}

bool Kistler_0x7e8_status_lamp_current_control_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t Kistler_0x7e8_status_filter_off_on_encode(float value)
{
    return (uint8_t)(value);
}

float Kistler_0x7e8_status_filter_off_on_decode(uint8_t value)
{
    return ((float)value);
}

bool Kistler_0x7e8_status_filter_off_on_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t Kistler_0x7e8_status_stst_encode(float value)
{
    return (uint8_t)(value);
}

float Kistler_0x7e8_status_stst_decode(uint8_t value)
{
    return ((float)value);
}

bool Kistler_0x7e8_status_stst_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t Kistler_0x7e8_status_direction_head_encode(float value)
{
    return (uint8_t)(value);
}

float Kistler_0x7e8_status_direction_head_decode(uint8_t value)
{
    return ((float)value);
}

bool Kistler_0x7e8_status_direction_head_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t Kistler_0x7e8_status_direction_head_is_valid_encode(float value)
{
    return (uint8_t)(value);
}

float Kistler_0x7e8_status_direction_head_is_valid_decode(uint8_t value)
{
    return ((float)value);
}

bool Kistler_0x7e8_status_direction_head_is_valid_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t Kistler_0x7e8_status_direction_mounting_encode(float value)
{
    return (uint8_t)(value);
}

float Kistler_0x7e8_status_direction_mounting_decode(uint8_t value)
{
    return ((float)value);
}

bool Kistler_0x7e8_status_direction_mounting_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t Kistler_0x7e8_status_direction_motion_encode(float value)
{
    return (uint8_t)(value);
}

float Kistler_0x7e8_status_direction_motion_decode(uint8_t value)
{
    return ((float)value);
}

bool Kistler_0x7e8_status_direction_motion_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t Kistler_0x7e8_status_ang_vel_correction_encode(float value)
{
    return (uint8_t)(value);
}

float Kistler_0x7e8_status_ang_vel_correction_decode(uint8_t value)
{
    return ((float)value);
}

bool Kistler_0x7e8_status_ang_vel_correction_is_in_range(uint8_t value)
{
    return (value <= 1u);
}
