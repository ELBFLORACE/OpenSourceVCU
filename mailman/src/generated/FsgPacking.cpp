/**
 * @file Fsg
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

#include "generated/Fsg.hpp"



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

static inline uint8_t pack_right_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint8_t pack_right_shift_u16(
    uint16_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value >> shift) & mask);
}

static inline uint8_t unpack_left_shift_u8(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint8_t)((uint8_t)(value & mask) << shift);
}

static inline uint16_t unpack_left_shift_u16(
    uint8_t value,
    uint8_t shift,
    uint8_t mask)
{
    return (uint16_t)((uint16_t)(value & mask) << shift);
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

int fsg_0x430_fs_datalogger_status_t::pack(
    uint8_t *dst_p,
    // const struct fsg_0x430_fs_datalogger_status_t *src_p,
    size_t size)
{
    auto src_p = this;
    uint8_t status_logging;
    uint8_t status_ready;
    uint8_t status_triggered_current;
    uint8_t status_triggered_voltage;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u8(src_p->msg_cnt, 0u, 0xffu);
    status_ready = (uint8_t)src_p->status_ready;
    dst_p[1] |= pack_left_shift_u8(status_ready, 0u, 0x01u);
    status_logging = (uint8_t)src_p->status_logging;
    dst_p[1] |= pack_left_shift_u8(status_logging, 1u, 0x02u);
    status_triggered_voltage = (uint8_t)src_p->status_triggered_voltage;
    dst_p[1] |= pack_left_shift_u8(status_triggered_voltage, 2u, 0x04u);
    status_triggered_current = (uint8_t)src_p->status_triggered_current;
    dst_p[1] |= pack_left_shift_u8(status_triggered_current, 3u, 0x08u);
    dst_p[2] |= pack_left_shift_u16(src_p->voltage, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(src_p->voltage, 8u, 0xffu);
    dst_p[4] |= pack_left_shift_u16(src_p->current, 0u, 0xffu);
    dst_p[5] |= pack_right_shift_u16(src_p->current, 8u, 0xffu);

    return (8);
}

int fsg_0x430_fs_datalogger_status_t::unpack(
    struct fsg_0x430_fs_datalogger_status_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint8_t status_logging;
    uint8_t status_ready;
    uint8_t status_triggered_current;
    uint8_t status_triggered_voltage;

    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->msg_cnt = unpack_right_shift_u8(src_p[0], 0u, 0xffu);
    status_ready = unpack_right_shift_u8(src_p[1], 0u, 0x01u);

    if ((status_ready & (1u << 0)) != 0u) {
        status_ready |= 0xfeu;
    }

    dst_p->status_ready = (int8_t)status_ready;
    status_logging = unpack_right_shift_u8(src_p[1], 1u, 0x02u);

    if ((status_logging & (1u << 0)) != 0u) {
        status_logging |= 0xfeu;
    }

    dst_p->status_logging = (int8_t)status_logging;
    status_triggered_voltage = unpack_right_shift_u8(src_p[1], 2u, 0x04u);

    if ((status_triggered_voltage & (1u << 0)) != 0u) {
        status_triggered_voltage |= 0xfeu;
    }

    dst_p->status_triggered_voltage = (int8_t)status_triggered_voltage;
    status_triggered_current = unpack_right_shift_u8(src_p[1], 3u, 0x08u);

    if ((status_triggered_current & (1u << 0)) != 0u) {
        status_triggered_current |= 0xfeu;
    }

    dst_p->status_triggered_current = (int8_t)status_triggered_current;
    dst_p->voltage = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    dst_p->voltage |= unpack_left_shift_u16(src_p[3], 8u, 0xffu);
    dst_p->current = unpack_right_shift_u16(src_p[4], 0u, 0xffu);
    dst_p->current |= unpack_left_shift_u16(src_p[5], 8u, 0xffu);

    return (0);
}

int fsg_0x430_fs_datalogger_status_init(struct fsg_0x430_fs_datalogger_status_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct fsg_0x430_fs_datalogger_status_t));

    return 0;
}

uint8_t Fsg_0x430_fs_datalogger_status_msg_cnt_encode(float value)
{
    return (uint8_t)(value);
}

float Fsg_0x430_fs_datalogger_status_msg_cnt_decode(uint8_t value)
{
    return ((float)value);
}

bool Fsg_0x430_fs_datalogger_status_msg_cnt_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

int8_t Fsg_0x430_fs_datalogger_status_status_ready_encode(float value)
{
    return (int8_t)(value);
}

float Fsg_0x430_fs_datalogger_status_status_ready_decode(int8_t value)
{
    return ((float)value);
}

bool Fsg_0x430_fs_datalogger_status_status_ready_is_in_range(int8_t value)
{
    return ((value >= -1) && (value <= 0));
}

int8_t Fsg_0x430_fs_datalogger_status_status_logging_encode(float value)
{
    return (int8_t)(value);
}

float Fsg_0x430_fs_datalogger_status_status_logging_decode(int8_t value)
{
    return ((float)value);
}

bool Fsg_0x430_fs_datalogger_status_status_logging_is_in_range(int8_t value)
{
    return ((value >= 0) && (value <= 1));
}

int8_t Fsg_0x430_fs_datalogger_status_status_triggered_voltage_encode(float value)
{
    return (int8_t)(value);
}

float Fsg_0x430_fs_datalogger_status_status_triggered_voltage_decode(int8_t value)
{
    return ((float)value);
}

bool Fsg_0x430_fs_datalogger_status_status_triggered_voltage_is_in_range(int8_t value)
{
    return ((value >= -1) && (value <= 0));
}

int8_t Fsg_0x430_fs_datalogger_status_status_triggered_current_encode(float value)
{
    return (int8_t)(value);
}

float Fsg_0x430_fs_datalogger_status_status_triggered_current_decode(int8_t value)
{
    return ((float)value);
}

bool Fsg_0x430_fs_datalogger_status_status_triggered_current_is_in_range(int8_t value)
{
    return ((value >= -1) && (value <= 0));
}

uint16_t Fsg_0x430_fs_datalogger_status_voltage_encode(float value)
{
    return (uint16_t)(value / 16.0f);
}

float Fsg_0x430_fs_datalogger_status_voltage_decode(uint16_t value)
{
    return ((float)value * 16.0f);
}

bool Fsg_0x430_fs_datalogger_status_voltage_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

uint16_t Fsg_0x430_fs_datalogger_status_current_encode(float value)
{
    return (uint16_t)(value / 64.0f);
}

float Fsg_0x430_fs_datalogger_status_current_decode(uint16_t value)
{
    return ((float)value * 64.0f);
}

bool Fsg_0x430_fs_datalogger_status_current_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

int fsg_0x502_dv_system_status_t::pack(
    uint8_t *dst_p,
    // const struct fsg_0x502_dv_system_status_t *src_p,
    size_t size)
{
    auto src_p = this;
    if (size < 5u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 5);

    dst_p[0] |= pack_left_shift_u8(src_p->as_state, 0u, 0x07u);
    dst_p[0] |= pack_left_shift_u8(src_p->ebs_state, 3u, 0x18u);
    dst_p[0] |= pack_left_shift_u8(src_p->ami_state, 5u, 0xe0u);
    dst_p[1] |= pack_left_shift_u8(src_p->steering_state, 0u, 0x01u);
    dst_p[1] |= pack_left_shift_u8(src_p->asb_redundancy_state, 1u, 0x06u);
    dst_p[1] |= pack_left_shift_u8(src_p->lapcounter, 3u, 0x78u);
    dst_p[1] |= pack_left_shift_u8(src_p->cones_count_actual, 7u, 0x80u);
    dst_p[2] |= pack_right_shift_u8(src_p->cones_count_actual, 1u, 0x7fu);
    dst_p[2] |= pack_left_shift_u16(src_p->cones_count_all, 7u, 0x80u);
    dst_p[3] |= pack_right_shift_u16(src_p->cones_count_all, 1u, 0xffu);
    dst_p[4] |= pack_right_shift_u16(src_p->cones_count_all, 9u, 0x7fu);

    return (5);
}

int fsg_0x502_dv_system_status_t::unpack(
    struct fsg_0x502_dv_system_status_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    if (size < 5u) {
        return (-EINVAL);
    }

    dst_p->as_state = unpack_right_shift_u8(src_p[0], 0u, 0x07u);
    dst_p->ebs_state = unpack_right_shift_u8(src_p[0], 3u, 0x18u);
    dst_p->ami_state = unpack_right_shift_u8(src_p[0], 5u, 0xe0u);
    dst_p->steering_state = unpack_right_shift_u8(src_p[1], 0u, 0x01u);
    dst_p->asb_redundancy_state = unpack_right_shift_u8(src_p[1], 1u, 0x06u);
    dst_p->lapcounter = unpack_right_shift_u8(src_p[1], 3u, 0x78u);
    dst_p->cones_count_actual = unpack_right_shift_u8(src_p[1], 7u, 0x80u);
    dst_p->cones_count_actual |= unpack_left_shift_u8(src_p[2], 1u, 0x7fu);
    dst_p->cones_count_all = unpack_right_shift_u16(src_p[2], 7u, 0x80u);
    dst_p->cones_count_all |= unpack_left_shift_u16(src_p[3], 1u, 0xffu);
    dst_p->cones_count_all |= unpack_left_shift_u16(src_p[4], 9u, 0x7fu);

    return (0);
}

int fsg_0x502_dv_system_status_init(struct fsg_0x502_dv_system_status_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct fsg_0x502_dv_system_status_t));

    return 0;
}

uint8_t Fsg_0x502_dv_system_status_as_state_encode(float value)
{
    return (uint8_t)(value);
}

float Fsg_0x502_dv_system_status_as_state_decode(uint8_t value)
{
    return ((float)value);
}

bool Fsg_0x502_dv_system_status_as_state_is_in_range(uint8_t value)
{
    return (value <= 7u);
}

uint8_t Fsg_0x502_dv_system_status_ebs_state_encode(float value)
{
    return (uint8_t)(value);
}

float Fsg_0x502_dv_system_status_ebs_state_decode(uint8_t value)
{
    return ((float)value);
}

bool Fsg_0x502_dv_system_status_ebs_state_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t Fsg_0x502_dv_system_status_ami_state_encode(float value)
{
    return (uint8_t)(value);
}

float Fsg_0x502_dv_system_status_ami_state_decode(uint8_t value)
{
    return ((float)value);
}

bool Fsg_0x502_dv_system_status_ami_state_is_in_range(uint8_t value)
{
    return (value <= 7u);
}

uint8_t Fsg_0x502_dv_system_status_steering_state_encode(float value)
{
    return (uint8_t)(value);
}

float Fsg_0x502_dv_system_status_steering_state_decode(uint8_t value)
{
    return ((float)value);
}

bool Fsg_0x502_dv_system_status_steering_state_is_in_range(uint8_t value)
{
    return (value <= 1u);
}

uint8_t Fsg_0x502_dv_system_status_asb_redundancy_state_encode(float value)
{
    return (uint8_t)(value);
}

float Fsg_0x502_dv_system_status_asb_redundancy_state_decode(uint8_t value)
{
    return ((float)value);
}

bool Fsg_0x502_dv_system_status_asb_redundancy_state_is_in_range(uint8_t value)
{
    return (value <= 3u);
}

uint8_t Fsg_0x502_dv_system_status_lapcounter_encode(float value)
{
    return (uint8_t)(value);
}

float Fsg_0x502_dv_system_status_lapcounter_decode(uint8_t value)
{
    return ((float)value);
}

bool Fsg_0x502_dv_system_status_lapcounter_is_in_range(uint8_t value)
{
    return (value <= 15u);
}

uint8_t Fsg_0x502_dv_system_status_cones_count_actual_encode(float value)
{
    return (uint8_t)(value);
}

float Fsg_0x502_dv_system_status_cones_count_actual_decode(uint8_t value)
{
    return ((float)value);
}

bool Fsg_0x502_dv_system_status_cones_count_actual_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint16_t Fsg_0x502_dv_system_status_cones_count_all_encode(float value)
{
    return (uint16_t)(value);
}

float Fsg_0x502_dv_system_status_cones_count_all_decode(uint16_t value)
{
    return ((float)value);
}

bool Fsg_0x502_dv_system_status_cones_count_all_is_in_range(uint16_t value)
{
    (void)value;

    return (true);
}

int fsg_0x501_dv_driving_dynamics_2_t::pack(
    uint8_t *dst_p,
    // const struct fsg_0x501_dv_driving_dynamics_2_t *src_p,
    size_t size)
{
    auto src_p = this;
    uint16_t acceleration_lateral;
    uint16_t acceleration_longitudinal;
    uint16_t yaw_rate;

    if (size < 6u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 6);

    acceleration_longitudinal = (uint16_t)src_p->acceleration_longitudinal;
    dst_p[0] |= pack_left_shift_u16(acceleration_longitudinal, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(acceleration_longitudinal, 8u, 0xffu);
    acceleration_lateral = (uint16_t)src_p->acceleration_lateral;
    dst_p[2] |= pack_left_shift_u16(acceleration_lateral, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(acceleration_lateral, 8u, 0xffu);
    yaw_rate = (uint16_t)src_p->yaw_rate;
    dst_p[4] |= pack_left_shift_u16(yaw_rate, 0u, 0xffu);
    dst_p[5] |= pack_right_shift_u16(yaw_rate, 8u, 0xffu);

    return (6);
}

int fsg_0x501_dv_driving_dynamics_2_t::unpack(
    struct fsg_0x501_dv_driving_dynamics_2_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t acceleration_lateral;
    uint16_t acceleration_longitudinal;
    uint16_t yaw_rate;

    if (size < 6u) {
        return (-EINVAL);
    }

    acceleration_longitudinal = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    acceleration_longitudinal |= unpack_left_shift_u16(src_p[1], 8u, 0xffu);
    dst_p->acceleration_longitudinal = (int16_t)acceleration_longitudinal;
    acceleration_lateral = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    acceleration_lateral |= unpack_left_shift_u16(src_p[3], 8u, 0xffu);
    dst_p->acceleration_lateral = (int16_t)acceleration_lateral;
    yaw_rate = unpack_right_shift_u16(src_p[4], 0u, 0xffu);
    yaw_rate |= unpack_left_shift_u16(src_p[5], 8u, 0xffu);
    dst_p->yaw_rate = (int16_t)yaw_rate;

    return (0);
}

int fsg_0x501_dv_driving_dynamics_2_init(struct fsg_0x501_dv_driving_dynamics_2_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct fsg_0x501_dv_driving_dynamics_2_t));

    return 0;
}

int16_t Fsg_0x501_dv_driving_dynamics_2_acceleration_longitudinal_encode(float value)
{
    return (int16_t)(value / 0.001953125f);
}

float Fsg_0x501_dv_driving_dynamics_2_acceleration_longitudinal_decode(int16_t value)
{
    return ((float)value * 0.001953125f);
}

bool Fsg_0x501_dv_driving_dynamics_2_acceleration_longitudinal_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

int16_t Fsg_0x501_dv_driving_dynamics_2_acceleration_lateral_encode(float value)
{
    return (int16_t)(value / 0.001953125f);
}

float Fsg_0x501_dv_driving_dynamics_2_acceleration_lateral_decode(int16_t value)
{
    return ((float)value * 0.001953125f);
}

bool Fsg_0x501_dv_driving_dynamics_2_acceleration_lateral_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

int16_t Fsg_0x501_dv_driving_dynamics_2_yaw_rate_encode(float value)
{
    return (int16_t)(value / 0.0078125f);
}

float Fsg_0x501_dv_driving_dynamics_2_yaw_rate_decode(int16_t value)
{
    return ((float)value * 0.0078125f);
}

bool Fsg_0x501_dv_driving_dynamics_2_yaw_rate_is_in_range(int16_t value)
{
    (void)value;

    return (true);
}

int fsg_0x500_dv_driving_dynamics1_t::pack(
    uint8_t *dst_p,
    // const struct fsg_0x500_dv_driving_dynamics1_t *src_p,
    size_t size)
{
    auto src_p = this;
    uint8_t motor_moment_actual;
    uint8_t motor_moment_target;
    uint8_t steering_angle_actual;
    uint8_t steering_angle_target;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    dst_p[0] |= pack_left_shift_u8(src_p->speed_actual, 0u, 0xffu);
    dst_p[1] |= pack_left_shift_u8(src_p->speed_target, 0u, 0xffu);
    steering_angle_actual = (uint8_t)src_p->steering_angle_actual;
    dst_p[2] |= pack_left_shift_u8(steering_angle_actual, 0u, 0xffu);
    steering_angle_target = (uint8_t)src_p->steering_angle_target;
    dst_p[3] |= pack_left_shift_u8(steering_angle_target, 0u, 0xffu);
    dst_p[4] |= pack_left_shift_u8(src_p->brake_hydr_actual, 0u, 0xffu);
    dst_p[5] |= pack_left_shift_u8(src_p->brake_hydr_target, 0u, 0xffu);
    motor_moment_actual = (uint8_t)src_p->motor_moment_actual;
    dst_p[6] |= pack_left_shift_u8(motor_moment_actual, 0u, 0xffu);
    motor_moment_target = (uint8_t)src_p->motor_moment_target;
    dst_p[7] |= pack_left_shift_u8(motor_moment_target, 0u, 0xffu);

    return (8);
}

int fsg_0x500_dv_driving_dynamics1_t::unpack(
    struct fsg_0x500_dv_driving_dynamics1_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint8_t motor_moment_actual;
    uint8_t motor_moment_target;
    uint8_t steering_angle_actual;
    uint8_t steering_angle_target;

    if (size < 8u) {
        return (-EINVAL);
    }

    dst_p->speed_actual = unpack_right_shift_u8(src_p[0], 0u, 0xffu);
    dst_p->speed_target = unpack_right_shift_u8(src_p[1], 0u, 0xffu);
    steering_angle_actual = unpack_right_shift_u8(src_p[2], 0u, 0xffu);
    dst_p->steering_angle_actual = (int8_t)steering_angle_actual;
    steering_angle_target = unpack_right_shift_u8(src_p[3], 0u, 0xffu);
    dst_p->steering_angle_target = (int8_t)steering_angle_target;
    dst_p->brake_hydr_actual = unpack_right_shift_u8(src_p[4], 0u, 0xffu);
    dst_p->brake_hydr_target = unpack_right_shift_u8(src_p[5], 0u, 0xffu);
    motor_moment_actual = unpack_right_shift_u8(src_p[6], 0u, 0xffu);
    dst_p->motor_moment_actual = (int8_t)motor_moment_actual;
    motor_moment_target = unpack_right_shift_u8(src_p[7], 0u, 0xffu);
    dst_p->motor_moment_target = (int8_t)motor_moment_target;

    return (0);
}

int fsg_0x500_dv_driving_dynamics1_init(struct fsg_0x500_dv_driving_dynamics1_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct fsg_0x500_dv_driving_dynamics1_t));

    return 0;
}

uint8_t Fsg_0x500_dv_driving_dynamics1_speed_actual_encode(float value)
{
    return (uint8_t)(value);
}

float Fsg_0x500_dv_driving_dynamics1_speed_actual_decode(uint8_t value)
{
    return ((float)value);
}

bool Fsg_0x500_dv_driving_dynamics1_speed_actual_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t Fsg_0x500_dv_driving_dynamics1_speed_target_encode(float value)
{
    return (uint8_t)(value);
}

float Fsg_0x500_dv_driving_dynamics1_speed_target_decode(uint8_t value)
{
    return ((float)value);
}

bool Fsg_0x500_dv_driving_dynamics1_speed_target_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

int8_t Fsg_0x500_dv_driving_dynamics1_steering_angle_actual_encode(float value)
{
    return (int8_t)(value / 0.5f);
}

float Fsg_0x500_dv_driving_dynamics1_steering_angle_actual_decode(int8_t value)
{
    return ((float)value * 0.5f);
}

bool Fsg_0x500_dv_driving_dynamics1_steering_angle_actual_is_in_range(int8_t value)
{
    (void)value;

    return (true);
}

int8_t Fsg_0x500_dv_driving_dynamics1_steering_angle_target_encode(float value)
{
    return (int8_t)(value / 0.5f);
}

float Fsg_0x500_dv_driving_dynamics1_steering_angle_target_decode(int8_t value)
{
    return ((float)value * 0.5f);
}

bool Fsg_0x500_dv_driving_dynamics1_steering_angle_target_is_in_range(int8_t value)
{
    (void)value;

    return (true);
}

uint8_t Fsg_0x500_dv_driving_dynamics1_brake_hydr_actual_encode(float value)
{
    return (uint8_t)(value);
}

float Fsg_0x500_dv_driving_dynamics1_brake_hydr_actual_decode(uint8_t value)
{
    return ((float)value);
}

bool Fsg_0x500_dv_driving_dynamics1_brake_hydr_actual_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

uint8_t Fsg_0x500_dv_driving_dynamics1_brake_hydr_target_encode(float value)
{
    return (uint8_t)(value);
}

float Fsg_0x500_dv_driving_dynamics1_brake_hydr_target_decode(uint8_t value)
{
    return ((float)value);
}

bool Fsg_0x500_dv_driving_dynamics1_brake_hydr_target_is_in_range(uint8_t value)
{
    (void)value;

    return (true);
}

int8_t Fsg_0x500_dv_driving_dynamics1_motor_moment_actual_encode(float value)
{
    return (int8_t)(value);
}

float Fsg_0x500_dv_driving_dynamics1_motor_moment_actual_decode(int8_t value)
{
    return ((float)value);
}

bool Fsg_0x500_dv_driving_dynamics1_motor_moment_actual_is_in_range(int8_t value)
{
    (void)value;

    return (true);
}

int8_t Fsg_0x500_dv_driving_dynamics1_motor_moment_target_encode(float value)
{
    return (int8_t)(value);
}

float Fsg_0x500_dv_driving_dynamics1_motor_moment_target_decode(int8_t value)
{
    return ((float)value);
}

bool Fsg_0x500_dv_driving_dynamics1_motor_moment_target_is_in_range(int8_t value)
{
    (void)value;

    return (true);
}

int fsg_0x511_brake_and_ebs_pressure_t::pack(
    uint8_t *dst_p,
    // const struct fsg_0x511_brake_and_ebs_pressure_t *src_p,
    size_t size)
{
    auto src_p = this;
    uint16_t brakepressure_front;
    uint16_t brakepressure_rear;
    uint16_t ebs_1_pressure;
    uint16_t ebs_2_pressure;

    if (size < 8u) {
        return (-EINVAL);
    }

    memset(&dst_p[0], 0, 8);

    ebs_1_pressure = (uint16_t)src_p->ebs_1_pressure;
    dst_p[0] |= pack_left_shift_u16(ebs_1_pressure, 0u, 0xffu);
    dst_p[1] |= pack_right_shift_u16(ebs_1_pressure, 8u, 0x0fu);
    ebs_2_pressure = (uint16_t)src_p->ebs_2_pressure;
    dst_p[2] |= pack_left_shift_u16(ebs_2_pressure, 0u, 0xffu);
    dst_p[3] |= pack_right_shift_u16(ebs_2_pressure, 8u, 0x0fu);
    brakepressure_front = (uint16_t)src_p->brakepressure_front;
    dst_p[4] |= pack_left_shift_u16(brakepressure_front, 0u, 0xffu);
    dst_p[5] |= pack_right_shift_u16(brakepressure_front, 8u, 0x0fu);
    brakepressure_rear = (uint16_t)src_p->brakepressure_rear;
    dst_p[6] |= pack_left_shift_u16(brakepressure_rear, 0u, 0xffu);
    dst_p[7] |= pack_right_shift_u16(brakepressure_rear, 8u, 0x0fu);

    return (8);
}

int fsg_0x511_brake_and_ebs_pressure_t::unpack(
    struct fsg_0x511_brake_and_ebs_pressure_t *dst_p,
    const uint8_t *src_p,
    size_t size)
{
    uint16_t brakepressure_front;
    uint16_t brakepressure_rear;
    uint16_t ebs_1_pressure;
    uint16_t ebs_2_pressure;

    if (size < 8u) {
        return (-EINVAL);
    }

    ebs_1_pressure = unpack_right_shift_u16(src_p[0], 0u, 0xffu);
    ebs_1_pressure |= unpack_left_shift_u16(src_p[1], 8u, 0x0fu);

    if ((ebs_1_pressure & (1u << 11)) != 0u) {
        ebs_1_pressure |= 0xf000u;
    }

    dst_p->ebs_1_pressure = (int16_t)ebs_1_pressure;
    ebs_2_pressure = unpack_right_shift_u16(src_p[2], 0u, 0xffu);
    ebs_2_pressure |= unpack_left_shift_u16(src_p[3], 8u, 0x0fu);

    if ((ebs_2_pressure & (1u << 11)) != 0u) {
        ebs_2_pressure |= 0xf000u;
    }

    dst_p->ebs_2_pressure = (int16_t)ebs_2_pressure;
    brakepressure_front = unpack_right_shift_u16(src_p[4], 0u, 0xffu);
    brakepressure_front |= unpack_left_shift_u16(src_p[5], 8u, 0x0fu);

    if ((brakepressure_front & (1u << 11)) != 0u) {
        brakepressure_front |= 0xf000u;
    }

    dst_p->brakepressure_front = (int16_t)brakepressure_front;
    brakepressure_rear = unpack_right_shift_u16(src_p[6], 0u, 0xffu);
    brakepressure_rear |= unpack_left_shift_u16(src_p[7], 8u, 0x0fu);

    if ((brakepressure_rear & (1u << 11)) != 0u) {
        brakepressure_rear |= 0xf000u;
    }

    dst_p->brakepressure_rear = (int16_t)brakepressure_rear;

    return (0);
}

int fsg_0x511_brake_and_ebs_pressure_init(struct fsg_0x511_brake_and_ebs_pressure_t *msg_p)
{
    if (msg_p == NULL) return -1;

    memset(msg_p, 0, sizeof(struct fsg_0x511_brake_and_ebs_pressure_t));

    return 0;
}

int16_t Fsg_0x511_brake_and_ebs_pressure_ebs_1_pressure_encode(float value)
{
    return (int16_t)(value);
}

float Fsg_0x511_brake_and_ebs_pressure_ebs_1_pressure_decode(int16_t value)
{
    return ((float)value);
}

bool Fsg_0x511_brake_and_ebs_pressure_ebs_1_pressure_is_in_range(int16_t value)
{
    return ((value >= 0) && (value <= 100));
}

int16_t Fsg_0x511_brake_and_ebs_pressure_ebs_2_pressure_encode(float value)
{
    return (int16_t)(value);
}

float Fsg_0x511_brake_and_ebs_pressure_ebs_2_pressure_decode(int16_t value)
{
    return ((float)value);
}

bool Fsg_0x511_brake_and_ebs_pressure_ebs_2_pressure_is_in_range(int16_t value)
{
    return ((value >= 0) && (value <= 100));
}

int16_t Fsg_0x511_brake_and_ebs_pressure_brakepressure_front_encode(float value)
{
    return (int16_t)(value);
}

float Fsg_0x511_brake_and_ebs_pressure_brakepressure_front_decode(int16_t value)
{
    return ((float)value);
}

bool Fsg_0x511_brake_and_ebs_pressure_brakepressure_front_is_in_range(int16_t value)
{
    return ((value >= 0) && (value <= 200));
}

int16_t Fsg_0x511_brake_and_ebs_pressure_brakepressure_rear_encode(float value)
{
    return (int16_t)(value);
}

float Fsg_0x511_brake_and_ebs_pressure_brakepressure_rear_decode(int16_t value)
{
    return ((float)value);
}

bool Fsg_0x511_brake_and_ebs_pressure_brakepressure_rear_is_in_range(int16_t value)
{
    return ((value >= 0) && (value <= 200));
}
