#ifndef MMDATA
#define MMDATA

#include "motor_driver/MotorDriver.hpp"
#include "rclcpp/rclcpp.hpp"
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h> /* for close() for socket */

#pragma pack(1)

class MailmanData
{
public:
    rclcpp::Time last_steering_stamp;
    motor_driver::MotorDriver* motorDriver;
    int steering_id = 0x30;

    // ports
    int inverter_port_receive;
    int sock;

    // socket addresses
    struct sockaddr_in sa_inverter_torques;
    struct sockaddr_in sa_inverter_in;

    int fd_can1;
    int fd_can2;
    int fd_can3;
    int fd_can4;

private:
};

typedef struct
{
    // Datasheet table 6.3 - InverterDataOutput

    int32_t setpoint_speed_inv1 = 0; // byte 42
    int32_t setpoint_speed_inv2 = 0;
    int32_t setpoint_speed_inv3 = 0;
    int32_t setpoint_speed_inv4 = 0;

    float torque_limit_pos_motor1 = 0; // byte 58
    float torque_limit_pos_motor2 = 0;
    float torque_limit_pos_motor3 = 0;
    float torque_limit_pos_motor4 = 0;

    float torque_limit_neg_motor1 = 0; // byte 74
    float torque_limit_neg_motor2 = 0;
    float torque_limit_neg_motor3 = 0;
    float torque_limit_neg_motor4 = 0;

    // Datasheet Table 6.4
    uint8_t control_byte_motor1 = 0; // byte 90
    uint8_t control_byte_motor2 = 0;
    uint8_t control_byte_motor3 = 0;
    uint8_t control_byte_motor4 = 0;

    float setpoint_current_id = 0; // byte 94
    float setpoint_current_iq = 0; // byte 98

    // Reserved Bytes
    int8_t b0 = 0x00; // byte 102
    int8_t b1 = 0x00;
    int8_t b2 = 0x00;
    int8_t b3 = 0x00;
    int8_t b4 = 0x00;
    int8_t b5 = 0x00;
    int8_t b6 = 0x00;
    int8_t b7 = 0x00;
    int8_t b8 = 0x00;
    int8_t b9 = 0x00;
    int8_t b10 = 0x00;
    int8_t b11 = 0x00;
    int8_t b12 = 0x00;
    int8_t b13 = 0x00;
    int8_t b14 = 0x00;
    int8_t b15 = 0x00;
    int8_t b16 = 0x00;
    int8_t b17 = 0x00; // byte 119

} Inverter_msg;

/*
This definition is based on the
DATASHEET AND HANDLING GUIDELINES FOR 4 X INVERTER PCB KIT
Revision: 1.03 - April 26, 2024
All values are given in big endian when received from UDP

*/
typedef struct
{
    // Inverter Data Input
    // Datasheet Table 6.6 - 6.8
    int32_t actual_speed_motor1 = 0; // byte 42
    int32_t actual_speed_motor2 = 0;
    int32_t actual_speed_motor3 = 0;
    int32_t actual_speed_motor4 = 0;

    int32_t setpoint_speed_motor1 = 0; // byte 58
    int32_t setpoint_speed_motor2 = 0;
    int32_t setpoint_speed_motor3 = 0;
    int32_t setpoint_speed_motor4 = 0;

    float actual_torque_motor1 = 0; // byte 74
    float actual_torque_motor2 = 0;
    float actual_torque_motor3 = 0;
    float actual_torque_motor4 = 0;

    float setpoint_torque_motor1 = 0; // byte 90
    float setpoint_torque_motor2 = 0;
    float setpoint_torque_motor3 = 0;
    float setpoint_torque_motor4 = 0;

    // 0 - Idle
    // 1 - Drive
    // 2 - Error
    // 3 - config values missing
    uint8_t state_inv1 = 0; // byte 106;
    uint8_t state_inv2 = 0;
    uint8_t state_inv3 = 0;
    uint8_t state_inv4 = 0;

    float actual_temp_motor1 = 0; // byte 110
    float actual_temp_motor2 = 0;
    float actual_temp_motor3 = 0;
    float actual_temp_motor4 = 0;

    // Actual temperature of power switches
    float actual_temp_pwr_module1 = 0; // byte 126
    float actual_temp_pwr_module2 = 0;
    float actual_temp_pwr_module3 = 0;
    float actual_temp_pwr_module4 = 0;

    // Datasheet Table 6.7
    float currently_permitted_output_current_m1 = 0; // byte 142
    float currently_permitted_output_current_m2 = 0;
    float currently_permitted_output_current_m3 = 0;
    float currently_permitted_output_current_m4 = 0;

    // Setpoint for current Id for inverter
    float setpoint_current_id_m1 = 0; // byte 158
    float setpoint_current_id_m2 = 0;
    float setpoint_current_id_m3 = 0;
    float setpoint_current_id_m4 = 0;

    // Actual current value Id for inverter
    float actual_current_id_m1 = 0; // byte 174
    float actual_current_id_m2 = 0;
    float actual_current_id_m3 = 0;
    float actual_current_id_m4 = 0;

    // Setpoint for current Iq for inverter
    float setpoint_current_iq_m1 = 0; // byte 190
    float setpoint_current_iq_m2 = 0;
    float setpoint_current_iq_m3 = 0;
    float setpoint_current_iq_m4 = 0;

    // Actual current value Iq for inverter
    float actual_current_iq_m1 = 0; // byte 206
    float actual_current_iq_m2 = 0;
    float actual_current_iq_m3 = 0;
    float actual_current_iq_m4 = 0;

    float output_voltage_ud_m1 = 0; // byte 222
    float output_voltage_ud_m2 = 0;
    float output_voltage_ud_m3 = 0;
    float output_voltage_ud_m4 = 0;

    float output_voltage_uq_m1 = 0; // byte 238
    float output_voltage_uq_m2 = 0;
    float output_voltage_uq_m3 = 0;
    float output_voltage_uq_m4 = 0;

    float current_dc_link_voltage_12 = 0; // byte 254
    float current_dc_link_voltage_34 = 0; // byte 258
    float input_dc_current_ampere_12 = 0; // byte 262
    float input_dc_current_ampere_34 = 0; // byte 266
    float current_switching_frequency = 0; // byte 270

    // Datasheet Table 6.8
    uint8_t mirror_control_enable_disable_input = 0; // byte 274

    // Actual temperature for additonal sensor channels
    float actual_temp_add_sensor0 = 0; // byte 275
    float actual_temp_add_sensor1 = 0; // byte 279
    float actual_temp_add_sensor2 = 0; // byte 283
    float actual_temp_add_sensor3 = 0; // byte 287
    float actual_temp_add_sensor4 = 0; // byte 291

    // Actual temperature at the edge of HV-Board 1-2
    float actual_temp_edge_hv_12 = 0; // byte 295
    // Actual temperature at the middle of HV-Board 1-2
    float actual_temp_middle_hv_12 = 0; // byte 299
    // Actual temperature at the edge of HV-Board 3-4
    float actual_temp_edge_hv_34 = 0; // byte 303
    // Actual temperature at the middle of HV-Board 1-4
    float actual_temp_middle_hv_14 = 0; // byte 307

    // Actual temperature of the carrier board
    float actual_temp_carrier = 0; // byte 311

    // Actual temperature of CPU
    float actual_temp_cpu = 0; // byte 315
    // Actual temperature of System on Module (top side of board)
    float actual_temp_module_top = 0; // byte 319
    // Actual temperature of System on Module (bottom side of board)
    float actual_temp_module_bottom = 0; // byte 323

    // Actual low voltage current consumption of inverter
    float actual_lv_current = 0; // byte 327
    // Actual power supply voltage of inverter
    float actual_power_supply_voltage = 0; // byte 331

    // Elapsed time since startup of inverter
    float time = 0; // byte 335

    // Reserved padding bytes
    int64_t b1 = 0; // byte 339
    int64_t b8 = 0; // byte 347
    int64_t b16 = 0; // byte 355
    int64_t b24 = 0; // byte 363
    int64_t b32 = 0; // byte 371

} Inverter_receive_data;

#pragma pack()

#endif // MMDATA
