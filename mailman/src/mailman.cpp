#include "rclcpp/rclcpp.hpp"

#include "CANReader.hpp"
#include "MailmanData.hpp"
#include "motor_driver/MotorDriver.hpp"
#include "vcu_msgs/msg/steering_actuator.hpp"
#include "vcu_shared_lib/enums.hpp"

#include "ix_msgs/msg/float32_stamped.hpp"

#include "std_srvs/srv/trigger.hpp"

#include "vcu_msgs/msg/accumulator_output.hpp"
#include "vcu_msgs/msg/assm_output.hpp"
#include "vcu_msgs/msg/data_logger.hpp"
#include "vcu_msgs/msg/dis_output.hpp"
#include "vcu_msgs/msg/inverter_output.hpp"
#include "vcu_msgs/msg/pdu_output.hpp"
#include "vcu_msgs/msg/vehicle_state.hpp"

#include "vcu_can_msgs_yourcar17/msg/dummy.hpp"
#include "vcu_can_msgs_yourcar17/msg/inverter_data_input.hpp"

#include <algorithm>
#include <cerrno>
#include <string>
#include <thread>

#include <endian.h>

#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include <chrono>
#include <ctime>
#include <ratio>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <eigen3/Eigen/Geometry>
#include <linux/can.h>
#include <linux/can/raw.h>

#pragma pack(1)

rclcpp::Node::SharedPtr node;

// publishers
rclcpp::Publisher<vcu_can_msgs_yourcar17::msg::InverterDataInput>::SharedPtr inverter_data_pub;
rclcpp::Publisher<vcu_msgs::msg::SteeringActuator>::SharedPtr steering_act_pub;

MailmanData* mmData = new MailmanData();
CANReader* canReader;

/*!
 * \brief Send UDP data package.
 * \param data Pointer to the data.
 * \param datasize The size of the data in bytes.
 * \param sa The socket address.
 */
void sendDataUDP(void* data, int datasize, sockaddr_in sa)
{
    int bytes_sent = sendto(mmData->sock, data, datasize, 0, (struct sockaddr*)&sa, sizeof sa);

    if (bytes_sent <= 0)
    {
        printf("Error sending packet: %s\n", strerror(errno));
    }
}

/*!
 * \brief Reverse float endianess.
 * \param inFloat Input.
 * \return The reversed float.
 */
float revFloatEnd(const float inFloat)
{
    // reverse the endianess of a float, for Int use htobe16
    float retVal;
    char* floatToConvert = (char*)&inFloat;
    char* returnFloat = (char*)&retVal;

    // swap the bytes into a temporary buffer
    returnFloat[0] = floatToConvert[3];
    returnFloat[1] = floatToConvert[2];
    returnFloat[2] = floatToConvert[1];
    returnFloat[3] = floatToConvert[0];

    return retVal;
}

// Revert endianess
int32_t revInt32End(const int32_t inInt)
{
    int32_t retVal;
    char* intToConvert = (char*)&inInt;
    char* returnInt = (char*)&retVal;

    returnInt[0] = intToConvert[3];
    returnInt[1] = intToConvert[2];
    returnInt[2] = intToConvert[1];
    returnInt[3] = intToConvert[0];

    return retVal;
}

void callbackDataLogger(const vcu_msgs::msg::DataLogger::SharedPtr msg)
{
    // 0x500
    auto dvDrivingDynamics1Msg = vcu_can_msgs_yourcar17::msg::FsgDvDrivingDynamics1();

    dvDrivingDynamics1Msg.speed_actual = msg->speed_actual;
    dvDrivingDynamics1Msg.speed_target = msg->speed_target;
    dvDrivingDynamics1Msg.steering_angle_actual = msg->steering_angle_actual;
    dvDrivingDynamics1Msg.steering_angle_target = msg->steering_angle_target;
    dvDrivingDynamics1Msg.brake_hydr_actual = msg->brake_hydr_actual;
    dvDrivingDynamics1Msg.brake_hydr_target = msg->brake_hydr_target;
    dvDrivingDynamics1Msg.motor_moment_actual = msg->motor_moment_actual;
    dvDrivingDynamics1Msg.motor_moment_target = msg->motor_moment_target;

    send_on_can(mmData->fd_can3, dvDrivingDynamics1Msg);

    // 0x501
    auto dvDrivingDynamics2Msg = vcu_can_msgs_yourcar17::msg::FsgDvDrivingDynamics2();

    dvDrivingDynamics2Msg.acceleration_longitudinal = msg->acceleration_longitudinal;
    dvDrivingDynamics2Msg.acceleration_lateral = msg->acceleration_lateral;
    dvDrivingDynamics2Msg.yaw_rate = msg->yaw_rate;

    send_on_can(mmData->fd_can3, dvDrivingDynamics2Msg);

    // 0x502
    auto dvSystemStatusMsg = vcu_can_msgs_yourcar17::msg::FsgDvSystemStatus();

    dvSystemStatusMsg.as_state = msg->as_state;
    dvSystemStatusMsg.ebs_state = msg->ebs_state;
    dvSystemStatusMsg.ami_state = msg->ami_state;
    dvSystemStatusMsg.steering_state = msg->steering_state;
    dvSystemStatusMsg.asb_redundancy_state = msg->ebs_redundancy_state;
    dvSystemStatusMsg.lapcounter = msg->lapcounter;
    dvSystemStatusMsg.cones_count_actual = msg->cones_count_actual;
    dvSystemStatusMsg.cones_count_all = msg->cones_count_all;

    send_on_can(mmData->fd_can3, dvSystemStatusMsg);

    /*
    All signals mentioned in the team’s Autonomous System Form (ASF) have to be
    provided within the up to five messages with CAN-IDs 0x511 to 0x515. Each
    message can be up to 8 B of data length. Cycle time is 100 ms
    */

    // 0x511
    auto asfBrakeAndEbsPressureMsg = vcu_can_msgs_yourcar17::msg::FsgBrakeAndEbsPressure();

    asfBrakeAndEbsPressureMsg.ebs_1_pressure = msg->ebs_front_pressure;
    asfBrakeAndEbsPressureMsg.ebs_2_pressure = msg->ebs_rear_pressure;
    asfBrakeAndEbsPressureMsg.brakepressure_front = msg->brakepressure_front;
    asfBrakeAndEbsPressureMsg.brakepressure_rear = msg->brakepressure_rear;

    send_on_can(mmData->fd_can3, asfBrakeAndEbsPressureMsg);
}

void tmotorsStart(int fd, int id)
{
    mmData->motorDriver = new motor_driver::MotorDriver(id, fd);
    mmData->motorDriver->enableMotor();
}

void tmotorsSendPosition(float angle)
{
    motor_driver::motorCommand command = { -angle, 0, 500, 5, 0 };
    mmData->motorDriver->sendRadCommand(command);
}

/*!
 * \brief Initialize the RES
 * \return Always 0
 */
int resInit(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    (void)response;
    int nbytes;

    struct can_frame frame;
    frame.can_id = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x01;
    frame.data[1] = 0x00;
    nbytes = write(mmData->fd_can3, &frame, sizeof(struct can_frame));
    std::cout << "Hello RES" << std::endl;
    if (nbytes < 0)
    {
        perror("Error in RES Startup");
        return nbytes;
    }
    return 0;
}

/*!
 * \brief Initialize the communication on a can bus
 * \param ifname Name of the can bus
 * \return Return the can as int
 */
int initCanBus(const char* ifname)
{
    int fd;
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) == -1)
    {
        perror("Error while opening socket");
        return -1;
    }

    strcpy(ifr.ifr_name, ifname);
    ioctl(fd, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) == -1)
    {
        perror("Error in socket bind");
        return -2;
    }
    return fd;
}

/*!
 * \brief Lateral function callback.
 * \param msg The message.
 * \return void
 */
void cbFuncLat(const ix_msgs::msg::Float32Stamped::SharedPtr msg) { tmotorsSendPosition(msg->data); }

/*!
 * \brief Receive all the data sent by the inverter
 * \return void
 */

// Inverter Input
void receiveThreadInverterDirect()
{
    int sock;
    struct sockaddr_in sa;
    ssize_t recsize;
    socklen_t fromlen;
    Inverter_receive_data data;

    // initialize socket
    memset(&sa, 0, sizeof sa);
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = htonl(INADDR_ANY);
    sa.sin_port = htons((short)mmData->inverter_port_receive);
    fromlen = sizeof sa;
    int broadcast = 1;

    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof broadcast);

    // Add timeout to socket recv so that it doesnt block indefinetly
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 10;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
    // -------

    if (bind(sock, (struct sockaddr*)&sa, sizeof sa) == -1)
    {
        perror("error bind failed");
        close(sock);
    }

    // receive data until ROS is exited
    while (rclcpp::ok())
    {
        recsize = recvfrom(sock, (void*)&data, sizeof(data), 0, (struct sockaddr*)&sa, &fromlen);

        if (recsize < 0)
        {
            // This indicates there was no message available after the specified timeout
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                continue;
            }
            else
            {
                fprintf(stderr, "%s\n", strerror(errno));
            }
        }

        // All values are given in big endian and DV PC is using little endian CPU, therefore endianess needs to be
        // reversed for every value bigger than 1 byte
        {
            // Created by coping the "Inverter_receive_data" struct definition and then search and replacing with the
            // following RegExp's
            //
            // Search: float ([^ ]+) = 0;
            // Replace data.$1 = revFloatEnd(data.$1);
            //
            // Search: int32_t ([^ ]+) = 0;
            // Replace data.$1 = revInt32End(data.$1);

            data.actual_speed_motor1 = revInt32End(data.actual_speed_motor1);
            data.actual_speed_motor2 = revInt32End(data.actual_speed_motor2);
            data.actual_speed_motor3 = revInt32End(data.actual_speed_motor3);
            data.actual_speed_motor4 = revInt32End(data.actual_speed_motor4);

            data.setpoint_speed_motor1 = revInt32End(data.setpoint_speed_motor1);
            data.setpoint_speed_motor2 = revInt32End(data.setpoint_speed_motor2);
            data.setpoint_speed_motor3 = revInt32End(data.setpoint_speed_motor3);
            data.setpoint_speed_motor4 = revInt32End(data.setpoint_speed_motor4);

            data.actual_torque_motor1 = revFloatEnd(data.actual_torque_motor1); // byte 74
            data.actual_torque_motor2 = revFloatEnd(data.actual_torque_motor2);
            data.actual_torque_motor3 = revFloatEnd(data.actual_torque_motor3);
            data.actual_torque_motor4 = revFloatEnd(data.actual_torque_motor4);

            data.setpoint_torque_motor1 = revFloatEnd(data.setpoint_torque_motor1); // byte 90
            data.setpoint_torque_motor2 = revFloatEnd(data.setpoint_torque_motor2);
            data.setpoint_torque_motor3 = revFloatEnd(data.setpoint_torque_motor3);
            data.setpoint_torque_motor4 = revFloatEnd(data.setpoint_torque_motor4);

            /*
            // Don't need to be reversed
            uint8_t state_inv1 = 0; // byte 106;
            uint8_t state_inv2 = 0;
            uint8_t state_inv3 = 0;
            uint8_t state_inv4 = 0;
            */

            data.actual_temp_motor1 = revFloatEnd(data.actual_temp_motor1); // byte 110
            data.actual_temp_motor2 = revFloatEnd(data.actual_temp_motor2);
            data.actual_temp_motor3 = revFloatEnd(data.actual_temp_motor3);
            data.actual_temp_motor4 = revFloatEnd(data.actual_temp_motor4);

            data.actual_temp_pwr_module1 = revFloatEnd(data.actual_temp_pwr_module1); // byte 126
            data.actual_temp_pwr_module2 = revFloatEnd(data.actual_temp_pwr_module2);
            data.actual_temp_pwr_module3 = revFloatEnd(data.actual_temp_pwr_module3);
            data.actual_temp_pwr_module4 = revFloatEnd(data.actual_temp_pwr_module4);

            data.currently_permitted_output_current_m1
                = revFloatEnd(data.currently_permitted_output_current_m1); // byte 142
            data.currently_permitted_output_current_m2 = revFloatEnd(data.currently_permitted_output_current_m2);
            data.currently_permitted_output_current_m3 = revFloatEnd(data.currently_permitted_output_current_m3);
            data.currently_permitted_output_current_m4 = revFloatEnd(data.currently_permitted_output_current_m4);

            data.setpoint_current_id_m1 = revFloatEnd(data.setpoint_current_id_m1); // byte 158
            data.setpoint_current_id_m2 = revFloatEnd(data.setpoint_current_id_m2);
            data.setpoint_current_id_m3 = revFloatEnd(data.setpoint_current_id_m3);
            data.setpoint_current_id_m4 = revFloatEnd(data.setpoint_current_id_m4);

            data.actual_current_id_m1 = revFloatEnd(data.actual_current_id_m1); // byte 174
            data.actual_current_id_m2 = revFloatEnd(data.actual_current_id_m2);
            data.actual_current_id_m3 = revFloatEnd(data.actual_current_id_m3);
            data.actual_current_id_m4 = revFloatEnd(data.actual_current_id_m4);

            data.setpoint_current_iq_m1 = revFloatEnd(data.setpoint_current_iq_m1); // byte 190
            data.setpoint_current_iq_m2 = revFloatEnd(data.setpoint_current_iq_m2);
            data.setpoint_current_iq_m3 = revFloatEnd(data.setpoint_current_iq_m3);
            data.setpoint_current_iq_m4 = revFloatEnd(data.setpoint_current_iq_m4);

            data.actual_current_iq_m1 = revFloatEnd(data.actual_current_iq_m1); // byte 206
            data.actual_current_iq_m2 = revFloatEnd(data.actual_current_iq_m2);
            data.actual_current_iq_m3 = revFloatEnd(data.actual_current_iq_m3);
            data.actual_current_iq_m4 = revFloatEnd(data.actual_current_iq_m4);

            data.output_voltage_ud_m1 = revFloatEnd(data.output_voltage_ud_m1); // byte 222
            data.output_voltage_ud_m2 = revFloatEnd(data.output_voltage_ud_m2);
            data.output_voltage_ud_m3 = revFloatEnd(data.output_voltage_ud_m3);
            data.output_voltage_ud_m4 = revFloatEnd(data.output_voltage_ud_m4);

            data.output_voltage_uq_m1 = revFloatEnd(data.output_voltage_uq_m1); // byte 238
            data.output_voltage_uq_m2 = revFloatEnd(data.output_voltage_uq_m2);
            data.output_voltage_uq_m3 = revFloatEnd(data.output_voltage_uq_m3);
            data.output_voltage_uq_m4 = revFloatEnd(data.output_voltage_uq_m4);

            data.current_dc_link_voltage_12 = revFloatEnd(data.current_dc_link_voltage_12); // byte 254
            data.current_dc_link_voltage_34 = revFloatEnd(data.current_dc_link_voltage_34); // byte 258
            data.input_dc_current_ampere_12 = revFloatEnd(data.input_dc_current_ampere_12); // byte 262
            data.input_dc_current_ampere_34 = revFloatEnd(data.input_dc_current_ampere_34); // byte 266
            data.current_switching_frequency = revFloatEnd(data.current_switching_frequency); // byte 270

            // Don't need to be reversed
            // uint8_t mirror_control_enable_disable_input = 0; // byte 274

            data.actual_temp_add_sensor0 = revFloatEnd(data.actual_temp_add_sensor0); // byte 275
            data.actual_temp_add_sensor1 = revFloatEnd(data.actual_temp_add_sensor1); // byte 279
            data.actual_temp_add_sensor2 = revFloatEnd(data.actual_temp_add_sensor2); // byte 283
            data.actual_temp_add_sensor3 = revFloatEnd(data.actual_temp_add_sensor3); // byte 287
            data.actual_temp_add_sensor4 = revFloatEnd(data.actual_temp_add_sensor4); // byte 291

            data.actual_temp_edge_hv_12 = revFloatEnd(data.actual_temp_edge_hv_12); // byte 295
            data.actual_temp_middle_hv_12 = revFloatEnd(data.actual_temp_middle_hv_12); // byte 299
            data.actual_temp_edge_hv_34 = revFloatEnd(data.actual_temp_edge_hv_34); // byte 303
            data.actual_temp_middle_hv_14 = revFloatEnd(data.actual_temp_middle_hv_14); // byte 307

            data.actual_temp_carrier = revFloatEnd(data.actual_temp_carrier); // byte 311

            data.actual_temp_cpu = revFloatEnd(data.actual_temp_cpu); // byte 315
            data.actual_temp_module_top = revFloatEnd(data.actual_temp_module_top); // byte 319
            data.actual_temp_module_bottom = revFloatEnd(data.actual_temp_module_bottom); // byte 323

            data.actual_lv_current = revFloatEnd(data.actual_lv_current); // byte 327
            data.actual_power_supply_voltage = revFloatEnd(data.actual_power_supply_voltage); // byte 331

            data.time = revFloatEnd(data.time); // byte 335

            // Reserved padding bytes
            /*
            // Don't need to be reversed
            int64_t b1 = 0; // byte 339
            int64_t b8 = 0; // byte 347
            int64_t b16 = 0; // byte 355
            int64_t b24 = 0; // byte 363
            int64_t b32 = 0; // byte 371
             */
        }

        // inverter data for monitoring
        // idm is short for inverter_data_message
        auto idm = vcu_can_msgs_yourcar17::msg::InverterDataInput();
        {
            // Created by coping the "Inverter_receive_data" struct definition and then search and replacing with the
            // following RegExp
            //
            // Search: \w+ ([^ ]+) = 0;
            // Replace idm.$1 = data.$1;

            idm.actual_speed_motor1 = data.actual_speed_motor1; // byte 42
            idm.actual_speed_motor2 = data.actual_speed_motor2;
            idm.actual_speed_motor3 = data.actual_speed_motor3;
            idm.actual_speed_motor4 = data.actual_speed_motor4;

            idm.setpoint_speed_motor1 = data.setpoint_speed_motor1; // byte 58
            idm.setpoint_speed_motor2 = data.setpoint_speed_motor2;
            idm.setpoint_speed_motor3 = data.setpoint_speed_motor3;
            idm.setpoint_speed_motor4 = data.setpoint_speed_motor4;

            idm.actual_torque_motor1 = data.actual_torque_motor1; // byte 74
            idm.actual_torque_motor2 = data.actual_torque_motor2;
            idm.actual_torque_motor3 = data.actual_torque_motor3;
            idm.actual_torque_motor4 = data.actual_torque_motor4;

            idm.setpoint_torque_motor1 = data.setpoint_torque_motor1; // byte 90
            idm.setpoint_torque_motor2 = data.setpoint_torque_motor2;
            idm.setpoint_torque_motor3 = data.setpoint_torque_motor3;
            idm.setpoint_torque_motor4 = data.setpoint_torque_motor4;

            idm.state_inv1 = data.state_inv1; // byte 106;
            idm.state_inv2 = data.state_inv2;
            idm.state_inv3 = data.state_inv3;
            idm.state_inv4 = data.state_inv4;

            idm.actual_temp_motor1 = data.actual_temp_motor1; // byte 110
            idm.actual_temp_motor2 = data.actual_temp_motor2;
            idm.actual_temp_motor3 = data.actual_temp_motor3;
            idm.actual_temp_motor4 = data.actual_temp_motor4;

            // Actual temperature of power switches
            idm.actual_temp_pwr_module1 = data.actual_temp_pwr_module1; // byte 126
            idm.actual_temp_pwr_module2 = data.actual_temp_pwr_module2;
            idm.actual_temp_pwr_module3 = data.actual_temp_pwr_module3;
            idm.actual_temp_pwr_module4 = data.actual_temp_pwr_module4;

            // Datasheet Table 6.7
            idm.currently_permitted_output_current_m1 = data.currently_permitted_output_current_m1; // byte 142
            idm.currently_permitted_output_current_m2 = data.currently_permitted_output_current_m2;
            idm.currently_permitted_output_current_m3 = data.currently_permitted_output_current_m3;
            idm.currently_permitted_output_current_m4 = data.currently_permitted_output_current_m4;

            // Setpoint for current Id for inverter
            idm.setpoint_current_id_m1 = data.setpoint_current_id_m1; // byte 158
            idm.setpoint_current_id_m2 = data.setpoint_current_id_m2;
            idm.setpoint_current_id_m3 = data.setpoint_current_id_m3;
            idm.setpoint_current_id_m4 = data.setpoint_current_id_m4;

            // Actual current value Id for inverter
            idm.actual_current_id_m1 = data.actual_current_id_m1; // byte 174
            idm.actual_current_id_m2 = data.actual_current_id_m2;
            idm.actual_current_id_m3 = data.actual_current_id_m3;
            idm.actual_current_id_m4 = data.actual_current_id_m4;

            // Setpoint for current Iq for inverter
            idm.setpoint_current_iq_m1 = data.setpoint_current_iq_m1; // byte 190
            idm.setpoint_current_iq_m2 = data.setpoint_current_iq_m2;
            idm.setpoint_current_iq_m3 = data.setpoint_current_iq_m3;
            idm.setpoint_current_iq_m4 = data.setpoint_current_iq_m4;

            // Actual current value Iq for inverter
            idm.actual_current_iq_m1 = data.actual_current_iq_m1; // byte 206
            idm.actual_current_iq_m2 = data.actual_current_iq_m2;
            idm.actual_current_iq_m3 = data.actual_current_iq_m3;
            idm.actual_current_iq_m4 = data.actual_current_iq_m4;

            idm.output_voltage_ud_m1 = data.output_voltage_ud_m1; // byte 222
            idm.output_voltage_ud_m2 = data.output_voltage_ud_m2;
            idm.output_voltage_ud_m3 = data.output_voltage_ud_m3;
            idm.output_voltage_ud_m4 = data.output_voltage_ud_m4;

            idm.output_voltage_uq_m1 = data.output_voltage_uq_m1; // byte 238
            idm.output_voltage_uq_m2 = data.output_voltage_uq_m2;
            idm.output_voltage_uq_m3 = data.output_voltage_uq_m3;
            idm.output_voltage_uq_m4 = data.output_voltage_uq_m4;

            idm.current_dc_link_voltage_12 = data.current_dc_link_voltage_12; // byte 254
            idm.current_dc_link_voltage_34 = data.current_dc_link_voltage_34; // byte 258
            idm.input_dc_current_ampere_12 = data.input_dc_current_ampere_12; // byte 262
            idm.input_dc_current_ampere_34 = data.input_dc_current_ampere_34; // byte 266
            idm.current_switching_frequency = data.current_switching_frequency; // byte 270

            idm.mirror_control_enable_disable_input = data.mirror_control_enable_disable_input; // byte 274

            idm.actual_temp_add_sensor0 = data.actual_temp_add_sensor0; // byte 275
            idm.actual_temp_add_sensor1 = data.actual_temp_add_sensor1; // byte 279
            idm.actual_temp_add_sensor2 = data.actual_temp_add_sensor2; // byte 283
            idm.actual_temp_add_sensor3 = data.actual_temp_add_sensor3; // byte 287
            idm.actual_temp_add_sensor4 = data.actual_temp_add_sensor4; // byte 291

            idm.actual_temp_edge_hv_12 = data.actual_temp_edge_hv_12; // byte 295
            idm.actual_temp_middle_hv_12 = data.actual_temp_middle_hv_12; // byte 299
            idm.actual_temp_edge_hv_34 = data.actual_temp_edge_hv_34; // byte 303
            idm.actual_temp_middle_hv_14 = data.actual_temp_middle_hv_14; // byte 307

            idm.actual_temp_carrier = data.actual_temp_carrier; // byte 311

            idm.actual_temp_cpu = data.actual_temp_cpu; // byte 315
            idm.actual_temp_module_top = data.actual_temp_module_top; // byte 319
            idm.actual_temp_module_bottom = data.actual_temp_module_bottom; // byte 323

            idm.actual_lv_current = data.actual_lv_current; // byte 327
            idm.actual_power_supply_voltage = data.actual_power_supply_voltage; // byte 331

            idm.time = data.time; // byte 335
        }
        inverter_data_pub->publish(idm);
    }
}

/*!
 * \brief Callback on the inverter output request sent by the vcu
 * \param msg The message containing the entire request message
 * \return void
 */

void callbackInverterRequest(const vcu_msgs::msg::InverterOutput::SharedPtr msg)
{

    Inverter_msg message;

    // Assemble control byte
    uint8_t inv1_control = (uint8_t)0x00; // Std: current_ctrl: off, asc_allowed: on, reset: on, enabled: on
    uint8_t inv2_control = (uint8_t)0x00; // Std: current_ctrl: off, asc_allowed: on, reset: on, enabled: on
    uint8_t inv3_control = (uint8_t)0x00; // Std: current_ctrl: off, asc_allowed: on, reset: on, enabled: on
    uint8_t inv4_control = (uint8_t)0x00; // Std: current_ctrl: off, asc_allowed: on, reset: on, enabled: on
    inv1_control = ((bool)msg->current_control.rr << 3) | ((bool)msg->asc_allowed.rr << 2) | ((bool)msg->reset.rr << 1)
        | (bool)msg->enabled.rr;
    inv2_control = ((bool)msg->current_control.rl << 3) | ((bool)msg->asc_allowed.rl << 2) | ((bool)msg->reset.rl << 1)
        | (bool)msg->enabled.rl;
    inv3_control = ((bool)msg->current_control.fl << 3) | ((bool)msg->asc_allowed.fl << 2) | ((bool)msg->reset.fl << 1)
        | (bool)msg->enabled.fl;
    inv4_control = ((bool)msg->current_control.fr << 3) | ((bool)msg->asc_allowed.fr << 2) | ((bool)msg->reset.fr << 1)
        | (bool)msg->enabled.fr;

    message.setpoint_speed_inv1 = revInt32End((int32_t)msg->wheelspeed_setpoints.rr);
    message.setpoint_speed_inv2 = revInt32End((int32_t)msg->wheelspeed_setpoints.rl);
    message.setpoint_speed_inv3 = revInt32End((int32_t)msg->wheelspeed_setpoints.fl);
    message.setpoint_speed_inv4 = revInt32End((int32_t)msg->wheelspeed_setpoints.fr);

    message.torque_limit_pos_motor1 = revFloatEnd((float)(msg->upper_torque_bounds.rr));
    message.torque_limit_pos_motor2 = revFloatEnd((float)(msg->upper_torque_bounds.rl));
    message.torque_limit_pos_motor3 = revFloatEnd((float)(msg->upper_torque_bounds.fl));
    message.torque_limit_pos_motor4 = revFloatEnd((float)(msg->upper_torque_bounds.fr));

    message.torque_limit_neg_motor1 = revFloatEnd((float)(msg->lower_torque_bounds.rr));
    message.torque_limit_neg_motor2 = revFloatEnd((float)(msg->lower_torque_bounds.rl));
    message.torque_limit_neg_motor3 = revFloatEnd((float)(msg->lower_torque_bounds.fl));
    message.torque_limit_neg_motor4 = revFloatEnd((float)(msg->lower_torque_bounds.fr));

    message.control_byte_motor1 = inv1_control;
    message.control_byte_motor2 = inv2_control;
    message.control_byte_motor3 = inv3_control;
    message.control_byte_motor4 = inv4_control;

    message.setpoint_current_id = revFloatEnd(0.0);
    message.setpoint_current_iq = revFloatEnd(0.0);

    sendDataUDP((void*)&message, sizeof(message), mmData->sa_inverter_torques);
}

void inverterStart()
{
    Inverter_msg message;
    sendDataUDP((void*)&message, sizeof(message), mmData->sa_inverter_torques);
}

void callbackPDU(const vcu_msgs::msg::PDUOutput::SharedPtr msg)
{
    // auto vcuPowerControlMsg = vcu_can_msgs_yourcar17::msg::Dummy();
    // vcuPowerControlMsg.pdu_power_inverter = msg->power_inverter;
    // vcuPowerControlMsg.pdu_power_rtds = msg->power_rtds;
    // vcuPowerControlMsg.pdu_power_brakelight = msg->power_brakelight;
    // vcuPowerControlMsg.pdu_power_sdc = msg->power_sdc;
    // vcuPowerControlMsg.pdu_power_ods = msg->power_ods;
    // vcuPowerControlMsg.pdu_power_res_racing_mode = msg->power_res_racing_mode;
    // vcuPowerControlMsg.pdu_power_can_logger = msg->power_can_logger;
    // vcuPowerControlMsg.pdu_power_lidar_utr = msg->power_lidar_swr;
    // vcuPowerControlMsg.pdu_power_lidar_utl = msg->power_lidar_swl;
    // vcuPowerControlMsg.pdu_power_lidar_mh = msg->power_lidar_mh;
    // vcuPowerControlMsg.pdu_power_tire_temp = msg->power_tire_temp;
    // vcuPowerControlMsg.pdu_power_res = msg->power_res;
    // vcuPowerControlMsg.pdu_enable_lv_dcdc_actuator = msg->power_lv_steering_actuator;
    // vcuPowerControlMsg.pdu_power_assi = msg->power_assi;
    // vcuPowerControlMsg.pdu_power_rad_fan_l = msg->power_fan_swl;
    // vcuPowerControlMsg.pdu_power_schenkler_ogss = msg->power_ogss_schenkler;
    // vcuPowerControlMsg.pdu_power_rad_fan_r = msg->power_fan_swr;
    // vcuPowerControlMsg.pdu_enable_lv_dcdc_12_1 = msg->power_lv_dcdc_12;
    // vcuPowerControlMsg.pdu_enable_lv_dcdc_12_2 = msg->power_lv_dcdc_12_2;
    // vcuPowerControlMsg.pdu_enable_lv_dcdc_pc = msg->power_lv_dcdc_pc;
    // vcuPowerControlMsg.pdu_enable_hv_dcdc_accu = msg->power_lv_dcdc_akku;
    // vcuPowerControlMsg.pdu_power_driver_communication = msg->power_driver_communication;
    // vcuPowerControlMsg.pdu_enable_push_to_talk = msg->enable_push_to_talk;
    // vcuPowerControlMsg.pdu_power_ssb_front = msg->power_ssb;

    // send_on_can(mmData->fd_can1, vcuPowerControlMsg);

    // auto vcuPwmControlMsg = vcu_can_msgs_yourcar17::msg::Dummy();
    // vcuPwmControlMsg.pdu_pwm_free_1 = 0;
    // vcuPwmControlMsg.pdu_pwm_free_2 = 0;
    // vcuPwmControlMsg.pdu_pwm_free_3 = 0;
    // vcuPwmControlMsg.pdu_pwm_cooling_pump = msg->pwm_cooling_pump;
    // vcuPwmControlMsg.pdu_pwm_ebs_pump = msg->pwm_ebs_pump;
    // vcuPwmControlMsg.pdu_pwm_mono_rear_lid_fans = msg->pwm_accu_fan;
    // vcuPwmControlMsg.pdu_pwm_rad_fan_l = msg->pwm_swl_fan;
    // vcuPwmControlMsg.pdu_pwm_rad_fan_r = msg->pwm_swr_fan;

    // send_on_can(mmData->fd_can1, vcuPwmControlMsg);
}

void callbackASSM(const vcu_msgs::msg::ASSMOutput::SharedPtr msg)
{
    // auto canMsg = vcu_can_msgs_yourcar17::msg::Dummy();
    // canMsg.as_state = msg->as_state;
    // canMsg.watchdog_assm = msg->watchdog_assm;
    // canMsg.ebs_power_stage1 = msg->ebs_power_stage_front;
    // canMsg.ebs_power_stage2 = msg->ebs_power_stage_rear;
    // canMsg.assm_sdc_close_cmd = msg->assm_sdc_close_cmd;

    // send_on_can(mmData->fd_can3, canMsg);
}

void sendDisOutput(const vcu_msgs::msg::DISOutput::SharedPtr msg)
{
    // auto pedalsPercentMsg = vcu_can_msgs_yourcar17::msg::Dummy();
    // auto vcuControlsMsg = vcu_can_msgs_yourcar17::msg::Dummy();
    // auto vcuParametersMsg = vcu_can_msgs_yourcar17::msg::Dummy();
    // auto displayNavigationMsg = vcu_can_msgs_yourcar17::msg::Dummy();

    // // Pedal percent
    // {
    //     pedalsPercentMsg.dis_accel_pedal_percent = std::clamp(msg->accel_pedal * 100.0, 0.0, 100.0);
    //     pedalsPercentMsg.dis_brake_pedal_percent = std::clamp(msg->reku_pedal * 100.0, 0.0, 100.0);
    // }

    // // VCU Controls
    // {
    //     vcuControlsMsg.state_push_to_talk = msg->push_to_talk_state;
    //     vcuControlsMsg.state_reku = msg->reku_enabled;
    // }

    // // Dis parameters
    // {
    //     vcuParametersMsg.config_max_torque_plus = msg->torque_upper_bound;
    //     vcuParametersMsg.config_max_torque_minus = msg->torque_lower_bound;
    //     vcuParametersMsg.power_limit = msg->power_limit;
    //     vcuParametersMsg.wheel_rpm_limit = msg->rpm_limit;

    //     vcuParametersMsg.slip_acceleration = msg->slip_acceleration;
    //     vcuParametersMsg.slip_braking = msg->slip_braking;

    //     vcuParametersMsg.config_drs = 0;

    //     vcuParametersMsg.vehicle_control_mode = msg->as_state;
    //     vcuParametersMsg.vehicle_mode = msg->selected_mission;

    //     vcuParametersMsg.config_enable_asr = msg->asr_enabled;
    //     vcuParametersMsg.config_enable_bsr = msg->bsr_enabled;
    // }

    // // Display navigation logic
    // {
    //     displayNavigationMsg.display_lock_screen = msg->display_lock_screen;
    //     displayNavigationMsg.display_navigate_left = msg->display_navigate_left;
    //     displayNavigationMsg.display_navigate_right = msg->display_navigate_right;
    //     displayNavigationMsg.display_select_screen = msg->display_select_screen;
    // }

    // send_on_can(mmData->fd_can1, pedalsPercentMsg);
    // send_on_can(mmData->fd_can1, vcuControlsMsg);
    // send_on_can(mmData->fd_can1, vcuParametersMsg);
    // send_on_can(mmData->fd_can1, displayNavigationMsg);
}

void callbackAccumulator(const vcu_msgs::msg::AccumulatorOutput::SharedPtr msg)
{
    // auto canMsg = vcu_can_msgs_yourcar17::msg::Dummy();
    // canMsg.bmc_hv_permission = msg->hv_enabled;

    // send_on_can(mmData->fd_can2, canMsg);
}

void steeringInit(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    (void)response;

    tmotorsStart(mmData->fd_can3, mmData->steering_id);
}

/*!
 * \brief Load all of the ros parameters specified for the mailman
 * \param node The mailman node so that the parameters can be read and assigned
 * \param ros2ParamTriple ROS2 parameters are stored in a triple containing variable, topic name, default value
 * \return true if all parameters were set, false otherwise
 */
bool loadParams(rclcpp::Node::SharedPtr node, std::vector<std::tuple<std::string, int*, int>>& ros2ParamTriple)
{
    // some wrapper used for loading ros2 params since rclcpp is overly complicated
    // also gives a warning when some params are missing

    std::vector<rclcpp::Parameter> params;
    std::vector<std::string> failedParams;
    for (int i = 0; (size_t)i < ros2ParamTriple.size(); ++i)
    {
        rclcpp::Parameter p(std::get<0>(ros2ParamTriple[i]));
        node->declare_parameter(p.get_name(), std::get<2>(ros2ParamTriple[i]));
        params.push_back(p);
    }
    for (int i = 0; (size_t)i < ros2ParamTriple.size(); ++i)
    {
        if (!node->get_parameter(params[i].get_name(), *(std::get<1>(ros2ParamTriple[i]))))
        {
            *std::get<1>(ros2ParamTriple[i]) = std::get<2>(ros2ParamTriple[i]);
            failedParams.push_back(std::get<0>(ros2ParamTriple[i]));
        }
    }
    std::stringstream ssFailedTopics;
    for (int i = 0; (size_t)i < failedParams.size(); ++i)
    {
        ssFailedTopics << failedParams[i];
        if ((size_t)i != (failedParams.size() - 1))
        {
            ssFailedTopics << ", ";
        }
    }

    bool allParamsSet = (failedParams.size() == 0);
    if (!allParamsSet)
    {
        std::string missingParamsString = ssFailedTopics.str();
        RCLCPP_ERROR_STREAM(node->get_logger(), "Some params(s) are not set! Missing params: " << missingParamsString);
    }

    return allParamsSet;
}

// main function
int main(int argc, char** argv)
{
    std::string inverter_ip;
    int inverter_port_setpoints;

    // initialize ROS
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("mailman");

    inverter_data_pub
        = node->create_publisher<vcu_can_msgs_yourcar17::msg::InverterDataInput>("/sensor/inverter/data_input", 1);
    steering_act_pub = node->create_publisher<vcu_msgs::msg::SteeringActuator>("/sensors/steering_actuator", 10);

    // init subscribers
    canReader = new CANReader(mmData, node, steering_act_pub);

    canReader->can1Reader = new Can1Data(node);
    can1DataInitializePublishers(canReader->can1Reader, node);
    canReader->can2Reader = new Can2Data(node);
    can2DataInitializePublishers(canReader->can2Reader, node);
    canReader->can3Reader = new Can3Data(node);
    can3DataInitializePublishers(canReader->can3Reader, node);
    canReader->fsgCan3Reader = new FsgData(node);
    fsgDataInitializePublishers(canReader->fsgCan3Reader, node);
    canReader->can4Reader = new Can4Data(node);
    can4DataInitializePublishers(canReader->can4Reader, node);

    auto latsub = node->create_subscription<ix_msgs::msg::Float32Stamped>("/vcu/steering/output", 1, cbFuncLat);
    auto inverter_request_sub
        = node->create_subscription<vcu_msgs::msg::InverterOutput>("/vcu/inverter/output", 1, callbackInverterRequest);
    auto pduSub = node->create_subscription<vcu_msgs::msg::PDUOutput>("/vcu/pdu/output", 1, callbackPDU);
    auto assmSub = node->create_subscription<vcu_msgs::msg::ASSMOutput>("/vcu/assm/output", 1, callbackASSM);
    auto accumulatorSub = node->create_subscription<vcu_msgs::msg::AccumulatorOutput>(
        "/vcu/accumulator/output", 1, callbackAccumulator);
    auto dataLoggerSub
        = node->create_subscription<vcu_msgs::msg::DataLogger>("/vcu/datalogger/output", 1, callbackDataLogger);
    auto disOutputSub = node->create_subscription<vcu_msgs::msg::DISOutput>("/vcu/dis/output", 1, sendDisOutput);

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resInitServie
        = node->create_service<std_srvs::srv::Trigger>("vcu_res_initialize", resInit);

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr steeringInitService
        = node->create_service<std_srvs::srv::Trigger>("vcu_steering_initialize", steeringInit);

    std::vector<std::tuple<std::string, int*, int>> ros2ParamTriple;
    ros2ParamTriple.push_back(
        { "/mailman/inverter/ports/receive", &mmData->inverter_port_receive, 42442 }); // new Port of Inverter
    ros2ParamTriple.push_back({ "/mailman/inverter/ports/setpoints", &inverter_port_setpoints, 42422 });

    bool allParamsSet = loadParams(node, ros2ParamTriple);
    (void)allParamsSet;

    node->declare_parameter("/mailman/inverter/ip_address", std::string("192.168.1.42"));

    if (!node->get_parameter("/mailman/inverter/ip_address", inverter_ip))
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Missing input topic!");
    }

    // create an Internet, datagram, socket using UDP
    mmData->sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (mmData->sock == -1)
    {
        // if socket failed to initialize, exit
        printf("Error Creating Socket");
    }

    // IPv4 adresses is a uint32_t, convert a string representation of the octets to the appropriate value
    mmData->sa_inverter_torques.sin_addr.s_addr = inet_addr(inverter_ip.c_str());
    mmData->sa_inverter_torques.sin_port = htons((short)inverter_port_setpoints);

    mmData->last_steering_stamp = rclcpp::Time(0);

    mmData->fd_can1 = initCanBus("can0"); // can1
    mmData->fd_can2 = initCanBus("can1"); // can2
    mmData->fd_can3 = initCanBus("can2"); // can3
    mmData->fd_can4 = initCanBus("can3"); // can4

    inverterStart();

    std::thread receiveCan1(&CANReader::receiveThread_can1, canReader);
    std::thread receiveCan2(&CANReader::receiveThread_can2, canReader);
    std::thread receiveCan3(&CANReader::receiveThread_can3, canReader);
    std::thread receiveCan4(&CANReader::receiveThread_can4, canReader);
    std::thread receiveInverter(receiveThreadInverterDirect);

    printf("Start spin\n");
    rclcpp::spin(node);

    printf("spin end\n");

    receiveCan1.join();
    printf("receive 1 stopped\n");
    receiveCan2.join();
    printf("receive 2 stopped\n");
    receiveCan3.join();
    printf("receive 3 stopped\n");
    receiveCan4.join();
    printf("receive 4 stopped\n");
    receiveInverter.join();
    printf("receive Inverter stopped\n");
    close(mmData->sock); /* close the socket */
    printf("socket closed\n");

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
