#include "motor_driver/MotorDriver.hpp"
#include <cstdint>

namespace motor_driver
{

MotorDriver::MotorDriver(const int motor_id, int socket_descrp)
    : motor_id(motor_id)
    , is_motor_enabled(false)
    , motor_CAN_interface_(socket_descrp)
{
    switch (motor_type_)
    {
    case MotorType::AK80_6_V1:
        std::cout << "Using Motor Type AK80-6 V1" << std::endl;
        current_params_ = default_params::AK80_6_V1_params;
        break;
    case MotorType::AK80_6_V1p1:
        std::cout << "Using Motor Type AK80-6 V1.1" << std::endl;
        current_params_ = default_params::AK80_6_V1p1_params;
        break;
    case MotorType::AK80_6_V2:
        std::cout << "Using Motor Type AK80-6 V2" << std::endl;
        current_params_ = default_params::AK80_6_V2_params;
        break;
    case MotorType::AK80_9_V1p1:
        std::cout << "Using Motor Type AK80-9 V1.1" << std::endl;
        current_params_ = default_params::AK80_9_V1p1_params;
        break;
    case MotorType::AK80_9_V2:
        std::cout << "Using Motor Type AK80-9 V2" << std::endl;
        current_params_ = default_params::AK80_9_V2_params;
        break;
    case MotorType::AK70_10V1p1:
        std::cout << "Using Motor Type AK70-10 V1.1" << std::endl;
        current_params_ = default_params::AK70_10_V1p1_params;
        break;
    case MotorType::AK10_9_V1p1:
        std::cout << "Using Motor Type AK10-9 V1.1" << std::endl;
        current_params_ = default_params::AK10_9_V1p1_params;
        break;
    default:
        perror("Specified Motor Type Not Found!!");
    }
}

motorState MotorDriver::enableMotor()
{
    // Bugfix: To remove the initial kick at motor start.
    // The current working theory is that the motor is not
    // set to zero position when enabled. Hence the
    // last command is executed. So we set zero position
    // and then enable the motor.
    setZeroPosition();
    motorState state;

    // Make it compilable
    state.motor_id = 0;
    state.position = 0;
    state.torque = 0;
    state.velocity = 0;
    state.temperature = 0;
    state.error_code = 0;
    // end

    motor_CAN_interface_.sendCANFrame(this->motor_id, default_msgs::motorEnableMsg);
    // usleep(motorReplyWaitTime);
    // if (motor_CAN_interface_.receiveCANFrame(CAN_reply_msg_))
    // {
    //     state = decodeCANFrame(CAN_reply_msg_);
    //     is_motor_enabled = true;
    // }
    // else
    // {
    //     perror("MotorDriver::enableMotor() Unable to Receive CAN Reply.");
    // }

    // if (this->motor_id != state.motor_id)
    //     perror("MotorDriver::enableMotor() Received message does not have the same "
    //            "motor id!!");

    return state;
}

motorState MotorDriver::disableMotor()
{
    motorState state;
    encodeCANFrame(default_msgs::zeroCmdStruct, CAN_msg_);
    motor_CAN_interface_.sendCANFrame(this->motor_id, CAN_msg_);
    usleep(motorReplyWaitTime);

    if (motor_CAN_interface_.receiveCANFrame(CAN_reply_msg_))
    {
        state = decodeCANFrame(CAN_reply_msg_);
    }
    else
    {
        perror("MotorDriver::disableMotor() Unable to Receive CAN Reply.");
    }

    // Do the actual disabling after zero command.
    motor_CAN_interface_.sendCANFrame(this->motor_id, default_msgs::motorDisableMsg);
    usleep(motorReplyWaitTime);
    if (motor_CAN_interface_.receiveCANFrame(CAN_reply_msg_))
    {
        state = decodeCANFrame(CAN_reply_msg_);
        is_motor_enabled = false;
    }
    else
    {
        perror("MotorDriver::disableMotor() Unable to Receive CAN Reply.");
    }

    if (this->motor_id != state.motor_id)
        perror("MotorDriver::disableMotor() Received message does not have the same "
               "motor id!!");

    return state;
}

motorState MotorDriver::setZeroPosition()
{
    motorState state;

    // Make it compilable
    state.motor_id = 0;
    state.position = 0;
    state.torque = 0;
    state.velocity = 0;
    state.temperature = 0;
    state.error_code = 0;
    // end

    motor_CAN_interface_.sendCANFrame(this->motor_id, default_msgs::motorSetZeroPositionMsg);
    usleep(motorReplyWaitTime);
    //     if (motor_CAN_interface_.receiveCANFrame(CAN_reply_msg_))
    //     {
    //         // state = decodeCANFrame(CAN_reply_msg_);
    //     }
    //     else
    //     {
    //         perror("MotorDriver::setZeroPosition() Unable to Receive CAN Reply.");
    //     }

    //     while (state.position > (1 * (pi / 180)))
    //     {
    //         motor_CAN_interface_.sendCANFrame(this->motor_id, default_msgs::motorSetZeroPositionMsg);
    //         usleep(motorReplyWaitTime);
    //         if (motor_CAN_interface_.receiveCANFrame(CAN_reply_msg_))
    //         {
    //             state = decodeCANFrame(CAN_reply_msg_);
    //         }
    //         else
    //         {
    //             perror("MotorDriver::setZeroPosition() Unable to Receive CAN Reply.");
    //         }
    //     }
    return state;
}

motorState MotorDriver::sendRadCommand(const motorCommand& motor_rad_command)
{
    motorState state;

    // Make it compilable
    state.motor_id = 0;
    state.position = 0;
    state.torque = 0;
    state.velocity = 0;
    state.temperature = 0;
    state.error_code = 0;
    // end

    encodeCANFrame(motor_rad_command, this->CAN_msg_);
    // TODO: Enable enabled check better across multiple objects of this class.
    // if (is_motor_enabled_[cmd_motor_id])
    // {
    //     std::cout << "MotorDriver::sendRadCommand() Motor in disabled state.
    //                   Did you want to really do this?" << std::endl;
    // }
    motor_CAN_interface_.sendCANFrame(this->motor_id, CAN_msg_);
    // usleep(motorReplyWaitTime);
    //  if (motor_CAN_interface_.receiveCANFrame(CAN_reply_msg_))
    //  {
    //      state = decodeCANFrame(CAN_reply_msg_);
    //  }
    //  else
    //  {
    //      perror("MotorDriver::sendRadCommand() Unable to Receive CAN Reply.");
    //  }

    return state;
}

motorState MotorDriver::sendDegreeCommand(const motorCommand& motor_deg_command)
{
    motorCommand command = motor_deg_command;
    command.p_des *= (pi / 180);
    command.v_des *= (pi / 180);

    motorState state = sendRadCommand(command);

    state.position *= (180 / pi);

    return state;
}

const motorParams& MotorDriver::getMotorParams() const { return current_params_; }

void MotorDriver::setMotorParams(const motorParams& new_params) { current_params_ = new_params; }

motorState MotorDriver::decodeCANFrame(const unsigned char* CAN_reply_msg) const
{
    // unpack ints from can buffer
    int id = CAN_reply_msg[0];
    int p_int = (CAN_reply_msg[1] << 8) | CAN_reply_msg[2];
    int v_int = (CAN_reply_msg[3] << 4) | (CAN_reply_msg[4] >> 4);
    int i_int = ((CAN_reply_msg[4] & 0xF) << 8) | CAN_reply_msg[5];
    // convert unsigned ints to floats
    float p = uint_to_float(p_int, current_params_.P_MIN, current_params_.P_MAX, 16);
    float v = uint_to_float(v_int, current_params_.V_MIN, current_params_.V_MAX, 12);
    float i = uint_to_float(i_int, -current_params_.T_MAX, current_params_.T_MAX,
        12); // here -T_MAX, in encode T_MIN

    int8_t temperature = CAN_reply_msg[6];
    uint8_t error_code = CAN_reply_msg[7];

    motorState state { .motor_id = id,
        .position = p * current_params_.AXIS_DIRECTION,
        .velocity = v * current_params_.AXIS_DIRECTION,
        .torque = i * current_params_.AXIS_DIRECTION,
        .temperature = temperature,
        .error_code = error_code
    };

    return state;
}

void MotorDriver::encodeCANFrame(const motorCommand& cmd_to_send, unsigned char* CAN_msg) const
{
    float p_des = cmd_to_send.p_des * current_params_.AXIS_DIRECTION;
    float v_des = cmd_to_send.v_des * current_params_.AXIS_DIRECTION;
    float tau_ff = cmd_to_send.tau_ff * current_params_.AXIS_DIRECTION;

    // Apply Saturation based on the limits
    p_des = fminf(fmaxf(current_params_.P_MIN, p_des), current_params_.P_MAX);
    v_des = fminf(fmaxf(current_params_.V_MIN, v_des), current_params_.V_MAX);
    tau_ff = fminf(fmaxf(current_params_.T_MIN, tau_ff), current_params_.T_MAX);
    float kp = fminf(fmaxf(current_params_.KP_MIN, cmd_to_send.kp), current_params_.KP_MAX);
    float kd = fminf(fmaxf(current_params_.KD_MIN, cmd_to_send.kd), current_params_.KD_MAX);

    // convert floats to unsigned ints
    int p_int = float_to_uint(p_des, current_params_.P_MIN, current_params_.P_MAX, 16);
    int v_int = float_to_uint(v_des, current_params_.V_MIN, current_params_.V_MAX, 12);
    int kp_int = float_to_uint(kp, current_params_.KP_MIN, current_params_.KP_MAX, 12);
    int kd_int = float_to_uint(kd, current_params_.KD_MIN, current_params_.KD_MAX, 12);
    int t_int = float_to_uint(tau_ff, current_params_.T_MIN, current_params_.T_MAX, 12);

    // pack ints into the can message
    CAN_msg[0] = p_int >> 8;
    CAN_msg[1] = p_int & 0xFF;
    CAN_msg[2] = v_int >> 4;
    CAN_msg[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    CAN_msg[4] = kp_int & 0xFF;
    CAN_msg[5] = kd_int >> 4;
    CAN_msg[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    CAN_msg[7] = t_int & 0xff;
}

int MotorDriver::float_to_uint(float x, float x_min, float x_max, int bits) const
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float MotorDriver::uint_to_float(int x_int, float x_min, float x_max, int bits) const
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

} // namespace motor_driver
