#include "motor_driver/CANInterface.hpp"

namespace CAN_interface
{
CANInterface::CANInterface(int socket_descrp) { this->socket_descrp_ = socket_descrp; }

bool CANInterface::sendCANFrame(int can_id, const unsigned char* CANMsg)
{
    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = 8;
    memcpy(frame.data, CANMsg, 8);

    if (write(socket_descrp_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("CANInterface: Error writing to CAN Interface.");
        return false;
    }
    else
    {
        return true;
    }
}

bool CANInterface::receiveCANFrame(unsigned char* CANMsg)
{
    // Listen to all CAN messages. Filter by Motor ID later in the motor driver class.
    struct can_frame frame;

    if (read(socket_descrp_, &frame, sizeof(struct can_frame)) < 0)
    {
        perror("CANInterface: Error Reading Data.");
        return false;
    }
    else
    {
        memcpy(CANMsg, frame.data, frame.can_dlc);
        return true;
    }
}

CANInterface::~CANInterface()
{

    if (close(socket_descrp_) < 0)
    {
        perror("CANInterface: Error Closing CAN Socket.");
    }
}
}