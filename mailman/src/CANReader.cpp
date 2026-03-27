#include "CANReader.hpp"
#include <cstdio>
#include <poll.h>

static inline uint8_t unpack_right_shift_u8(uint8_t value, uint8_t shift, uint8_t mask)
{
    return (uint8_t)((uint8_t)(value & mask) >> shift);
}

// Timeout before cancelling a tried read from a CAN bus
// Polling is necessary as otherwise the read operations
// might block indefinetly making a proper ROS shutdown
// impossible
#define CAN_TIMEOUT 100

void CANReader::receiveThread_can1()
{
    int filedesc = this->mmData->fd_can1;
    struct pollfd poll_fds[1];
    poll_fds[0].fd = filedesc;
    poll_fds[0].events = POLLIN; // = Wait for data to be available

    while (rclcpp::ok())
    {
        // Check if a read is possible in the given timeout
        int ret = poll(poll_fds, 1, CAN_TIMEOUT);
        if (ret == -1)
        {
            perror("Polling error in can1\n");
            continue;
        }
        else if (ret == 0)
        {
            // Read was not possible in specified timeout
            continue;
        }

        struct can_frame frame;
        int bytes_read = read(filedesc, &frame, sizeof(frame));
        (void)bytes_read;

        // Send out generated messages
        this->can1Reader->receive(&frame, bytes_read);
    }
    return;
}

void CANReader::receiveThread_can2()
{
    int filedesc = this->mmData->fd_can2;
    struct pollfd poll_fds[1];
    poll_fds[0].fd = filedesc;
    poll_fds[0].events = POLLIN; // = Wait for data to be available

    while (rclcpp::ok())
    {
        // Check if a read is possible in the given timeout
        int ret = poll(poll_fds, 1, CAN_TIMEOUT);
        if (ret == -1)
        {
            perror("Polling error in can2\n");
            continue;
        }
        else if (ret == 0)
        {
            // Read was not possible in specified timeout
            continue;
        }

        struct can_frame frame;
        int bytes_read = read(filedesc, &frame, sizeof(frame));
        (void)bytes_read;

        // Send out generated messages
        this->can2Reader->receive(&frame, bytes_read);
    }

    return;
}

void CANReader::receiveThread_can3()
{
    int filedesc = this->mmData->fd_can3;
    struct pollfd poll_fds[1];
    poll_fds[0].fd = filedesc;
    poll_fds[0].events = POLLIN; // = Wait for data to be available

    while (rclcpp::ok())
    {
        // Check if a read is possible in the given timeout
        int ret = poll(poll_fds, 1, CAN_TIMEOUT);
        if (ret == -1)
        {
            perror("Polling error in can3\n");
            continue;
        }
        else if (ret == 0)
        {
            // Read was not possible in specified timeout
            continue;
        }

        struct can_frame frame;
        int bytes_read = read(filedesc, &frame, sizeof(frame));

        // Send out generated messages
        this->can3Reader->receive(&frame, bytes_read);
        this->fsgCan3Reader->receive(&frame, bytes_read);
    }
    return;
}

void CANReader::receiveThread_can4()
{
    int filedesc = this->mmData->fd_can4;
    struct pollfd poll_fds[1];
    poll_fds[0].fd = filedesc;
    poll_fds[0].events = POLLIN; // = Wait for data to be available

    while (rclcpp::ok())
    {
        // Check if a read is possible in the given timeout
        int ret = poll(poll_fds, 1, CAN_TIMEOUT);
        if (ret == -1)
        {
            perror("Polling error in can1\n");
            continue;
        }
        else if (ret == 0)
        {
            // Read was not possible in specified timeout
            continue;
        }

        struct can_frame frame;
        int bytes_read = read(filedesc, &frame, sizeof(frame));
        (void)bytes_read;

        // Send out generated messages
        this->can4Reader->receive(&frame, bytes_read);
    }
    return;
}
