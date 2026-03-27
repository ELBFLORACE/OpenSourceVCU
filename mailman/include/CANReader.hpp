#ifndef CANREADER_HPP
#define CANREADER_HPP

#include <linux/can.h>

#include "MailmanData.hpp"
#include "vcu_msgs/msg/steering_actuator.hpp"

#include "generated/Can1.hpp"
#include "generated/Can2.hpp"
#include "generated/Can3.hpp"
#include "generated/Can4.hpp"
#include "generated/Fsg.hpp"

class CANReader
{
public:
    CANReader(MailmanData* mmData, rclcpp::Node::SharedPtr node,
        rclcpp::Publisher<vcu_msgs::msg::SteeringActuator>::SharedPtr steering_act_pub)
        : mmData(mmData)
        , node(node)
        , steering_act_pub(steering_act_pub)
    {
    }

    Can1Data* can1Reader;
    Can2Data* can2Reader;
    Can3Data* can3Reader;
    Can4Data* can4Reader;
    FsgData* fsgCan3Reader;

    MailmanData* mmData;
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<vcu_msgs::msg::SteeringActuator>::SharedPtr steering_act_pub;
    void receiveThread_can1();
    void receiveThread_can2();
    void receiveThread_can3();
    void receiveThread_can4();

private:
};

#endif // CANREADER_HPP
