/**
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * @file CoolingNode.hpp
 *
 * @brief Defining Cooling node and structs for parameters and messages
 *
 * @author Laurin Hesslich <laurin.hesslich@elbflorace.de>
 *
 * @copyright MIT
 *
 * @version 1.0.0
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

#ifndef COOLING_NODE_H
#define COOLING_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "ix_msgs/msg/accumulator.hpp"
#include "vcu_msgs/msg/cooling_request.hpp"
#include "vcu_msgs/msg/temps.hpp"

#define COOLING_MSG_RATE 100ms

using namespace std::chrono_literals;

/**
 * @brief Parameters of the param file
 */
struct CoolingParameters
{
    float startTempAccuFans;
    float endTempAccuFans;
    float startTempSidewingLeft;
    float endTempSidewingLeft;
    float startTempSidewingRight;
    float endTempSidewingRight;
    float startTempCoolingPump;
    float endTempCoolingPump;
};

/**
 * @struct CoolingRequest
 * @brief Struct for Cooling Request Message
 *
 * @details Struct for saving the cooling requests before transfering them into a message.
 *          Saved as percentage values from 0 - 100 % and later converted to PWM values.
 */
struct CoolingRequest
{
    float accu_cooling; /* Cooling request value for accu fans 0-100 percent            */
    float swl_cooling; /* Cooling request value for sidewing left fan 0-100 percent    */
    float swr_cooling; /* Cooling request value for sidewing right fan 0-100 percent   */
    float cooling_pump; /* Cooling request value for cooling pump 0-100 percent         */
};

/**
 * @class CoolingNode
 *
 * @brief Handling the cooling of the car
 */
class CoolingNode : public rclcpp::Node
{
private:
    CoolingParameters coolingConfig;
    CoolingRequest request;

    /** Subsriber are advertised here: */
    rclcpp::Subscription<ix_msgs::msg::Accumulator>::SharedPtr accumulatorSub;
    rclcpp::Subscription<vcu_msgs::msg::Temps>::SharedPtr tempsSub;

    /** Publisher are advertised here: */
    rclcpp::Publisher<vcu_msgs::msg::CoolingRequest>::SharedPtr coolingRequestPub;

    rclcpp::TimerBase::SharedPtr timer_cooling;

    void publishCoolingRequest();

public:
    CoolingNode();

    void cbAccumulatorInput(const ix_msgs::msg::Accumulator msg);
    void cbTempsInput(const vcu_msgs::msg::Temps msg);
    float findGreatest(float nums[], int size);
    void loadParameters();
};

#endif