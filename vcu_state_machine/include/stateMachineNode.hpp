/**
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * @file StateMachineNode.hpp
 *
 * @brief Defining the state machine node and structs for internal use as well as messages
 *
 * @author Laurin Hesslich <laurin.hesslich@elbflorace.de> 2024-2026
 *
 * @copyright MIT
 *
 * @version 1.0.0
 *
 * @warning DO NOT CHANGE SOMETHING!!!
 *          This package is compliant with the rules from 2025 and will also
 *          be compliant with the future rules, provided nothing changes significantly.
 *          Therefore, no changes must be made to this package!
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

#ifndef STATE_MACHINE_NODE_H
#define STATE_MACHINE_NODE_H

#include "StateMachineData.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ix_msgs/msg/accumulator.hpp"
#include "ix_msgs/msg/inverter.hpp"
#include "ix_msgs/msg/mission.hpp"
#include "ix_msgs/msg/wheels.hpp"
#include "std_msgs/msg/bool.hpp"
#include "vcu_msgs/msg/accel_brake_steer.hpp"
#include "vcu_msgs/msg/assm.hpp"
#include "vcu_msgs/msg/driver_input.hpp"
#include "vcu_msgs/msg/res_input.hpp"
#include "vcu_msgs/msg/vehicle_state.hpp"
#include "vcu_msgs/msg/vehicle_state_debug.hpp"
#include "vcu_msgs/msg/vehicle_state_readable.hpp"
#include "vcu_msgs/msg/watchdog.hpp"

/* ------------------------- Constants ------------------------------------- */

/** Publishing rate of the vehicle state message */
#define VEHICLE_STATE_MSG_RATE 5ms
/** Publishing rate of the vehcile state debug message */
#define DEBUG_MSG_RATE 10ms
/** Convertion from seconds to miliseconds */
#define S_TO_MS 1000
/** Convertion from nanoseconds to miliseconds */
#define NS_TO_MS 1000000
/** Time in ms needed for a button press to be a normal button press */
#define NORMAL_BUTTON_PRESS_THRESHOLD 50
/** Time in ms needed for a button press to be a extended button press */
#define EXTENDED_BUTTON_PRESS_THRESHOLD 1000
/** Time in ms needed for a button press to be a long button press */
#define LONG_BUTTON_PRESS_THRESHOLD 3000

class StateMachineNode : public rclcpp::Node
{
private:
    VehicleStateVector stateVector;

    // Initializing Publisher and Subscriber
    rclcpp::Publisher<vcu_msgs::msg::VehicleState>::SharedPtr vehicleStatePub;
    rclcpp::Publisher<vcu_msgs::msg::VehicleStateDebug>::SharedPtr vehicleStateDebugPub;
    rclcpp::Publisher<vcu_msgs::msg::VehicleStateReadable>::SharedPtr vehicleStateReadablePub;

    // rclcpp::Subscription<vcu_msgs::msg::Inverter>::SharedPtr inverterSub;
    rclcpp::Subscription<ix_msgs::msg::Inverter>::SharedPtr inverterSub;
    rclcpp::Subscription<ix_msgs::msg::Accumulator>::SharedPtr accumulatorSub;
    rclcpp::Subscription<vcu_msgs::msg::ASSM>::SharedPtr assmSub;
    rclcpp::Subscription<vcu_msgs::msg::DriverInput>::SharedPtr driverInputSub;
    rclcpp::Subscription<ix_msgs::msg::Mission>::SharedPtr missionSub;
    rclcpp::Subscription<vcu_msgs::msg::RESInput>::SharedPtr resInputSub;
    rclcpp::Subscription<vcu_msgs::msg::AccelBrakeSteer>::SharedPtr accelBrakeSteerSub;
    rclcpp::Subscription<vcu_msgs::msg::Watchdog>::SharedPtr watchdogSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finishedSub;

    // Global time variables
    uint32_t timeExTSPushed = 0;
    uint32_t timeInTSPushed = 0;
    uint32_t timeR2DPushed = 0;

    rclcpp::TimerBase::SharedPtr timer_state;
    rclcpp::TimerBase::SharedPtr timer_debug;

    void publishVehicleState();
    void publishDebug();
    void publishDebugReadable();

public:
    StateMachineNode();

    void cbWatchdog(const vcu_msgs::msg::Watchdog msg);
    void cbSSBFrontInput(const vcu_msgs::msg::AccelBrakeSteer msg);
    void cbResInput(const vcu_msgs::msg::RESInput msg);
    void cbMissiont(const ix_msgs::msg::Mission msg);
    void cbDISInput(const vcu_msgs::msg::DriverInput msg);
    void cbASSMInput(const vcu_msgs::msg::ASSM msg);
    void cbAccumulatorInput(const ix_msgs::msg::Accumulator msg);
    void cbInverter(const ix_msgs::msg::Inverter msg);
    void cbFinished(const std_msgs::msg::Bool msg);
    void loadParameters();
};

#endif
