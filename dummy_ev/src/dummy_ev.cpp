#include "rclcpp/rclcpp.hpp"
#include <cmath>

#include "ix_msgs/msg/inverter_setpoints.hpp"

#include "vcu_msgs/msg/accel_brake_steer.hpp"

#include "EV_Data.hpp"

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<ix_msgs::msg::InverterSetpoints>::SharedPtr pubControlsOutput;
EV_Data evData = EV_Data();

void pedalPositionCalculation(float apps1, float apps2, float brakeforce, Wheels<float>* desiredRPM,
    Wheels<float>* desiredTorqueLowerBound, Wheels<float>* desiredTorqueUpperBound)
{
    auto configApps = evData.getAPPSCalibration();
    double relative_pos_apps1
        = (apps1 - configApps.apps1.released) / (configApps.apps1.pressed - configApps.apps1.released);
    double relative_pos_apps2
        = (apps2 - configApps.apps2.released) / (configApps.apps2.pressed - configApps.apps2.released);

    float gaspedal = (relative_pos_apps1 + relative_pos_apps2) / 2.0;
    float rekupedal = 0.0;

    if (brakeforce > 250.0)
    {
        rekupedal = (brakeforce - 250) / 690.0;
    }

    rekupedal = std::clamp(rekupedal, 0.0f, 1.0f);
    gaspedal = std::clamp(gaspedal, 0.0f, 1.0f);

    if (rekupedal > 0.0)
    {
        float torque = -5.0;
        *desiredTorqueLowerBound = Wheels<float>(torque, torque, torque, torque);
    }
    else if (gaspedal > 0.0)
    {
        float rpm = 5000.0 * gaspedal;
        float torque = 5.0;
        *desiredRPM = Wheels<float>(rpm, rpm, rpm, rpm);
        *desiredTorqueUpperBound = Wheels<float>(torque, torque, torque, torque);
    }
}

void callbackAccelBrakeSteer(const vcu_msgs::msg::AccelBrakeSteer::SharedPtr msg)
{
    APPS appsCalibration;
    appsCalibration.apps1.pressed = msg->apps1_pressed;
    appsCalibration.apps1.released = msg->apps1_released;
    appsCalibration.apps2.pressed = msg->apps2_pressed;
    appsCalibration.apps2.released = msg->apps2_released;
    evData.setAppsCalibration(appsCalibration);

    Wheels<float> desiredRPM(0.0, 0.0, 0.0, 0.0);
    Wheels<float> desiredTorqueLowerBound(0.0, 0.0, 0.0, 0.0);
    Wheels<float> desiredTorqueUpperBound(0.0, 0.0, 0.0, 0.0);

    pedalPositionCalculation(
        msg->apps1, msg->apps2, msg->brakeforce, &desiredRPM, &desiredTorqueLowerBound, &desiredTorqueUpperBound);

    auto out = ix_msgs::msg::InverterSetpoints();
    out.wheelspeed_setpoints = desiredRPM.to_msg();
    out.lower_torque_bounds = desiredTorqueLowerBound.to_msg();
    out.upper_torque_bounds = desiredTorqueUpperBound.to_msg();
    pubControlsOutput->publish(out);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("dummy_ev");
    auto accelBrakeSteerSub = node->create_subscription<vcu_msgs::msg::AccelBrakeSteer>(
        "/vcu/data/accelBrakeSteer", 1, callbackAccelBrakeSteer);

    pubControlsOutput = node->create_publisher<ix_msgs::msg::InverterSetpoints>("/controls/inverter/request", 1);
    rclcpp::spin(node);
}
