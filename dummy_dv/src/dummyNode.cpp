#include "rclcpp/rclcpp.hpp"
#include <cmath>

#include "std_msgs/msg/bool.hpp"

#include "ix_msgs/msg/float32_stamped.hpp"
#include "ix_msgs/msg/inverter_setpoints.hpp"

#include "vcu_msgs/msg/accel_brake_steer.hpp"

#include "vcu_shared_lib/wheels.hpp"

rclcpp::TimerBase::SharedPtr initialWaitTimer;
rclcpp::TimerBase::SharedPtr inspectionTimer;
rclcpp::TimerBase::SharedPtr drivingTimer;

bool inDriving = false;
bool allowedToDrive = false;
bool finished = false;
double iSteering = 0.0;

rclcpp::Node::SharedPtr node;
rclcpp::Publisher<ix_msgs::msg::InverterSetpoints>::SharedPtr pubControlsOutput;
rclcpp::Publisher<ix_msgs::msg::Float32Stamped>::SharedPtr pubSteeringOutput;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finishedPub; //+++ Todo: Change to service +++//

void triggerInspectionEnd()
{
    inDriving = false;
    allowedToDrive = false;
    finished = true;
    auto finishedState = std_msgs::msg::Bool();
    finishedState.data = true;
    finishedPub->publish(finishedState);
}

void triggerInspectionStart()
{
    if (allowedToDrive)
    {
        if (!inDriving)
        {
            inDriving = true;
            inspectionTimer->reset();
            drivingTimer->reset();
        }
    }
}

void calculateControls()
{
    if (!inDriving)
    {
        inspectionTimer->reset();
        return;
    }
    auto inverterOut = ix_msgs::msg::InverterSetpoints();
    if (inDriving)
    {
        inverterOut.wheelspeed_setpoints = Wheels<int>(500, 500, 500, 500).to_msg();
        inverterOut.upper_torque_bounds = Wheels<int>(5, 5, 5, 5).to_msg();
        inverterOut.lower_torque_bounds = Wheels<int>(0, 0, 0, 0).to_msg();
    }
    else
    {
        inverterOut.wheelspeed_setpoints = Wheels<int>(500, 500, 500, 500).to_msg();
        inverterOut.upper_torque_bounds = Wheels<int>(0, 0, 0, 0).to_msg();
        inverterOut.lower_torque_bounds = Wheels<int>(0, 0, 0, 0).to_msg();
    }
    pubControlsOutput->publish(inverterOut);

    auto steeringOut = ix_msgs::msg::Float32Stamped();

    steeringOut.data = 1.0472 * sin(0.01 * iSteering);
    iSteering += 1.0;
    pubSteeringOutput->publish(steeringOut);

    auto finishedState = std_msgs::msg::Bool();
    finishedState.data = false;
    finishedPub->publish(finishedState);
    drivingTimer->reset();
}

void enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if ((bool)msg->data && !finished)
    {
        allowedToDrive = true;
        if (!allowedToDrive)
        {
            initialWaitTimer->reset();
        }
    }
    else
    {
        allowedToDrive = false;
        initialWaitTimer->reset();
        inspectionTimer->reset();
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("dummy_inspection_node");

    inspectionTimer = node->create_wall_timer(std::chrono::milliseconds(30000), triggerInspectionEnd);
    initialWaitTimer = node->create_wall_timer(std::chrono::milliseconds(3000), triggerInspectionStart);
    drivingTimer = node->create_wall_timer(std::chrono::milliseconds(10), calculateControls);

    auto dvDriveEnableSub = node->create_subscription<std_msgs::msg::Bool>("/vcu/dv/allowance", 1, enableCallback);

    pubControlsOutput = node->create_publisher<ix_msgs::msg::InverterSetpoints>("/controls/inverter/request", 1);
    pubSteeringOutput = node->create_publisher<ix_msgs::msg::Float32Stamped>("/controls/steering/request", 1);
    finishedPub = node->create_publisher<std_msgs::msg::Bool>("/dv/mission/state", 1);
    rclcpp::spin(node);
}
