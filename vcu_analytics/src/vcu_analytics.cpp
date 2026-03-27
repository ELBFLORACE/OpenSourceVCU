#include "rclcpp/rclcpp.hpp"

#include "Analytics_Data.hpp"

#include "ix_msgs/msg/accumulator.hpp"
#include "ix_msgs/msg/energymeter.hpp"
#include "ix_msgs/msg/float32_stamped.hpp"
#include "vcu_msgs/msg/hardware_analytics.hpp"

#include <boost/circular_buffer.hpp>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

rclcpp::Node::SharedPtr node;
AnalyticsData data;

rclcpp::Publisher<ix_msgs::msg::Wheels>::SharedPtr wheelMotorEfficiencyPublisher;
rclcpp::Publisher<vcu_msgs::msg::HardwareAnalytics>::SharedPtr hardwareAnalyticsPublisher;
rclcpp::Publisher<ix_msgs::msg::Accumulator>::SharedPtr meanedVoltagePublisher;
rclcpp::Publisher<ix_msgs::msg::Float32Stamped>::SharedPtr zeroLoadPublisher;

// Todo : parameterize these buffer lengths
boost::circular_buffer<float> buffer_cvl(20000);
boost::circular_buffer<float> buffer_cvh(20000);
boost::circular_buffer<float> buffer_accuVoltage(20000);

float current_cvl;

void callbackWheelspeeds(const ix_msgs::msg::Wheels::SharedPtr msg) { data.setCurrentWSPD(msg); }

void callbackWheelEfficiency(const ix_msgs::msg::Inverter::SharedPtr msg)
{
    wheelMotorEfficiencyPublisher->publish(data.getWheelMotorEfficiency(msg));
}

void callbackAccu(const ix_msgs::msg::Accumulator::SharedPtr msg)
{
    buffer_cvl.push_back(msg->cell_voltage_low);
    buffer_cvh.push_back(msg->cell_voltage_high);
    current_cvl = msg->cell_voltage_low;
    buffer_accuVoltage.push_back(msg->accumulator_voltage);

    float sum_cvl = std::accumulate(buffer_cvl.begin(), buffer_cvl.end(), 0.0f);
    float sum_cvh = std::accumulate(buffer_cvh.begin(), buffer_cvh.end(), 0.0f);
    float sum_accu = std::accumulate(buffer_accuVoltage.begin(), buffer_accuVoltage.end(), 0.0f);

    float avg_cvl = sum_cvl / static_cast<float>(buffer_cvl.size());
    float avg_cvh = sum_cvh / static_cast<float>(buffer_cvh.size());
    float avg_accuVoltage = sum_accu / static_cast<float>(buffer_accuVoltage.size());

    auto out = ix_msgs::msg::Accumulator();
    out.header.stamp = node->now();
    out.cell_voltage_low = avg_cvl;
    out.cell_voltage_high = avg_cvh;
    out.accumulator_voltage = avg_accuVoltage;

    meanedVoltagePublisher->publish(out);
}

void cbEnergymeter(const ix_msgs::msg::Energymeter::SharedPtr msg)
{
    if (std::abs(msg->ts_current) < 0.1)
    {
        auto out = ix_msgs::msg::Float32Stamped();
        out.header.stamp = node->now();
        out.data = current_cvl;

        zeroLoadPublisher->publish(out);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("vcu_analytics");

    data = AnalyticsData();

    auto motorspeedsSub
        = node->create_subscription<ix_msgs::msg::Wheels>("/sensors/motor_speeds", 1, callbackWheelspeeds);
    auto motorEfficiencySub
        = node->create_subscription<ix_msgs::msg::Inverter>("/vcu/data/inverter", 1, callbackWheelEfficiency);
    auto accuSub = node->create_subscription<ix_msgs::msg::Accumulator>("/vcu/data/accumulator", 1, callbackAccu);
    auto energymeterSub
        = node->create_subscription<ix_msgs::msg::Energymeter>("/vcu/data/energymeter", 1, cbEnergymeter);

    wheelMotorEfficiencyPublisher = node->create_publisher<ix_msgs::msg::Wheels>("/analytics/motorEfficiency", 1);
    hardwareAnalyticsPublisher = node->create_publisher<vcu_msgs::msg::HardwareAnalytics>("/analytics/hardware", 1);
    meanedVoltagePublisher = node->create_publisher<ix_msgs::msg::Accumulator>("/analytics/meanedVoltages", 1);
    zeroLoadPublisher = node->create_publisher<ix_msgs::msg::Float32Stamped>("/analytics/zeroLoadCVL", 1);

    rclcpp::spin(node);
}
