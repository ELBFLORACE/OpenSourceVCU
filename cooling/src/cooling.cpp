/**
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * @file cooling.cpp
 *
 * @brief Implementing Cooling node and currently cooling logic
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

#include "CoolingNode.hpp"

using std::placeholders::_1;

/**
 * @brief Constructor for Cooling Node. Creating publisher and subsribes on relevant topics.
 */
CoolingNode::CoolingNode()
    : Node("cooling")
{
    printf("Hello from Cooling\n");

    loadParameters();

    coolingRequestPub = this->create_publisher<vcu_msgs::msg::CoolingRequest>("/vcu/cooling/request", 3);

    accumulatorSub = this->create_subscription<ix_msgs::msg::Accumulator>(
        "/vcu/data/accumulator", 1, std::bind(&CoolingNode::cbAccumulatorInput, this, _1));
    tempsSub = this->create_subscription<vcu_msgs::msg::Temps>(
        "/vcu/temps", 1, std::bind(&CoolingNode::cbTempsInput, this, _1));

    timer_cooling = this->create_wall_timer(COOLING_MSG_RATE, std::bind(&CoolingNode::publishCoolingRequest, this));
}

/**
 * @brief Publishing the cooling request, so that the vcu can send it to the PDU
 */
void CoolingNode::publishCoolingRequest()
{
    vcu_msgs::msg::CoolingRequest coolingMsg;

    coolingMsg.stamp = this->now();

    coolingMsg.accu_cooling = request.accu_cooling;
    coolingMsg.swl_cooling = request.swl_cooling;
    coolingMsg.swr_cooling = request.swr_cooling;
    coolingMsg.cooling_pump = request.cooling_pump;

    coolingRequestPub->publish(coolingMsg);
}

/**
 * @brief Accu cooling logic
 *
 * @param msg BMS Can message containing CTH (CellTempHigh)
 */
void CoolingNode::cbAccumulatorInput(const ix_msgs::msg::Accumulator msg)
{
    float accu_cooling = 0;

    /** Here the cooling request values is calculated: */
    if (msg.cell_temperature_high - coolingConfig.startTempAccuFans > 0)
        accu_cooling = (msg.cell_temperature_high - coolingConfig.startTempAccuFans)
            / (coolingConfig.endTempAccuFans - coolingConfig.startTempAccuFans) * 100;

    /** End of cooling request calculation and queue it for publishing */

    request.accu_cooling = std::clamp(accu_cooling, 0.0f, 100.0f);
}

void CoolingNode::cbTempsInput(const vcu_msgs::msg::Temps msg)
{

    float swl_cooling = 0;
    float swr_cooling = 0;
    float cooling_pump = 0;

    int num_motor_temps = 4;
    float all_motor_temps[4]
        = { msg.actual_temp_motor1, msg.actual_temp_motor2, msg.actual_temp_motor3, msg.actual_temp_motor4 };

    float greatest_motor_temp = findGreatest(all_motor_temps, num_motor_temps);

    if (msg.temp_rad_swl_out - coolingConfig.startTempSidewingLeft > 0)
    {
        swl_cooling = (msg.temp_rad_swl_out - coolingConfig.startTempSidewingLeft)
            / (coolingConfig.endTempSidewingLeft - coolingConfig.startTempSidewingLeft) * 100;
    }

    if (msg.temp_rad_swr_out - coolingConfig.startTempSidewingRight > 0)
    {
        swr_cooling = (msg.temp_rad_swr_out - coolingConfig.startTempSidewingRight)
            / (coolingConfig.endTempSidewingRight - coolingConfig.startTempSidewingRight) * 100;
    }

    if (greatest_motor_temp - coolingConfig.startTempCoolingPump > 0)
    {
        cooling_pump = (greatest_motor_temp - coolingConfig.startTempCoolingPump)
            / (coolingConfig.endTempCoolingPump - coolingConfig.startTempCoolingPump) * 100;
    }

    request.swl_cooling = std::clamp(swl_cooling, 0.0f, 100.0f);
    request.swr_cooling = std::clamp(swr_cooling, 0.0f, 100.0f);
    request.cooling_pump = std::clamp(cooling_pump, 0.0f, 100.0f);
}

/**
 * @brief Load all the parameters conserning the cooling node
 * @return void
 */
void CoolingNode::loadParameters()
{
    this->declare_parameter("startTempAccuFans", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("endTempAccuFans", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("startTempSidewingLeft", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("endTempSidewingLeft", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("startTempSidewingRight", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("endTempSidewingRight", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("startTempCoolingPump", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("endTempCoolingPump", rclcpp::PARAMETER_DOUBLE);

    coolingConfig.startTempAccuFans = this->get_parameter("startTempAccuFans").as_double();
    coolingConfig.endTempAccuFans = this->get_parameter("endTempAccuFans").as_double();
    coolingConfig.startTempSidewingLeft = this->get_parameter("startTempSidewingLeft").as_double();
    coolingConfig.endTempSidewingLeft = this->get_parameter("endTempSidewingLeft").as_double();
    coolingConfig.startTempSidewingRight = this->get_parameter("startTempSidewingRight").as_double();
    coolingConfig.endTempSidewingRight = this->get_parameter("endTempSidewingRight").as_double();
    coolingConfig.startTempCoolingPump = this->get_parameter("startTempCoolingPump").as_double();
    coolingConfig.endTempCoolingPump = this->get_parameter("endTempCoolingPump").as_double();
}

float CoolingNode::findGreatest(float nums[], int size)
{
    int greatest = nums[0];

    for (int i = 1; i < size; i++)
    {
        if (greatest < nums[i])
        {
            greatest = nums[i];
        }
    }
    return greatest;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoolingNode>());
    rclcpp::shutdown();

    return 0;
}
