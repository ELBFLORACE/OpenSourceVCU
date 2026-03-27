#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <boost/format.hpp>

#include "vcu_msgs/msg/data_logger.hpp"
#include "vcu_msgs/msg/dis_output.hpp"
#include "vcu_msgs/msg/res_input.hpp"

#include "ix_msgs/msg/wheels.hpp"

#include "Statistic_Data.hpp"

#include <cmath>
#include <fstream>
#include <iostream>

#define LOGGING_INTERVAL 60000 // log vcu stats every minute
#define NS_TO_S 1000000000.0
#define GEAR_RATIO 12.23
#define RPM2MS 2.0 * M_PI * 0.206 / (GEAR_RATIO * 60.0)

rclcpp::TimerBase::SharedPtr loggingTimer;

rclcpp::Node::SharedPtr node;
StatisticData data;
DataLoggerData dlData;

void activateHV(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    (void)response;
    data.beginHV(node->now().seconds());
}

void deactivateHV(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    (void)response;
    data.endHV(node->now().seconds());
}

void callbackRES(const vcu_msgs::msg::RESInput::SharedPtr msg)
{
    dlData.resGoSignal = msg->go_activated;
    dlData.resStop = msg->stop_activated;
    dlData.resQuality = msg->res_radio_quality;
}

void callbackWSPD(const ix_msgs::msg::Wheels::SharedPtr msg)
{
    if (data.getLastDataLoggerTimeStamp() == 0)
    {
        data.setLastDataLoggerTimeStamp(msg->stamp.nanosec);
        return;
    }
    double speed = 0.25 * RPM2MS * (msg->fl + msg->fr + msg->rl + msg->rr);
    double timePassed = (double)(msg->stamp.nanosec - data.getLastDataLoggerTimeStamp()) / NS_TO_S;
    data.setLastDataLoggerTimeStamp(msg->stamp.nanosec);
    if (data.isInHV())
        data.increaseDrivenMeters(speed * timePassed);
}

void appendToCzechDataLoggerFile()
{
    if (!dlData.time_inited)
    {
        dlData.time_inited = true;
        dlData.t = std::time(0);
    }
    std::stringstream ss;
    ss << "/home/devuser/hosthome/vcu_logs/stats/dataLoggerCzech_";
    ss << std::put_time(std::localtime(&dlData.t), "%F_%T");
    ss << ".csv";
    auto filename = ss.str();
    std::replace(filename.begin(), filename.end(), ':', '-');
    std::fstream dataLoggerFile;
    std::fstream checkFile;

    checkFile.open(filename, std::fstream::in);
    dataLoggerFile.open(filename, std::fstream::in | std::fstream::out | std::fstream::app);
    if (!checkFile)
    {
        dataLoggerFile
            << "Speed_actual, Speed_target, Steering_angle_actual, Steering_angle_target, Brake_hydr_actual, "
               "Brake_hydr_target, Motor_moment_actual, Motor_moment_target, Acceleration_longitudinal, "
               "Acceleration_lateral, Yaw_rate, AS_state, EBS_state, AMI_state, Steering_state, ASB_State, "
               "Lap_counter, Cones_count_actual, Cones_count_all, RES_Estop, RES_GoSignal, RES_Radio_Quality, "
               "EBS_Pressure_Front, EBS_Pressure_Rear \n";
    }
    checkFile.close();

    dataLoggerFile << boost::format("%.1f") % dlData.speedActual << ", " << boost::format("%.1f") % dlData.speedTarget
                   << ", " << boost::format("%.1f") % dlData.steeringAngleActual << ", "
                   << boost::format("%.1f") % dlData.steeringAngleTarget << ", "
                   << boost::format("%.1f") % dlData.brakeHydrActual << ", "
                   << boost::format("%.1f") % dlData.brakeHydrTarget << ", "
                   << boost::format("%.1f") % dlData.momentActual << ", " << boost::format("%.1f") % dlData.momentTarget
                   << ", " << boost::format("%.3f") % dlData.accelLong << ", "
                   << boost::format("%.3f") % dlData.accelLat << ", " << boost::format("%.2f") % dlData.yawRate << ", "
                   << boost::format("%.0f") % dlData.asState << ", " << boost::format("%.0f") % dlData.ebsState << ", "
                   << boost::format("%.0f") % dlData.amiState << ", " << boost::format("%.0f") % dlData.steeringState
                   << ", " << boost::format("%.0f") % dlData.absState << ", "
                   << boost::format("%.0f") % dlData.lapCounter << ", "
                   << boost::format("%.0f") % dlData.coneCountActual << ", "
                   << boost::format("%.0f") % dlData.coneCountAll << ", " << boost::format("%.0f") % dlData.resStop
                   << ", " << boost::format("%.0f") % dlData.resGoSignal << ", "
                   << boost::format("%.0f") % dlData.resQuality << ", "
                   << boost::format("%.2f") % dlData.ebsPressureFront << ", "
                   << boost::format("%.2f") % dlData.ebsPressureRear << "\n";

    dataLoggerFile.close();
}

void callbackDataLogger(const vcu_msgs::msg::DataLogger::SharedPtr msg)
{
    dlData.speedActual = (float)msg->speed_actual;
    dlData.speedTarget = (float)msg->speed_target;
    dlData.steeringAngleActual = (float)msg->steering_angle_actual;
    dlData.steeringAngleTarget = ((float)msg->steering_angle_target * 180.0) / M_PI;
    dlData.brakeHydrActual = (float)msg->brake_hydr_actual;
    dlData.brakeHydrTarget = (float)msg->brake_hydr_target;
    dlData.momentActual = (float)msg->motor_moment_actual;
    dlData.momentTarget = (float)msg->motor_moment_target;
    dlData.accelLong = (float)msg->acceleration_longitudinal;
    dlData.accelLat = (float)msg->acceleration_lateral;
    dlData.yawRate = (float)msg->yaw_rate;
    dlData.asState = (float)msg->as_state;
    dlData.ebsState = (float)msg->ebs_state;
    dlData.amiState = (float)msg->ami_state;
    dlData.steeringState = (float)msg->steering_state;
    switch (msg->ebs_state)
    {
    case 1:
    {
        dlData.absState = 1.0f;
        break;
    }
    case 2:
    {
        dlData.absState = 3.0f;
        break;
    }
    case 3:
    {
        dlData.absState = 2.0f;
        break;
    }
    default:
        dlData.absState = 1.0f;
    }
    dlData.lapCounter = (float)msg->lapcounter;
    dlData.coneCountActual = (float)msg->cones_count_actual;
    dlData.coneCountAll = (float)msg->cones_count_all;
    dlData.ebsPressureFront = (float)msg->ebs_front_pressure;
    dlData.ebsPressureRear = (float)msg->ebs_rear_pressure;
    appendToCzechDataLoggerFile();
}

void callbackDIS(const vcu_msgs::msg::DISOutput::SharedPtr msg) { data.setVehicleMode(msg->as_state); }

void appendToStatsFile()
{

    char filename[] = "/home/devuser/hosthome/vcu_logs/stats/vcuStatFile.csv";
    std::fstream statsFile;
    std::fstream checkFile;

    checkFile.open(filename, std::fstream::in);
    statsFile.open(filename, std::fstream::in | std::fstream::out | std::fstream::app);
    if (!checkFile)
    {
        statsFile << "Date, StartTime, RunTime, HVTime, DrivenMeters, VehicleMode \n";
    }
    checkFile.close();

    statsFile << data.getLoggingDateTime() << ", " << data.getRuntimeVCU(node->now().seconds()) << ", "
              << data.getRuntimeHV() << ", " << data.getDrivenMeters() << ", " << data.isInAS() << "\n";

    statsFile.close();
    loggingTimer->reset();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("vcu_statistics");
    data.setStartTime(node->now().seconds());

    auto dataLoggerSub
        = node->create_subscription<vcu_msgs::msg::DataLogger>("/vcu/datalogger/output", 1, callbackDataLogger);
    auto subWspd = node->create_subscription<ix_msgs::msg::Wheels>("/sensors/motor_speeds", 1, callbackWSPD);
    auto displayOutputSub = node->create_subscription<vcu_msgs::msg::DISOutput>("/vcu/dis/output", 1, callbackDIS);
    auto resSub = node->create_subscription<vcu_msgs::msg::RESInput>("/vcu/data/res", 1, callbackRES);

    loggingTimer = node->create_wall_timer(std::chrono::milliseconds(LOGGING_INTERVAL), appendToStatsFile);

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr activateHVTimer
        = node->create_service<std_srvs::srv::Trigger>("vcu_hv_activate", activateHV);
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr deactivateHVTimer
        = node->create_service<std_srvs::srv::Trigger>("vcu_hv_deactivate", deactivateHV);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
