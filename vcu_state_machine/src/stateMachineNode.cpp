/**
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * @file stateMachineNode.cpp
 *
 * @brief Implementation of the state machine node
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

#include "stateMachineNode.hpp"
#include "Utils.hpp"

using std::placeholders::_1;

StateMachineNode::StateMachineNode()
    : Node("stateMachineNode")
{
    printf("Hello from State Machine\n");
    stateVector = VehicleStateVector();

    loadParameters();

    // Publisher for vehicle state
    vehicleStatePub = this->create_publisher<vcu_msgs::msg::VehicleState>("/vcu/vehicleState", 10);
    vehicleStateDebugPub = this->create_publisher<vcu_msgs::msg::VehicleStateDebug>("/vcu/vehicleState/debug", 10);
    vehicleStateReadablePub
        = this->create_publisher<vcu_msgs::msg::VehicleStateReadable>("/vcu/vehicleState/readable", 10);

    // Subscribers
    inverterSub = this->create_subscription<ix_msgs::msg::Inverter>(
        "/vcu/data/inverter", 1, std::bind(&StateMachineNode::cbInverter, this, _1));
    accumulatorSub = this->create_subscription<ix_msgs::msg::Accumulator>(
        "/vcu/data/accumulator", 1, std::bind(&StateMachineNode::cbAccumulatorInput, this, _1));
    assmSub = this->create_subscription<vcu_msgs::msg::ASSM>(
        "/vcu/data/assm", 1, std::bind(&StateMachineNode::cbASSMInput, this, _1));
    driverInputSub = this->create_subscription<vcu_msgs::msg::DriverInput>(
        "/vcu/data/driverInput", 1, std::bind(&StateMachineNode::cbDISInput, this, _1));
    missionSub = this->create_subscription<ix_msgs::msg::Mission>(
        "/efr/mission", 1, std::bind(&StateMachineNode::cbMissiont, this, _1));
    resInputSub = this->create_subscription<vcu_msgs::msg::RESInput>(
        "/vcu/data/res", 1, std::bind(&StateMachineNode::cbResInput, this, _1));
    accelBrakeSteerSub = this->create_subscription<vcu_msgs::msg::AccelBrakeSteer>(
        "/vcu/data/accelBrakeSteer", 1, std::bind(&StateMachineNode::cbSSBFrontInput, this, _1));
    watchdogSub = this->create_subscription<vcu_msgs::msg::Watchdog>(
        "/vcu/watchdog", 1, std::bind(&StateMachineNode::cbWatchdog, this, _1));
    finishedSub = this->create_subscription<std_msgs::msg::Bool>(
        "/dv/mission/state", 1, std::bind(&StateMachineNode::cbFinished, this, _1));

    // Timers
    timer_state
        = this->create_wall_timer(VEHICLE_STATE_MSG_RATE, std::bind(&StateMachineNode::publishVehicleState, this));
    timer_debug = this->create_wall_timer(DEBUG_MSG_RATE, std::bind(&StateMachineNode::publishDebug, this));
}

/**
 * @brief Publishes the vehicle state message every 5ms
 */
void StateMachineNode::publishVehicleState()
{
    vcu_msgs::msg::VehicleState msg;
    msg.header.stamp = this->now();
    stateVector.updateState((msg.header.stamp.nanosec / NS_TO_MS) + msg.header.stamp.sec * S_TO_MS);

    // Publish the Vehicle State
    VehicleStateMsg stateMsg = stateVector.getVehicleStateMsg();

    msg.state = stateMsg.state;
    msg.sdc_relay_closed = stateMsg.sdc_relay_closed;
    msg.sdc_activated = stateMsg.sdc_activated;
    msg.ts_activated = stateMsg.ts_activated;
    msg.actuators_allowed = stateMsg.actuators_allowed;
    msg.ebs_valve_front = stateMsg.ebs_valve_front_activated;
    msg.ebs_valve_rear = stateMsg.ebs_valve_rear_activated;
    msg.ebs_state = stateMsg.ebs_state;

    msg.watchdog = stateMsg.watchdog;

    vehicleStatePub->publish(msg);
}

/**
 * @brief Publishes the vehicle state debug message every 10ms
 */
void StateMachineNode::publishDebug()
{
    vcu_msgs::msg::VehicleStateDebug msg;
    msg.header.stamp = this->now();

    msg.esus_state = stateVector.getEBS()->getESUSP();
    msg.mission_set = stateVector.getMissionState();
    msg.asms_on = stateVector.getASMSState();
    msg.sdc_ready = stateVector.getEBS()->getSDCReady();
    msg.stfr = stateVector.getVehicleStateMsg().stfr;
    msg.finished = stateVector.getIsFinished();
    msg.errors = errorsIntConverter(stateVector.getErrors());

    vehicleStateDebugPub->publish(msg);
    this->publishDebugReadable();
}

/**
 * @brief Publishes the readable vehicle state message
 * everytime time the vehicle state message is published
 */
void StateMachineNode::publishDebugReadable()
{
    vcu_msgs::msg::VehicleStateReadable msg;

    msg.header.stamp = this->now();

    VehicleStateMsg stateMsg = stateVector.getVehicleStateMsg();

    msg.vehicle_state = vehicleStateEnumToString(stateMsg.state);
    msg.ebs_state = ebsStateEnumToString(stateMsg.ebs_state);
    msg.esus_state = esusStateEnumToString(stateVector.getEBS()->getESUSP());
    msg.stfr = stfrConverter(stateVector.getVehicleStateMsg().stfr);
    msg.sdc_closed = stateVector.getbmcSdcState();
    msg.asms_state = stateVector.getAMSState();
    msg.actuators_allowed = stateMsg.actuators_allowed;
    msg.sdc_state = stateMsg.sdc_activated;
    msg.finished = stateVector.getIsFinished();

    msg.errors = errorsConverter(stateVector.getErrors());

    vehicleStateReadablePub->publish(msg);
}

/**
 * @brief Handling the data from the inverter
 *
 * @param msg ROS message from the inverter
 */
void StateMachineNode::cbInverter(const ix_msgs::msg::Inverter msg)
{
    Inverter invFL;
    Inverter invFR;
    Inverter invRL;
    Inverter invRR;

    if (!((bool)msg.inverter_ready.fl && (bool)msg.inverter_ready.fr && (bool)msg.inverter_ready.rl
            && (bool)msg.inverter_ready.rr))
    {
        stateVector.addError(Error::E_InvNotReady);
    }

    invFL.rpm = msg.actual_wheelspeeds.fl;
    invFR.rpm = msg.actual_wheelspeeds.fr;
    invRL.rpm = msg.actual_wheelspeeds.rl;
    invRR.rpm = msg.actual_wheelspeeds.rr;

    invFL.is_ready = (bool)msg.inverter_ready.fl;
    invFR.is_ready = (bool)msg.inverter_ready.fr;
    invRL.is_ready = (bool)msg.inverter_ready.rl;
    invRR.is_ready = (bool)msg.inverter_ready.rr;

    stateVector.setInverterFLState(invFL);
    stateVector.setInverterFRState(invFR);
    stateVector.setInverterRLState(invRL);
    stateVector.setInverterRRState(invRR);

    // Getting the bigger IC voltage from the inverter
    float voltage = (msg.current_dc_link_voltage_12 > msg.current_dc_link_voltage_34) ? msg.current_dc_link_voltage_12
                                                                                      : msg.current_dc_link_voltage_34;

    stateVector.setIcVoltage(voltage);
}

/**
 * @brief Handling the data from the accumulator
 *
 * @param msg Message from the Accumulator
 */
void StateMachineNode::cbAccumulatorInput(const ix_msgs::msg::Accumulator msg)
{
    AMS amsState;
    switch (msg.bms_state)
    {
    case 0:
        amsState = AMS::AMS_Idle;
        break;
    case 1:
        amsState = AMS::AMS_Precharge;
        break;
    case 2:
        amsState = AMS::AMS_Drive;
        break;
    case 3:
        amsState = AMS::AMS_reserved;
        break;
    case 4:
        amsState = AMS::AMS_Precharge_Failed;
        break;
    case 5:
        amsState = AMS::AMS_Data_Error;
        break;
    case 6:
        amsState = AMS::AMS_Relays_Stuck;
        break;
    default:
        std::cout << "[ERROR] Unknown AMS state" << std::endl;
        return;
    }
    stateVector.setAMSState(amsState);
    stateVector.setbmcSdcState(msg.bms_sdc_state);
}

/**
 * @brief Handling the data from the ASSM
 *
 * @param msg Message from the ASSM
 */
void StateMachineNode::cbASSMInput(const vcu_msgs::msg::ASSM msg)
{
    stateVector.setSDCReady(msg.sdc_ready_state);

    // external TS Button pushed
    if (msg.external_ts_on && timeExTSPushed == 0)
    {
        timeExTSPushed = stateVector.getTimeStamp(); // Time in ms
    }
    else if (!msg.external_ts_on)
    {
        timeExTSPushed = 0;
    }

    // After 50ms a button press is a button press
    if ((stateVector.getTimeStamp() - timeExTSPushed > NORMAL_BUTTON_PRESS_THRESHOLD) && timeExTSPushed != 0)
        stateVector.setSTFR(stateVector.triggerVS_AsOffToVS_AsReady());

    ASMS asmsState = ASMS::ASMS_Off;
    if (msg.asms_on)
    {
        asmsState = ASMS::ASMS_On;
    }

    stateVector.setASMSState(asmsState);
}

/**
 * @brief Handling the data from the DIS
 *
 * @param msg Message from the DIS
 */
void StateMachineNode::cbDISInput(const vcu_msgs::msg::DriverInput msg)
{

    // internal TS Button pushed
    if (msg.button_hv && timeInTSPushed == 0)
    {
        timeInTSPushed = stateVector.getTimeStamp(); // Time in ms
    }
    else if (!msg.button_hv)
    {
        timeInTSPushed = 0;
    }

    // After 50ms a button press is a button press
    if ((stateVector.getTimeStamp() - timeInTSPushed > NORMAL_BUTTON_PRESS_THRESHOLD)
        && (stateVector.getTimeStamp() - timeInTSPushed < EXTENDED_BUTTON_PRESS_THRESHOLD) && timeInTSPushed != 0)
    {
        stateVector.setSTFR(stateVector.triggerVS_IdleToVS_ManualReady());
        // Only if we want to activate TS in error state
        if (stateVector.getVehicleStateMsg().stfr == STFR::FalseInputState)
            stateVector.setSTFR(stateVector.triggerVS_ErrorToVS_ManualReady());
    }

    // State change manual ready to manual drive
    if (msg.button_start && timeR2DPushed == 0)
    {
        timeR2DPushed = stateVector.getTimeStamp();
    }
    else if (!msg.button_start)
    {
        timeR2DPushed = 0;
    }

    if ((stateVector.getTimeStamp() - timeR2DPushed > NORMAL_BUTTON_PRESS_THRESHOLD) && timeR2DPushed != 0)
    {
        stateVector.setSTFR(stateVector.triggerVS_ManualReadyToVS_ManualDrive());
        if (stateVector.getVehicleStateMsg().stfr == STFR::FalseInputState)
            stateVector.setSTFR(stateVector.triggerVS_ErrorToVS_Idle());
    }

    if ((stateVector.getTimeStamp() - timeR2DPushed > EXTENDED_BUTTON_PRESS_THRESHOLD) && timeR2DPushed != 0)
    {
        stateVector.setSTFR(stateVector.triggerVS_ManualDriveToVS_ManualReady());
    }
}

void StateMachineNode::cbMissiont(const ix_msgs::msg::Mission msg)
{
    if (msg.mission_confirmed)
        stateVector.setMissionState(MissionState::MS_Selected);
    else
        stateVector.setMissionState(MissionState::MS_NotSelected);
}

void StateMachineNode::cbResInput(const vcu_msgs::msg::RESInput msg)
{
    if (msg.stop_activated)
    {
        stateVector.setSTFR(stateVector.triggerVS_AsDriveToVS_AsEmergency());
        stateVector.setSTFR(stateVector.triggerVS_AsReadyToVS_AsEmergency());
        stateVector.setSTFR(stateVector.triggerVS_AsFinishedToVS_AsEmergency());
        return;
    }

    if (msg.go_activated)
    {
        stateVector.setSTFR(stateVector.triggerVS_AsReadyToVS_AsDrive());
    }
}

void StateMachineNode::cbSSBFrontInput(const vcu_msgs::msg::AccelBrakeSteer msg)
{
    stateVector.setAllPressures(
        msg.ebs_pressure_front, msg.ebs_pressure_rear, msg.brake_pressure_front, msg.brake_pressure_rear);
}

void StateMachineNode::cbFinished(const std_msgs::msg::Bool msg) { stateVector.setIsFinished(msg.data); }

void StateMachineNode::cbWatchdog(const vcu_msgs::msg::Watchdog msg)
{

    if (msg.pdu_timeout)
        stateVector.addError(Error::E_PDUTimeout);
    if (msg.ssb_front_timeout)
        stateVector.addError(Error::E_SSBFrontTimeout);
    if (msg.ssb_rear_timeout)
        stateVector.addError(Error::E_SSBRearTimeout);
    if (msg.bse_timeout)
        stateVector.addError(Error::E_BSETimeout);
    if (msg.ams_timeout)
        stateVector.addError(Error::E_AMSTimeout);
    if (msg.energymeter_timeout)
        stateVector.addError(Error::E_EnergymeterTimeout);
    if (msg.dis_timeout)
        stateVector.addError(Error::E_DISTimeout);
    if (msg.inverter_timeout)
        stateVector.addError(Error::E_InverterTimeout);
    if (msg.assm_timeout)
        stateVector.addError(Error::E_ASSMTimeout);
    if (msg.dv_timeout)
        stateVector.addError(Error::E_DVTimeout);
    if (msg.apps_implausible)
        stateVector.addError(Error::E_APPSImplausible);
    if (msg.apps1_error)
        stateVector.addError(Error::E_Apps1);
    if (msg.apps2_error)
        stateVector.addError(Error::E_Apps2);
    if (msg.brake_pressure_front_error)
        stateVector.addError(Error::E_BrakePressureFront);
    if (msg.brake_pressure_rear_error)
        stateVector.addError(Error::E_BrakePressureRear);
    if (msg.brake_force_error)
        stateVector.addError(Error::E_BrakeForceSensor);
    if (msg.ebs_front_error)
        stateVector.addError(Error::E_EBSFront);
    if (msg.ebs_rear_error)
        stateVector.addError(Error::E_EBSRear);

    stateVector.setAllowedToActuate(!msg.pedals_implausible);
}

/**
 * @brief Load all the parameters conserning the state machine and ebs state machine
 * @return void
 */
void StateMachineNode::loadParameters()
{
    this->declare_parameter("maxPrechargeTime", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("normalPrechargeTime", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("brakePressureForR2D", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("maxReleasedEBSPressureFront", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("maxReleasedEBSPressureRear", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("minPressurizedEBSPressureFront", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("minPressurizedEBSPressureRear", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("maxEBSPressureFront", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("maxEBSPressureRear", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("minEBSPressureForAsOff", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("maxReleasedBrakePressureFront", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("maxReleasedBrakePressureRear", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("minPressurizedBrakePressureFront", rclcpp::PARAMETER_DOUBLE);
    this->declare_parameter("minPressurizedBrakePressureRear", rclcpp::PARAMETER_DOUBLE);

    Config config;
    // Convert time from seconds to milliseconds
    config.maxPrechargeTime = this->get_parameter("maxPrechargeTime").as_double() * S_TO_MS;
    config.normalPrechargeTime = this->get_parameter("normalPrechargeTime").as_double() * S_TO_MS;
    config.brakePressureForR2D = this->get_parameter("brakePressureForR2D").as_double();
    config.maxReleasedEBSPressureFront = this->get_parameter("maxReleasedEBSPressureFront").as_double();
    config.maxReleasedEBSPressureRear = this->get_parameter("maxReleasedEBSPressureRear").as_double();
    config.minPressurizedEBSPressureFront = this->get_parameter("minPressurizedEBSPressureFront").as_double();
    config.minPressurizedEBSPressureRear = this->get_parameter("minPressurizedEBSPressureRear").as_double();
    config.maxEBSPressureFront = this->get_parameter("maxEBSPressureFront").as_double();
    config.maxEBSPressureRear = this->get_parameter("maxEBSPressureRear").as_double();
    config.minEBSPressureForAsOff = this->get_parameter("minEBSPressureForAsOff").as_double();
    config.maxReleasedBrakePressureFront = this->get_parameter("maxReleasedBrakePressureFront").as_double();
    config.maxReleasedBrakePressureRear = this->get_parameter("maxReleasedBrakePressureRear").as_double();
    config.minPressurizedBrakePressureFront = this->get_parameter("minPressurizedBrakePressureFront").as_double();
    config.minPressurizedBrakePressureRear = this->get_parameter("minPressurizedBrakePressureRear").as_double();

    stateVector.setConfig(config);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateMachineNode>());
    rclcpp::shutdown();

    return 0;
}
