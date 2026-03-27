#include "rclcpp/rclcpp.hpp"
#include <cstdint>
#include <string>

#include <eigen3/Eigen/Geometry>

#include "sensor_msgs/msg/imu.hpp"

#include "ix_msgs/msg/accumulator.hpp"
#include "ix_msgs/msg/dv_observer.hpp"
#include "ix_msgs/msg/energymeter.hpp"
#include "ix_msgs/msg/float32_stamped.hpp"
#include "ix_msgs/msg/gnss_data.hpp"
#include "ix_msgs/msg/inverter.hpp"
#include "ix_msgs/msg/inverter_setpoints.hpp"
#include "ix_msgs/msg/mission.hpp"
#include "ix_msgs/msg/pedals.hpp"
#include "ix_msgs/msg/wheels.hpp"
#include "vcu_shared_lib/wheels.hpp"

#include "vcu_can_msgs_yourcar17/msg/dummy.hpp"

#include "vcu_msgs/msg/accel_brake_steer.hpp"
#include "vcu_msgs/msg/accumulator_output.hpp"
#include "vcu_msgs/msg/assm.hpp"
#include "vcu_msgs/msg/assm_output.hpp"
#include "vcu_msgs/msg/cooling_request.hpp"
#include "vcu_msgs/msg/data_logger.hpp"
#include "vcu_msgs/msg/dis_output.hpp"
#include "vcu_msgs/msg/driver_input.hpp"
#include "vcu_msgs/msg/steering_actuator.hpp"

#include "vcu_msgs/msg/inverter_output.hpp"
#include "vcu_msgs/msg/pdu_output.hpp"
#include "vcu_msgs/msg/res_input.hpp"
#include "vcu_msgs/msg/ssb_rear.hpp"
#include "vcu_msgs/msg/temps.hpp"
#include "vcu_msgs/msg/vehicle_state.hpp"
#include "vcu_msgs/msg/watchdog.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "VCU_Data.hpp"
#include "vcu_shared_lib/params.hpp"

#define EMERRGENCY_SOUND_FREQUENCY 500
#define EMERGENCY_SOUND_ITERATIONS 18
#define R2D_SOUND_TIME 2000
#define DATA_LOGGER_TIME 100

#define GEAR_RATIO 12.23
#define RPM2MS 2.0 * M_PI * 0.206 / (GEAR_RATIO * 60.0)
#define MAX_WHEEL_MOMENT 26.0
#define MAX_BRAKE_PRESSURE 192.0
#define NS_TO_MS 1000000
#define PERCENT_TO_PWM 2.55
#define MAX_PWM 250.0f
#define STEERING_ACTUATOR_FREQUENCY 20

rclcpp::Node::SharedPtr node;
VCU_Data vcuData = VCU_Data();
rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pduPub;
rclcpp::Publisher<vcu_msgs::msg::AccelBrakeSteer>::SharedPtr accelBrakeSteerPublisher;
rclcpp::Publisher<ix_msgs::msg::Energymeter>::SharedPtr energymeterPublisher;
rclcpp::Publisher<vcu_msgs::msg::DriverInput>::SharedPtr driverInputPublisher;
rclcpp::Publisher<vcu_msgs::msg::ASSM>::SharedPtr assmPublisher;
rclcpp::Publisher<ix_msgs::msg::Inverter>::SharedPtr inverterPublisher;
rclcpp::Publisher<vcu_msgs::msg::SSBRear>::SharedPtr ssbRearPublisher;
rclcpp::Publisher<ix_msgs::msg::Accumulator>::SharedPtr accumulatorPublisher;
rclcpp::Publisher<ix_msgs::msg::GnssData>::SharedPtr gnssFrontDataPublisher;
rclcpp::Publisher<ix_msgs::msg::Float32Stamped>::SharedPtr steeringAnglePublisher;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuFrontDataPublisher;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuRearDataPublisher;

rclcpp::Publisher<vcu_msgs::msg::InverterOutput>::SharedPtr inverterOutputPublisher;
rclcpp::Publisher<ix_msgs::msg::Float32Stamped>::SharedPtr steeringOutputPublisher;
rclcpp::Publisher<vcu_msgs::msg::DISOutput>::SharedPtr disOutputPublisher;
rclcpp::Publisher<vcu_msgs::msg::AccumulatorOutput>::SharedPtr accumulatorOutputPublisher;
rclcpp::Publisher<vcu_msgs::msg::ASSMOutput>::SharedPtr assmOutputPublisher;
rclcpp::Publisher<vcu_msgs::msg::PDUOutput>::SharedPtr pduOutputPublisher;
rclcpp::Publisher<vcu_msgs::msg::RESInput>::SharedPtr resPublisher;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dvDriveEnablePublisher;
rclcpp::Publisher<vcu_msgs::msg::DataLogger>::SharedPtr dataLoggerOutputPublisher;
rclcpp::Publisher<vcu_msgs::msg::Temps>::SharedPtr tempsPublisher;
rclcpp::Publisher<ix_msgs::msg::Mission>::SharedPtr dvMissionPublisher;
rclcpp::Publisher<ix_msgs::msg::Wheels>::SharedPtr inv_torque_pub;
rclcpp::Publisher<ix_msgs::msg::Wheels>::SharedPtr inv_wspd_pub;

rclcpp::TimerBase::SharedPtr r2dTimer;
rclcpp::TimerBase::SharedPtr emergencyTimer;
rclcpp::TimerBase::SharedPtr dataLoggerTimer;
rclcpp::TimerBase::SharedPtr pushToTalkTimer;
rclcpp::TimerBase::SharedPtr missionConfirmTimer;

rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr hvEnableClient;
rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr hvDisableClient;
rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr steeringInitClient;
rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resInitClient;

ix_msgs::msg::Wheels last_state_inv;
ix_msgs::msg::GnssData gpsFrontData = ix_msgs::msg::GnssData();
sensor_msgs::msg::Imu imuFrontData = sensor_msgs::msg::Imu();
sensor_msgs::msg::Imu imuRearData = sensor_msgs::msg::Imu();

Eigen::Matrix3d imuFrontCalibrationMatrix;
Eigen::Matrix3d imuRearCalibrationMatrix;

int64_t ebsPWM = 100;
int steeringInitCounter = 0;

void inverterReset()
{
    auto out = vcu_msgs::msg::InverterOutput();
    out.header.stamp = node->now();
    Wheels<bool> reset;
    reset.fl = (last_state_inv.fl == 2);
    reset.fr = (last_state_inv.fr == 2);
    reset.rl = (last_state_inv.rl == 2);
    reset.rr = (last_state_inv.rr == 2);

    out.reset = reset.to_msg();
    inverterOutputPublisher->publish(out);
}

/**
 * @brief Callback to validate the inverter output against the config values specified beforehand
 * !! This function is only executed in HV!!
 * @param msg Inverter Setpoint message containing wheelspeed setpoints as well as torque limits per wheel
 * @return void
 */

void validateInverterOutput(const ix_msgs::msg::InverterSetpoints::SharedPtr msg)
{
    Wheels<float> out_wheelspeed_setpoints(msg->wheelspeed_setpoints);
    Wheels<float> out_torque_lower_bound(msg->lower_torque_bounds);
    Wheels<float> out_torque_upper_bound(msg->upper_torque_bounds);

    out_wheelspeed_setpoints.clamp(0.0f, (float)vcuData.getRPMLimit());
    out_torque_lower_bound.clamp((float)vcuData.getInverterTorqueLimit().lower, 0.0f);
    out_torque_upper_bound.clamp(0.0f, vcuData.getInverterTorqueLimit().upper);

    auto stateMachineData = vcuData.getStateMachinePointer();
    if (!stateMachineData->allowedToActuate())
    {
        out_wheelspeed_setpoints = Wheels<float>(500.0f, 500.0f, 500.0f, 500.0f);
        out_torque_lower_bound = Wheels<float>(0.0f, 0.0f, 0.0f, 0.0f);
        out_torque_upper_bound = Wheels<float>(0.0f, 0.0f, 0.0f, 0.0f);
    }

    if (out_torque_lower_bound.fl < 0.0f)
        out_torque_upper_bound.fl = 0.0f;
    else
        out_torque_lower_bound.fl = 0.0f;

    if (out_torque_lower_bound.fr < 0.0f)
        out_torque_upper_bound.fr = 0.0f;
    else
        out_torque_lower_bound.fr = 0.0f;

    if (out_torque_lower_bound.rl < 0.0f)
        out_torque_upper_bound.rl = 0.0f;
    else
        out_torque_lower_bound.rl = 0.0f;

    if (out_torque_lower_bound.rr < 0.0f)
        out_torque_upper_bound.rr = 0.0f;
    else
        out_torque_lower_bound.rr = 0.0f;

    auto out = vcu_msgs::msg::InverterOutput();
    out.header.stamp = node->now();
    out.wheelspeed_setpoints = out_wheelspeed_setpoints.to_msg();
    out.lower_torque_bounds = out_torque_lower_bound.to_msg();
    out.upper_torque_bounds = out_torque_upper_bound.to_msg();
    out.enabled = vcuData.getInverterEnabledStruct().to_msg();

    // Current control must not be used in the car
    out.current_control = Wheels<bool>(false, false, false, false).to_msg();
    out.asc_allowed = Wheels<bool>(true, true, true, true).to_msg();

    //+++ Todo: Better reset policy +++//
    // Reset Policy
    // VCU sends resets when Inverter in Error State == 2
    Wheels<bool> reset;
    reset.fl = (last_state_inv.fl == 2);
    reset.fr = (last_state_inv.fr == 2);
    reset.rl = (last_state_inv.rl == 2);
    reset.rr = (last_state_inv.rr == 2);

    out.reset = reset.to_msg();

    inverterOutputPublisher->publish(out);

    double speedTarget = 3.6 * 0.25 * RPM2MS
        * (out_wheelspeed_setpoints.fl + out_wheelspeed_setpoints.fr + out_wheelspeed_setpoints.rl
            + out_wheelspeed_setpoints.rr);
    double momentTarget = 0.25
        * (out_torque_lower_bound.fl + out_torque_upper_bound.fl + out_torque_lower_bound.fr + out_torque_upper_bound.fr
            + out_torque_lower_bound.rl + out_torque_upper_bound.rl + out_torque_lower_bound.rr
            + out_torque_upper_bound.rr)
        / MAX_WHEEL_MOMENT;
    vcuData.getDataLoggerDataPointer()->speedTarget = speedTarget;
    vcuData.getDataLoggerDataPointer()->momentTarget = momentTarget * 100.0; // 0..1 -> %
}

void validateSteeringOutput(const ix_msgs::msg::Float32Stamped::SharedPtr msg)
{
    auto out = ix_msgs::msg::Float32Stamped();
    auto stateMachineData = vcuData.getStateMachinePointer();
    if (!stateMachineData->allowedToActuate())
    {
        // dont publish at all, since setting value to 0 could also move it depending on current position
        return;
    }
    out.header.stamp = node->now();
    out.data = msg->data; //+++ Todo: Implement safety check here//
    vcuData.getDataLoggerDataPointer()->steeringAngleTarget = out.data;
    steeringOutputPublisher->publish(out);
}

void sendPDUMessage()
{
    auto stateMachine = vcuData.getStateMachinePointer();
    auto out = vcu_msgs::msg::PDUOutput();
    out.header.stamp = node->now();
    out.power_inverter = true;
    out.power_sdc = stateMachine->sdcActivated;
    out.pwm_ebs_pump = stateMachine->ebsPumpRequest ? ebsPWM : 0;
    out.power_rtds = vcuData.getR2DSound() || vcuData.getEmergencySound();
    out.power_brakelight = vcuData.isBraking();
    out.power_lv_steering_actuator = stateMachine->inAS();

    /* Cooling -> always enabled */
    out.power_fan_swl = true;
    out.power_fan_swr = true;

    out.pwm_cooling_pump = vcuData.getCoolingRequestData().cooling_pump;
    out.pwm_accu_fan = vcuData.getCoolingRequestData().accu_cooling;
    out.pwm_swl_fan = vcuData.getCoolingRequestData().swl_cooling;
    out.pwm_swr_fan = vcuData.getCoolingRequestData().swr_cooling;

    out.power_lv_dcdc_12 = true;
    out.ethernet = true;
    out.power_ogss_schenkler = stateMachine->tsActivated; // Only power Schenkler in HV

    // DV proper version, use this after docker launch over as launcher is fixed
    // out.power_lidar_swl = stateMachine->inAS();
    // out.power_lidar_mh = stateMachine->inAS();
    // out.power_lidar_swr = stateMachine->inAS();

    out.power_lidar_swl = false;
    out.power_lidar_mh = false;
    out.power_lidar_swr = false;
    out.power_ssb = true;

    out.power_assi = true;
    out.power_res = true;
    out.power_driver_communication = true; // Power up Driver Com any time for now

    out.enable_push_to_talk = vcuData.getPushToTalkState();
    out.power_res_racing_mode = stateMachine->inAS();

    // CAN logger always on
    out.power_can_logger = true;

    pduOutputPublisher->publish(out);
}

void sendASSMMessage()
{
    auto stateMachine = vcuData.getStateMachinePointer();
    auto out = vcu_msgs::msg::ASSMOutput();
    out.header.stamp = node->now();
    out.watchdog_assm = stateMachine->watchdogFlag;
    out.ebs_power_stage_front = stateMachine->ebsValveFront;
    out.ebs_power_stage_rear = stateMachine->ebsValveRear;
    out.assm_sdc_close_cmd = stateMachine->sdcRelayClosed;
    out.as_state = vcuData.getStateMachinePointer()->convertVehicleStateTo(CONVERSION_REQUEST::ASSI);
    assmOutputPublisher->publish(out);
}

void callbackSteering(const vcu_msgs::msg::SteeringActuator::SharedPtr msg)
{
    (void)msg;
    vcuData.setSteeringInitState(true);
}

void sendAccumulatorMessage()
{
    auto stateMachine = vcuData.getStateMachinePointer();
    auto out = vcu_msgs::msg::AccumulatorOutput();
    out.header.stamp = node->now();
    out.hv_enabled = stateMachine->tsActivated;
    accumulatorOutputPublisher->publish(out);
}

void sendDISMessage()
{
    auto stateMachine = vcuData.getStateMachinePointer();
    auto torqueLimits = vcuData.getInverterTorqueLimit();
    auto pedalsData = vcuData.getPedalsDataPointer();
    auto accelBrakeSteerPtr = vcuData.getAccelBrakeSteerPointer();
    auto accuData = vcuData.getAccumulatorData();

    auto driverInput = vcuData.getDriverInputData();

    auto out = vcu_msgs::msg::DISOutput();

    // Dis parameters
    out.header.stamp = node->now();
    out.torque_upper_bound = torqueLimits.upper;
    out.torque_lower_bound = torqueLimits.upper;

    out.as_state = stateMachine->inAS();
    out.selected_mission = vcuData.convertVehicleMissionTo(CONVERSION_REQUEST::DISPLAY);

    out.push_to_talk_state = vcuData.getPushToTalkState();
    out.reku_enabled = vcuData.getWatchdogDataPointer()->rekuState; // From watchdog

    // Display navigation
    // TODO: Actual display navigation logic
    out.display_lock_screen = false;
    out.display_navigate_left = false;
    out.display_navigate_right = false;
    if (driverInput.currentScreen > 1)
    {
        out.display_select_screen = driverInput.currentScreen;
    }
    else
    {
        out.display_select_screen = 255; // No selected screen
    }

    // Pedals percent for LED stripes
    out.accel_pedal = pedalsData->accel_pedal;
    out.reku_pedal = pedalsData->reku_pedal;

    out.bms_state = accuData.bmsState;
    out.bms_sdc_state = accuData.bmsSDCState;
    out.accumulator_voltage = accuData.accumulatorVoltage;
    out.bms_soc = accuData.bmsSOC;
    out.cvh = accuData.cellVoltageHigh;
    out.cvl = accuData.cellVoltageLow;
    out.cth = accuData.cellTemperatureHigh;
    out.ctl = accuData.cellTemperatureLow;

    out.ebs_pressure_front = accelBrakeSteerPtr->ebsPressureFront;
    out.ebs_pressure_rear = accelBrakeSteerPtr->ebsPressureRear;
    out.brake_pressure_front = accelBrakeSteerPtr->brakePressureFront;
    out.brake_pressure_rear = accelBrakeSteerPtr->brakePressureRear;

    disOutputPublisher->publish(out);
}

void sendTempsMsg()
{
    auto out = vcu_msgs::msg::Temps();

    // Sidewing temperatures from SSB Rear
    out.temp_rad_swl_in = vcuData.getTempsData()->temp_rad_swl_in;
    out.temp_rad_swl_out = vcuData.getTempsData()->temp_rad_swl_out;
    out.temp_rad_swr_in = vcuData.getTempsData()->temp_rad_swr_in;
    out.temp_rad_swr_out = vcuData.getTempsData()->temp_rad_swr_out;

    // Actual temperature of motors
    out.actual_temp_motor1 = vcuData.getTempsData()->actual_temp_motor1;
    out.actual_temp_motor2 = vcuData.getTempsData()->actual_temp_motor2;
    out.actual_temp_motor3 = vcuData.getTempsData()->actual_temp_motor3;
    out.actual_temp_motor4 = vcuData.getTempsData()->actual_temp_motor4;

    // Actual temperature of power switches
    out.actual_temp_pwr_module1 = vcuData.getTempsData()->actual_temp_pwr_module1;
    out.actual_temp_pwr_module2 = vcuData.getTempsData()->actual_temp_pwr_module2;
    out.actual_temp_pwr_module3 = vcuData.getTempsData()->actual_temp_pwr_module3;
    out.actual_temp_pwr_module4 = vcuData.getTempsData()->actual_temp_pwr_module4;

    // Actual temperature for additonal sensor channel 0 and 5
    // out.actual_temp_add_sensor0 = vcuData.getTempsData()->actual_temp_add_sensor0;
    out.actual_temp_add_sensor5 = vcuData.getTempsData()->actual_temp_add_sensor5;

    // Actual temperature of the carrier board
    out.actual_temp_carrier = vcuData.getTempsData()->actual_temp_carrier;

    tempsPublisher->publish(out);
}

void sendMission()
{
    auto out = ix_msgs::msg::Mission();

    out.selected_mission = vcuData.getVehicleMission();
    out.mission_confirmed = vcuData.isMissionConfirmed();
    out.vehicle_state = vcuData.getStateMachinePointer()->vehicleState;

    dvMissionPublisher->publish(out);
}

void playR2DSound()
{
    vcuData.setR2DSound(true);
    r2dTimer->reset();
}

void triggerR2D()
{
    vcuData.setR2DSound(false);
    r2dTimer->reset();
}

void pushToTalkTimeout() { vcuData.disablePushToTalk(); }

void togglePushToTalk()
{
    if (vcuData.getPushToTalkState())
        // vcuData.disablePushToTalk();
        pushToTalkTimer->reset(); // We currently don't want to turn the push to talk off when we press the button again
                                  // but instead just reset the reset-timer.
    else
    {
        pushToTalkTimer->reset();
        vcuData.enablePushToTalk();
    }
}

void playEmergencySound()
{
    vcuData.setEmergencySound(true);
    vcuData.incrementEmergencySoundCounter();
    emergencyTimer->reset();
}

void triggerEmergencyStop()
{
    int iteration = vcuData.getEmergencySoundCounter();
    // No emergency sound should be played
    if (iteration == 0)
    {
        vcuData.setEmergencySound(false);
        return;
    }

    // End of emergency is reached
    if (iteration == EMERGENCY_SOUND_ITERATIONS)
    {
        vcuData.resetEmergencySoundCounter();
        vcuData.setEmergencySound(false);
        return;
    }

    // Alternate between on and off
    vcuData.incrementEmergencySoundCounter();
    vcuData.setEmergencySound(!vcuData.getEmergencySound());
    emergencyTimer->reset();
}

void callbackIMUFrontDatalogger(const sensor_msgs::msg::Imu& msg)
{
    vcuData.getDataLoggerDataPointer()->accelerationLongitudinal = msg.linear_acceleration.x;
    vcuData.getDataLoggerDataPointer()->accelerationLateral = msg.linear_acceleration.y;
    vcuData.getDataLoggerDataPointer()->yawRate = msg.angular_velocity.z;
}

void callbackDVObserver(const ix_msgs::msg::DvObserver::SharedPtr msg)
{
    vcuData.getDataLoggerDataPointer()->lapcounter = msg->lapcount;
    vcuData.getDataLoggerDataPointer()->cones_count_actual = msg->cone_count_total;
    vcuData.getDataLoggerDataPointer()->cones_count_all = msg->cone_count_current;
}

void triggerDataLogger()
{
    auto out = vcu_msgs::msg::DataLogger();
    out.header.stamp = node->now();

    out.send_asf_message = vcuData.getStateMachinePointer()->inAS();
    out.ebs_front_pressure = vcuData.getAccelBrakeSteerPointer()->ebsPressureFront;
    out.ebs_rear_pressure = vcuData.getAccelBrakeSteerPointer()->ebsPressureRear;
    out.brakepressure_front = vcuData.getAccelBrakeSteerPointer()->brakePressureFront;
    out.brakepressure_rear = vcuData.getAccelBrakeSteerPointer()->brakePressureRear;

    out.speed_actual = vcuData.getDataLoggerDataPointer()->speedCurrent;
    out.speed_target = vcuData.getDataLoggerDataPointer()->speedTarget;
    out.steering_angle_actual = vcuData.getAccelBrakeSteerPointer()->steeringAngle;
    out.steering_angle_target = vcuData.getDataLoggerDataPointer()->steeringAngleTarget;
    out.brake_hydr_actual = 100.0
        * (vcuData.getAccelBrakeSteerPointer()->brakePressureFront
            + vcuData.getAccelBrakeSteerPointer()->brakePressureRear)
        / MAX_BRAKE_PRESSURE; // %
    out.brake_hydr_target = 0.0;
    out.motor_moment_actual = vcuData.getDataLoggerDataPointer()->momentCurrent;
    out.motor_moment_target = vcuData.getDataLoggerDataPointer()->momentTarget;

    out.acceleration_longitudinal = vcuData.getDataLoggerDataPointer()->accelerationLongitudinal;
    out.acceleration_lateral = vcuData.getDataLoggerDataPointer()->accelerationLateral;
    out.yaw_rate = vcuData.getDataLoggerDataPointer()->yawRate;

    uint8_t convertedEbsState = vcuData.getStateMachinePointer()->convertEBSStateTo(CONVERSION_REQUEST::DATA_LOGGER);
    out.as_state = vcuData.getStateMachinePointer()->convertVehicleStateTo(CONVERSION_REQUEST::DATA_LOGGER);
    out.ebs_state = convertedEbsState;
    out.ami_state = vcuData.convertVehicleMissionTo(CONVERSION_REQUEST::DATA_LOGGER);
    out.steering_state
        = vcuData.getStateMachinePointer()->allowedToActuate() && vcuData.getStateMachinePointer()->inAS();

    out.ebs_redundancy_state = convertedEbsState;
    // Switch 2 & 3 for the redundancy check
    switch (convertedEbsState)
    {
    case 2:
        out.ebs_redundancy_state = 3;
        break;
    case 3:
        out.ebs_redundancy_state = 2;
        break;
    default:
        break;
    }

    // Get Data from DV Observer
    out.lapcounter = vcuData.getDataLoggerDataPointer()->lapcounter;
    out.cones_count_actual = vcuData.getDataLoggerDataPointer()->cones_count_actual;
    out.cones_count_all = vcuData.getDataLoggerDataPointer()->cones_count_all;

    dataLoggerOutputPublisher->publish(out);
    dataLoggerTimer->reset();
}

void confirmMission() { vcuData.setIsMissionConfirmed(true); }

void callbackStateMachine(const vcu_msgs::msg::VehicleState::SharedPtr msg)
{
    auto stateMachine = vcuData.getStateMachinePointer();
    auto newVehicleState = VEHICLE_STATE(msg->state);

    /*Sound handling*/
    bool ev_enters_driving
        = (stateMachine->vehicleState == VEHICLE_STATE::ManualReady) && (newVehicleState == VEHICLE_STATE::ManualDrive);
    bool dv_enters_driving
        = (stateMachine->vehicleState == VEHICLE_STATE::AsReady) && (newVehicleState == VEHICLE_STATE::AsDrive);
    if (ev_enters_driving || dv_enters_driving)
        playR2DSound();

    bool dv_enters_emergency
        = (stateMachine->vehicleState == VEHICLE_STATE::AsOff || stateMachine->vehicleState == VEHICLE_STATE::AsReady
              || stateMachine->vehicleState == VEHICLE_STATE::AsDrive
              || stateMachine->vehicleState == VEHICLE_STATE::AsFinished)
        && (newVehicleState == VEHICLE_STATE::AsEmergency);
    if (dv_enters_emergency)
        playEmergencySound();
    /*-----*/

    stateMachine->vehicleState = newVehicleState;
    stateMachine->sdcActivated = msg->sdc_activated;
    bool oldTSState = stateMachine->tsActivated;
    stateMachine->tsActivated = msg->ts_activated;
    stateMachine->sdcRelayClosed = msg->sdc_relay_closed;
    stateMachine->ebsValveFront = msg->ebs_valve_front;
    stateMachine->ebsValveRear = msg->ebs_valve_rear;
    stateMachine->ebsPumpRequest = false;
    stateMachine->actuatorsAllowed = msg->actuators_allowed;
    stateMachine->ebsState = EBS(msg->ebs_state);
    if (stateMachine->first)
    {
        stateMachine->first = false;
    }
    else if (stateMachine->watchdogFlag == msg->watchdog)
    {
        // Emergency behaviour in case the state machine crashes
        stateMachine->sdcActivated = false;
        stateMachine->tsActivated = false;
    }
    stateMachine->watchdogFlag = msg->watchdog;
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    if (stateMachine->tsActivated && (!oldTSState))
    {
        // only send when ts was turned on in this moment
        auto result = hvEnableClient->async_send_request(request);
        (void)result;
    }
    else if (!stateMachine->tsActivated && (oldTSState))
    {
        // only send when ts was turned off in this moment
        auto result = hvDisableClient->async_send_request(request);
        (void)result;
    }

    sendAccumulatorMessage();
    sendASSMMessage();
    sendPDUMessage();
    sendDISMessage();
    sendTempsMsg(); // +++ Todo: can be published less often
    sendMission();

    if (!vcuData.isRESInitialized() && stateMachine->inAS())
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result = resInitClient->async_send_request(request);
    }
}

void callbackPDU(const vcu_can_msgs_yourcar17::msg::Dummy::SharedPtr msg)
{
    // std_msgs::msg::Header out;
    // out.stamp = node->now();
    // pduPub->publish(out);
    (void)msg;
}

void callbackAppsBseSteering(const vcu_can_msgs_yourcar17::msg::Dummy::SharedPtr msg)
{
    (void)msg;

    // auto out = vcu_msgs::msg::AccelBrakeSteer();
    // auto data = vcuData.getAccelBrakeSteerPointer();
    // auto calibrationData = vcuData.getCalibrationData();

    // data->steeringAngle = msg->steering_angle;
    // auto steeringAngleMsg = ix_msgs::msg::Float32Stamped();
    // steeringAngleMsg.header.stamp = node->now();
    // steeringAngleMsg.data = data->steeringAngle;
    // steeringAnglePublisher->publish(steeringAngleMsg);

    // data->apps1 = msg->apps_left;
    // data->apps2 = msg->apps_right;
    // data->brakeforce = msg->brakeforce;

    // out.header.stamp = node->now();
    // out.apps1 = data->apps1;
    // out.apps2 = data->apps2;
    // out.steering_angle = data->steeringAngle;
    // out.brakeforce = data->brakeforce;
    // out.brake_pressure_front = data->brakePressureFront;
    // out.brake_pressure_rear = data->brakePressureRear;
    // out.ebs_pressure_front = data->ebsPressureFront;
    // out.ebs_pressure_rear = data->ebsPressureRear;
    // out.apps1_pressed = calibrationData.apps1.pressed;
    // out.apps1_released = calibrationData.apps1.released;
    // out.apps2_pressed = calibrationData.apps2.pressed;
    // out.apps2_released = calibrationData.apps2.released;

    // accelBrakeSteerPublisher->publish(out);
}

void callbackBpsEbsPressure(const vcu_can_msgs_yourcar17::msg::Dummy::SharedPtr msg)
{
    // auto data = vcuData.getAccelBrakeSteerPointer();
    // data->ebsPressureFront = msg->ebs_pressure1;
    // data->ebsPressureRear = msg->ebs_pressure2;

    // float brakePressureFront = msg->brake_pressure_front;
    // float brakePressureRear = msg->brake_pressure_rear - 0.5;
    // brakePressureRear = std::clamp(brakePressureRear, 0.0f, 150.0f);

    // data->brakePressureFront = brakePressureFront;
    // data->brakePressureRear = brakePressureRear;

    // bool enoughBrakePressure = data->brakePressureFront > 0.5 || data->brakePressureRear > 0.5;
    // auto energymeterData = vcuData.getEnergymeterPointer();

    // bool enoughReku = energymeterData->tsCurrent < -1.0f;
    // vcuData.setBrakingState(enoughBrakePressure || enoughReku);
    (void)msg;
}

void callbackTSCharge(const vcu_can_msgs_yourcar17::msg::Dummy::SharedPtr msg)
{
    // auto data = vcuData.getEnergymeterPointer();
    // data->tsCharge = msg->energymeter_hv_charge;
    (void)msg;
}

void callbackTSCurrent(const vcu_can_msgs_yourcar17::msg::Dummy::SharedPtr msg)
{
    // auto data = vcuData.getEnergymeterPointer();
    // data->tsCurrent = msg->energymeter_hv_current;
    // auto out = ix_msgs::msg::Energymeter();
    // out.header.stamp = node->now();
    // out.ts_charge = data->tsCharge;
    // out.ts_current = data->tsCurrent;
    // out.ts_voltage = data->tsVoltage;
    // out.lv_voltage = data->lvVoltage;
    // out.temperature_energymeter = data->temperatureEnergymeter;
    // energymeterPublisher->publish(out);
    (void)msg;
}

void callbackTSVoltage(const vcu_can_msgs_yourcar17::msg::Dummy::SharedPtr msg)
{
    // auto data = vcuData.getEnergymeterPointer();
    // data->tsVoltage = msg->energymeter_hv_voltage;
    (void)msg;
}

void callbackLVVoltage(const vcu_can_msgs_yourcar17::msg::Dummy::SharedPtr msg)
{
    // auto data = vcuData.getEnergymeterPointer();
    // data->lvVoltage = msg->lvs_voltage;
    (void)msg;
}

void callbackTemperatureEnergymeter(const vcu_can_msgs_yourcar17::msg::Dummy::SharedPtr msg)
{
    // auto data = vcuData.getEnergymeterPointer();
    // data->temperatureEnergymeter = msg->energymeter_temperatur;
    (void)msg;
}

void callbackDriverInput(const vcu_can_msgs_yourcar17::msg::Dummy::ConstSharedPtr& msg)
{
    // auto data = vcuData.getDriverInputData();
    // auto out = vcu_msgs::msg::DriverInput();
    // out.header.stamp = node->now();

    // // Detect state changes
    // if (data.buttonDown == 0 && msg->button_down)
    // {
    //     vcuData.decrementVehicleMission();
    //     missionConfirmTimer->reset();
    //     vcuData.setIsMissionConfirmed(false);
    // }
    // if (data.buttonUp == 0 && msg->button_up)
    // {
    //     vcuData.incrementVehicleMission();
    //     missionConfirmTimer->reset();
    //     vcuData.setIsMissionConfirmed(false);
    // }

    // if (data.buttonPushToTalk == 0 && msg->button_front_top_right)
    //     togglePushToTalk();

    // data.knobTV = msg->tuning_knob1;
    // data.buttonHV = msg->button_hv;
    // data.buttonStart = msg->button_start;
    // data.pedalRight = msg->button_pedal_right;
    // data.pedalLeft = msg->button_pedal_left;
    // data.buttonReku = msg->button_recu;

    // data.buttonPushToTalk = msg->button_front_top_right;

    // data.buttonDown = msg->button_down;
    // data.buttonUp = msg->button_up;
    // data.buttonLeft = msg->button_left;
    // data.buttonRight = msg->button_right;

    // out.pedal_right = data.pedalRight;
    // out.pedal_left = data.pedalLeft;
    // out.button_assm = false;
    // out.button_reku = data.buttonReku;
    // out.button_hv = data.buttonHV;
    // out.button_start = data.buttonStart;
    // out.button_down = data.buttonDown;
    // out.button_up = data.buttonUp;
    // out.button_left = data.buttonLeft;
    // out.button_right = data.buttonRight;

    // vcuData.setDriverInputData(data);
    // driverInputPublisher->publish(out);
    (void)msg;
}

void callbackASSM(const vcu_can_msgs_yourcar17::msg::Dummy::SharedPtr msg)
{
    // auto out = vcu_msgs::msg::ASSM();
    // out.header.stamp = node->now();
    // out.wd_reset = msg->wd_reset;
    // out.wd_ok = msg->wd_ok;
    // out.wd_status = msg->wd_status;
    // out.sdc_ready_state = msg->assm_sdc_ready_state;
    // out.sdc_relay_state = msg->assm_sdc_relay_state;
    // out.sdc_out = msg->assm_sdc_out;
    // out.sdc_in = msg->assm_sdc_in;
    // out.sdc_close_state = msg->assm_sdc_close_state;

    // out.external_ts_on = msg->assm_sdc_out; //??

    // out.assi_blue_on = msg->assi_gpio_state_blue;
    // out.assi_yellow_on = msg->assi_gpio_state_yellow;
    // out.ebs_powerstage_rear_state = msg->assm_ebs_powerstage1_state;
    // out.ebs_powerstage_front_state = msg->assm_ebs_powerstage2_state;
    // out.asms_on = msg->asms_state;
    // out.ebs_front_release_state = msg->ebs_front_release_state;
    // out.ebs_rear_release_state = msg->ebs_rear_release_state;

    // assmPublisher->publish(out);

    // auto stateMachine = vcuData.getStateMachinePointer();

    // if (!stateMachine->inAS() || !out.asms_on)
    // {
    //     vcuData.setSteeringInitState(false);
    // }
    // else
    // {
    //     if (!vcuData.isSteeringInitialized())
    //     {

    //         if (out.asms_on && ((steeringInitCounter % STEERING_ACTUATOR_FREQUENCY) == 0))
    //         {
    //             steeringInitCounter = 0;
    //             auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    //             auto result = steeringInitClient->async_send_request(request);
    //         }
    //         steeringInitCounter++;
    //     }
    // }
    (void)msg;
}

void callbackInverter(const vcu_can_msgs_yourcar17::msg::Dummy::SharedPtr msg)
{
    // auto out = ix_msgs::msg::Inverter();
    // out.header.stamp = node->now();

    // out.wheelspeed_setpoints = Wheels<int>(
    //     msg->setpoint_speed_motor3, msg->setpoint_speed_motor4, msg->setpoint_speed_motor2,
    //     msg->setpoint_speed_motor1)
    //                                .to_msg();
    // out.actual_wheelspeeds = Wheels<int>(
    //     msg->actual_speed_motor3, msg->actual_speed_motor4, msg->actual_speed_motor2, msg->actual_speed_motor1)
    //                              .to_msg();
    // out.actual_torque = Wheels<float>(
    //     msg->actual_torque_motor3, msg->actual_torque_motor4, msg->actual_torque_motor2, msg->actual_torque_motor1)
    //                         .to_msg();
    // out.torque_setpoints = Wheels<float>(msg->setpoint_torque_motor3, msg->setpoint_torque_motor4,
    //     msg->setpoint_torque_motor2, msg->setpoint_torque_motor1)
    //                            .to_msg();

    // auto temps = vcuData.getTempsData();
    // temps->actual_temp_motor1 = msg->actual_temp_motor1;
    // temps->actual_temp_motor2 = msg->actual_temp_motor2;
    // temps->actual_temp_motor3 = msg->actual_temp_motor3;
    // temps->actual_temp_motor4 = msg->actual_temp_motor4;
    // temps->actual_temp_pwr_module1 = msg->actual_temp_pwr_module1;
    // temps->actual_temp_pwr_module2 = msg->actual_temp_pwr_module2;
    // temps->actual_temp_pwr_module3 = msg->actual_temp_pwr_module3;
    // temps->actual_temp_pwr_module4 = msg->actual_temp_pwr_module4;
    // temps->actual_temp_add_sensor0 = msg->actual_temp_add_sensor0;
    // temps->actual_temp_add_sensor5 = msg->actual_temp_add_sensor4;
    // temps->actual_temp_carrier = msg->actual_temp_carrier;

    // // State 0 = Idle, State 1 = Drive, State 2 = Error, State 3 = Config Values Missing
    // auto inverter_connected = ix_msgs::msg::Wheels();
    // char mask = 0x1 << 5;
    // inverter_connected.fl = (msg->state_inv3 & mask) != 0;
    // inverter_connected.fr = (msg->state_inv4 & mask) != 0;
    // inverter_connected.rl = (msg->state_inv2 & mask) != 0;
    // inverter_connected.rr = (msg->state_inv1 & mask) != 0;
    // out.inverter_connected = inverter_connected;

    // auto dclink_ready = ix_msgs::msg::Wheels();
    // mask = 0x1 << 4;
    // dclink_ready.fl = (msg->state_inv3 & mask) != 0;
    // dclink_ready.fr = (msg->state_inv4 & mask) != 0;
    // dclink_ready.rl = (msg->state_inv2 & mask) != 0;
    // dclink_ready.rr = (msg->state_inv1 & mask) != 0;
    // out.dclink_ready = dclink_ready;

    // auto derating = ix_msgs::msg::Wheels();
    // mask = 0x1 << 3;
    // derating.fl = (msg->state_inv3 & mask) != 0;
    // derating.fr = (msg->state_inv4 & mask) != 0;
    // derating.rl = (msg->state_inv2 & mask) != 0;
    // derating.rr = (msg->state_inv1 & mask) != 0;
    // out.derating = derating;

    // auto inverter_ready = ix_msgs::msg::Wheels();
    // mask = 0x1 << 2;
    // inverter_ready.fl = (msg->state_inv3 & mask) != 0;
    // inverter_ready.fr = (msg->state_inv4 & mask) != 0;
    // inverter_ready.rl = (msg->state_inv2 & mask) != 0;
    // inverter_ready.rr = (msg->state_inv1 & mask) != 0;
    // out.inverter_ready = inverter_ready;

    // auto state = ix_msgs::msg::Wheels();
    // mask = 0x3;
    // state.fl = (msg->state_inv3 & mask);
    // state.fr = (msg->state_inv4 & mask);
    // state.rl = (msg->state_inv2 & mask);
    // state.rr = (msg->state_inv1 & mask);
    // out.state = state;

    // out.current_dc_link_voltage_12 = msg->current_dc_link_voltage_12;
    // out.current_dc_link_voltage_34 = msg->current_dc_link_voltage_34;

    // last_state_inv = state;

    // inverterPublisher->publish(out);

    // double speedCurrent = 3.6 * 0.25 * RPM2MS
    //     * (msg->actual_speed_motor1 + msg->actual_speed_motor2 + msg->actual_speed_motor3 +
    //     msg->actual_speed_motor4);
    // double momentCurrent = 0.25
    //     * (msg->actual_torque_motor1 + msg->actual_torque_motor2 + msg->actual_torque_motor3
    //         + msg->actual_torque_motor4)
    //     / MAX_WHEEL_MOMENT;
    // vcuData.getDataLoggerDataPointer()->speedCurrent = speedCurrent;
    // vcuData.getDataLoggerDataPointer()->momentCurrent = momentCurrent * 100.0; // 0..1 -> %

    // auto wspd_msg = ix_msgs::msg::Wheels();
    // auto torque_msg = ix_msgs::msg::Wheels();

    // torque_msg.stamp = node->now();
    // torque_msg.rr = out.actual_torque.rr;
    // torque_msg.rl = out.actual_torque.rl;
    // torque_msg.fl = out.actual_torque.fl;
    // torque_msg.fr = out.actual_torque.fr;
    // inv_torque_pub->publish(torque_msg);

    // wspd_msg.stamp = node->now();
    // wspd_msg.rr = out.actual_wheelspeeds.rr;
    // wspd_msg.rl = out.actual_wheelspeeds.rl;
    // wspd_msg.fl = out.actual_wheelspeeds.fl;
    // wspd_msg.fr = out.actual_wheelspeeds.fr;
    // inv_wspd_pub->publish(wspd_msg);

    // if ((last_state_inv.fr == 2) || (last_state_inv.fl == 2) || (last_state_inv.rr == 2) || (last_state_inv.rl == 2))
    // {
    //     // always try to reset inverter errors if the car is not in ts, else let the inverterOutput validation handle
    //     it if (!vcuData.getStateMachinePointer()->tsActivated)
    //     {
    //         inverterReset();
    //     }
    // }
    (void)msg;
}

void callbackSSBRear(const vcu_can_msgs_yourcar17::msg::Dummy::SharedPtr msg)
{
    (void)msg;
    // auto out = vcu_msgs::msg::SSBRear();
    // out.header.stamp = node->now();
    // ssbRearPublisher->publish(out);
}

void callbackRES(const vcu_can_msgs_yourcar17::msg::Dummy::SharedPtr msg)
{
    // vcuData.markRESasInitialized();
    // auto out = vcu_msgs::msg::RESInput();
    // out.header.stamp = node->now();
    // out.go_activated = msg->res_button_k3;
    // out.stop_activated = !msg->res_e_stop;
    // out.res_radio_quality = msg->res_radio_quality;
    // resPublisher->publish(out);

    // auto stateMachine = vcuData.getStateMachinePointer();

    // auto driveEnableOut = std_msgs::msg::Bool();
    // if (stateMachine->inAS() && stateMachine->allowedToActuate())
    // {
    //     driveEnableOut.data = true;
    // }
    // else
    // {
    //     driveEnableOut.data = false;
    // }
    // dvDriveEnablePublisher->publish(driveEnableOut);
    (void)msg;
}

void callbackBMCOverview(const vcu_can_msgs_yourcar17::msg::Dummy::SharedPtr msg)
{
    // auto out = ix_msgs::msg::Accumulator();
    // out.header.stamp = node->now();

    // auto data = vcuData.getAccumulatorData();

    // data.bmsState = msg->bmc_state;
    // data.accumulatorVoltage = msg->accumulator_voltage;
    // data.bmsSOC = msg->bmc_soc;
    // data.cellVoltageHigh = msg->cell_voltage_high;
    // data.cellVoltageLow = msg->cell_voltage_low;
    // data.cellTemperatureHigh = msg->cell_temperature_high;
    // data.cellTemperatureLow = msg->cell_temperature_low;

    // out.bms_state = data.bmsState;
    // out.bms_sdc_state = data.bmsSDCState;
    // out.accumulator_voltage = data.accumulatorVoltage;
    // out.bms_soc = data.bmsSOC;
    // out.cell_voltage_high = data.cellVoltageHigh;
    // out.cell_voltage_low = data.cellVoltageLow;
    // out.cell_temperature_high = data.cellTemperatureHigh;
    // out.cell_temperature_low = data.cellTemperatureLow;

    // vcuData.setAccumulatorData(data);
    // accumulatorPublisher->publish(out);
    (void)msg;
}

void callbackCoolingRequest(const vcu_msgs::msg::CoolingRequest msg)
{
    CoolingRequest coolingRequest;
    coolingRequest.accu_cooling = std::clamp((msg.accu_cooling * 2.55f), 0.0f, 255.0f); // PWM in out car only up to 255
    coolingRequest.swl_cooling = std::clamp((msg.swl_cooling * 2.55f), 0.0f, 255.0f); // PWM in out car only up to 255
    coolingRequest.swr_cooling = std::clamp((msg.swr_cooling * 2.55f), 0.0f, 255.0f); // PWM in out car only up to 255
    coolingRequest.cooling_pump = std::clamp((msg.cooling_pump * 2.55f), 0.0f, 255.0f); // PWM in out car only up to 255

    vcuData.setCoolingRequestData(coolingRequest);
}

void callbackGpsFrontPosition(const vcu_can_msgs_yourcar17::msg::Dummy msg)
{
    // gpsFrontData.lon = msg.longitude_front;
    // gpsFrontData.lat = msg.latitude_front;

    // // Only publish on new position message -> debug always should get sent directly before position
    // gpsFrontData.header.stamp = node->now();
    // gnssFrontDataPublisher->publish(gpsFrontData);
    (void)msg;
}

void callbackGpsFrontDebug(const vcu_can_msgs_yourcar17::msg::Dummy msg)
{
    // Note: This is technically wrong as vel_x should be forwards velocity as
    // seen from cars refrence frame, but these values are currently only used
    // by the ASR for which this is irrelevant
    // gpsFrontData.vel_x = msg.gps_front_vel_east_fr;
    // gpsFrontData.vel_y = msg.gps_front_vel_north_fr;
    (void)msg;
}

void callbackGpsFrontStatus(const vcu_can_msgs_yourcar17::msg::Dummy msg)
{
    // gpsFrontData.fix = msg.gps_front_fix;
    // gpsFrontData.num_sats = msg.gps_front_num_satellites;
    (void)msg;
}

void callbackImuFrontAcc(const vcu_can_msgs_yourcar17::msg::Dummy msg)
{
    // Eigen::Vector3d vec(msg.acc_x_front, msg.acc_y_front, msg.acc_z_front);
    // Eigen::Vector3d accsCal = imuFrontCalibrationMatrix * vec;

    // imuFrontData.linear_acceleration.x = accsCal.x();
    // imuFrontData.linear_acceleration.y = accsCal.y();
    // imuFrontData.linear_acceleration.z = accsCal.z();
    (void)msg;
}

void callbackImuFrontRot(const vcu_can_msgs_yourcar17::msg::Dummy msg)
{
    // Eigen::Vector3d vec(msg.rot_x_front, msg.rot_y_front, msg.rot_z_front);
    // Eigen::Vector3d rotsCal = imuFrontCalibrationMatrix * vec;

    // imuFrontData.angular_velocity.x = (std::numbers::pi / 180.0) * rotsCal.x();
    // imuFrontData.angular_velocity.y = (std::numbers::pi / 180.0) * rotsCal.y();
    // imuFrontData.angular_velocity.z = (std::numbers::pi / 180.0) * rotsCal.z();

    // imuFrontData.header.stamp = node->now();
    // imuFrontData.header.frame_id = "imu_front";
    // imuFrontData.orientation_covariance[0] = -1;
    // imuFrontDataPublisher->publish(imuFrontData);

    // // Also update datalogger data
    // callbackIMUFrontDatalogger(imuFrontData);
    (void)msg;
}

void callbackImuRearAcc(const vcu_can_msgs_yourcar17::msg::Dummy msg)
{
    // Eigen::Vector3d vec(msg.acc_x_rear, msg.acc_y_rear, msg.acc_z_rear);
    // Eigen::Vector3d accsCal = imuRearCalibrationMatrix * vec;

    // imuRearData.linear_acceleration.x = accsCal.x();
    // imuRearData.linear_acceleration.y = accsCal.y();
    // imuRearData.linear_acceleration.z = accsCal.z();
    (void)msg;
}

void callbackImuRearRot(const vcu_can_msgs_yourcar17::msg::Dummy msg)
{
    // Eigen::Vector3d vec(msg.rot_x_rear, msg.rot_y_rear, msg.rot_z_rear);
    // Eigen::Vector3d rotsCal = imuRearCalibrationMatrix * vec;

    // imuRearData.angular_velocity.x = (std::numbers::pi / 180.0) * rotsCal.x();
    // imuRearData.angular_velocity.y = (std::numbers::pi / 180.0) * rotsCal.y();
    // imuRearData.angular_velocity.z = (std::numbers::pi / 180.0) * rotsCal.z();

    // imuRearData.header.stamp = node->now();
    // imuRearData.header.frame_id = "imu_rear";
    // imuRearData.orientation_covariance[0] = -1;
    // imuRearDataPublisher->publish(imuRearData);
    (void)msg;
}

void callbackTemps(const vcu_can_msgs_yourcar17::msg::Dummy msg)
{
    // auto temps = vcuData.getTempsData();

    // temps->temp_rad_swl_in = msg.temp_rad_swl_in;
    // temps->temp_rad_swl_out = msg.temp_rad_swl_out;
    // temps->temp_rad_swr_in = msg.temp_rad_swr_in;
    // temps->temp_rad_swr_out = msg.temp_rad_swr_out;
    (void)msg;
}

void callbackPedals(const ix_msgs::msg::Pedals msg)
{
    auto data = vcuData.getPedalsDataPointer();

    data->accel_pedal = msg.accel_pedal;
    data->reku_pedal = msg.reku_pedal;
}

void callbackWatchdogReku(const vcu_msgs::msg::Watchdog msg)
{
    vcuData.getWatchdogDataPointer()->rekuState = msg.reku_state;
}

void startUp() { }

/**
 * @brief Load all the parameters conserning the base vcu
 * @return void
 */
void loadParameters()
{
    node->declare_parameter("mission", rclcpp::PARAMETER_INTEGER);
    vcuData.setVehicleMission(VEHICLE_MISSION(node->get_parameter("mission").as_int()));

    CalibrationData* calibrationData = vcuData.getCalibrationDataPointer();
    int64_t* rpmLimit = vcuData.getRPMLimitPointer();
    TorqueLimit* torqueLimit = vcuData.getTorqueLimitPointer();

    ParameterTuples paramTuples = {

        /*--- APPS calibration values ---*/
        { "calibration.apps1.pressed", &calibrationData->apps1.pressed },
        { "calibration.apps1.released", &calibrationData->apps1.released },
        { "calibration.apps2.pressed", &calibrationData->apps2.pressed },
        { "calibration.apps2.released", &calibrationData->apps2.released }, { "inverterRPMLimit", rpmLimit },
        { "torqueLimit/upper", &torqueLimit->upper }, { "torqueLimit/lower", &torqueLimit->lower }
    };
    loadParams(node, paramTuples);

    /*--- Inverter specific parameter values for bounds ---*/
    node->declare_parameter("inverterEnable/fl", rclcpp::PARAMETER_BOOL);
    node->declare_parameter("inverterEnable/fr", rclcpp::PARAMETER_BOOL);
    node->declare_parameter("inverterEnable/rl", rclcpp::PARAMETER_BOOL);
    node->declare_parameter("inverterEnable/rr", rclcpp::PARAMETER_BOOL);

    Wheels<bool> inverterEnabled(node->get_parameter("inverterEnable/fl").as_bool(),
        node->get_parameter("inverterEnable/fr").as_bool(), node->get_parameter("inverterEnable/rl").as_bool(),
        node->get_parameter("inverterEnable/rr").as_bool());
    vcuData.setInverterEnabled(inverterEnabled);

    // Initialize imu rotation matrixes
    imuFrontCalibrationMatrix << -0.99975769, -0.01420748, -0.01681411, -0.012712, 0.99621996, -0.08593139, 0.01797142,
        -0.08569682, -0.99615917;
    imuFrontCalibrationMatrix.transpose();

    imuRearCalibrationMatrix << 0.02058633, 0.96789187, 0.25052253, 0.99973068, -0.01724339, -0.01553172, -0.01071317,
        0.2507748, -0.96798617;
    imuRearCalibrationMatrix.transpose();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("vcu");
    loadParameters();
    auto inverterOutputSub = node->create_subscription<ix_msgs::msg::InverterSetpoints>(
        "/controls/inverter/request", 1, validateInverterOutput);
    auto pduSub = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>("/sensor/can1/pdu_imd", 1, callbackPDU);
    auto appsBseSteeringSub = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>(
        "sensor/can1/apps_bse_steering", 1, callbackAppsBseSteering);
    auto bpsEbsSub = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>(
        "/sensor/can1/bps_ebs_pressure", 1, callbackBpsEbsPressure);
    auto tsCurrentSub = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>(
        "/sensor/can2/energymeter_current", 1, callbackTSCurrent);
    auto tsVoltageSub = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>(
        "/sensor/can2/energymeter_voltage_hv", 1, callbackTSVoltage);
    auto tsChargeSub = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>(
        "/sensor/can2/energymeter_charge", 1, callbackTSCharge);
    auto lvVoltageSub = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>(
        "/sensor/can2/energymeter_voltage_lv", 1, callbackLVVoltage);
    auto energymeterTemperatureSub = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>(
        "/sensor/can2/energymeter_temperature", 1, callbackTemperatureEnergymeter);
    auto driverInputSub = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>(
        "/sensor/can1/driver_input", 1, callbackDriverInput);
    auto assmSub
        = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>("/sensor/can3/assm_state", 1, callbackASSM);
    auto gpsFrontPositionSub = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>(
        "/sensor/can4/gps_position_front", 1, callbackGpsFrontPosition);
    auto gpsFrontDebugSub = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>(
        "/sensor/can4/gps_debug_front", 1, callbackGpsFrontDebug);
    auto gpsFrontStatusSub = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>(
        "/sensor/can4/gps_status_front", 1, callbackGpsFrontStatus);
    auto inverterSub = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>(
        "/sensor/inverter/data_input", 1, callbackInverter);
    auto ssbRearSub
        = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>("/sensor/can1/sdc_rear", 1, callbackSSBRear);
    auto bmsOverviewSub = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>(
        "/sensor/can2/accu_bmc_overview", 1, callbackBMCOverview);
    auto stateMachineSub
        = node->create_subscription<vcu_msgs::msg::VehicleState>("/vcu/vehicleState", 1, callbackStateMachine);
    auto resInputSub
        = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>("/sensor/can3/res", 1, callbackRES);
    auto steeringOutputSub = node->create_subscription<ix_msgs::msg::Float32Stamped>(
        "/controls/steering/request", 1, validateSteeringOutput);
    auto pedalsSub = node->create_subscription<ix_msgs::msg::Pedals>("/controls/pedals", 1, callbackPedals);
    auto coolingRequestSub
        = node->create_subscription<vcu_msgs::msg::CoolingRequest>("/vcu/cooling/request", 1, callbackCoolingRequest);
    auto rediatorTempsSub
        = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>("/sensor/can1/temp_radiator", 1, callbackTemps);
    auto dvObserverSub
        = node->create_subscription<ix_msgs::msg::DvObserver>("/state_controller/observation", 1, callbackDVObserver);
    auto steeringSub
        = node->create_subscription<vcu_msgs::msg::SteeringActuator>("/sensors/steering_actuator", 1, callbackSteering);
    auto watchdogSub = node->create_subscription<vcu_msgs::msg::Watchdog>("/vcu/watchdog", 1, callbackWatchdogReku);
    auto imuFrontAccSub = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>(
        "/sensor/can4/imu_acc_front", 1, callbackImuFrontAcc);
    auto imuRearAccSub = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>(
        "/sensor/can4/imu_acc_rear", 1, callbackImuRearAcc);
    auto imuFrontRotSub = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>(
        "/sensor/can4/imu_rot_front", 1, callbackImuFrontRot);
    auto imuRearRotSub = node->create_subscription<vcu_can_msgs_yourcar17::msg::Dummy>(
        "/sensor/can4/imu_rot_rear", 1, callbackImuRearRot);

    pduPub = node->create_publisher<std_msgs::msg::Header>("/vcu/data/pdu", 1);
    accelBrakeSteerPublisher = node->create_publisher<vcu_msgs::msg::AccelBrakeSteer>("/vcu/data/accelBrakeSteer", 1);
    steeringAnglePublisher = node->create_publisher<ix_msgs::msg::Float32Stamped>("/vcu/data/steeringAngle", 1);

    energymeterPublisher = node->create_publisher<ix_msgs::msg::Energymeter>("/vcu/data/energymeter", 1);
    driverInputPublisher = node->create_publisher<vcu_msgs::msg::DriverInput>("/vcu/data/driverInput", 1);
    assmPublisher = node->create_publisher<vcu_msgs::msg::ASSM>("/vcu/data/assm", 1);
    inverterPublisher = node->create_publisher<ix_msgs::msg::Inverter>("/vcu/data/inverter", 1);
    ssbRearPublisher = node->create_publisher<vcu_msgs::msg::SSBRear>("/vcu/data/ssbRear", 1);
    accumulatorPublisher = node->create_publisher<ix_msgs::msg::Accumulator>("/vcu/data/accumulator", 1);
    gnssFrontDataPublisher = node->create_publisher<ix_msgs::msg::GnssData>("/vcu/data/gnssFront", 1);
    imuFrontDataPublisher = node->create_publisher<sensor_msgs::msg::Imu>("/vcu/data/imuFront", 1);
    imuRearDataPublisher = node->create_publisher<sensor_msgs::msg::Imu>("/vcu/data/imuRear", 1);

    resPublisher = node->create_publisher<vcu_msgs::msg::RESInput>("/vcu/data/res", 1);
    dvDriveEnablePublisher = node->create_publisher<std_msgs::msg::Bool>("/vcu/dv/allowance", 1);
    dvMissionPublisher = node->create_publisher<ix_msgs::msg::Mission>("/efr/mission", 1);
    inv_wspd_pub = node->create_publisher<ix_msgs::msg::Wheels>("/vcu/data/motorSpeeds", 10);
    inv_torque_pub = node->create_publisher<ix_msgs::msg::Wheels>("/vcu/data/motorTorques", 10);

    pduOutputPublisher = node->create_publisher<vcu_msgs::msg::PDUOutput>("/vcu/pdu/output", 1);
    assmOutputPublisher = node->create_publisher<vcu_msgs::msg::ASSMOutput>("/vcu/assm/output", 1);
    accumulatorOutputPublisher = node->create_publisher<vcu_msgs::msg::AccumulatorOutput>("/vcu/accumulator/output", 1);
    steeringOutputPublisher = node->create_publisher<ix_msgs::msg::Float32Stamped>("/vcu/steering/output", 1);
    inverterOutputPublisher = node->create_publisher<vcu_msgs::msg::InverterOutput>("/vcu/inverter/output", 1);
    disOutputPublisher = node->create_publisher<vcu_msgs::msg::DISOutput>("/vcu/dis/output", 1);
    dataLoggerOutputPublisher = node->create_publisher<vcu_msgs::msg::DataLogger>("/vcu/datalogger/output", 1);
    tempsPublisher = node->create_publisher<vcu_msgs::msg::Temps>("/vcu/temps", 1);

    r2dTimer = node->create_wall_timer(std::chrono::milliseconds(R2D_SOUND_TIME), triggerR2D);
    emergencyTimer
        = node->create_wall_timer(std::chrono::milliseconds(EMERRGENCY_SOUND_FREQUENCY), triggerEmergencyStop);
    dataLoggerTimer = node->create_wall_timer(std::chrono::milliseconds(DATA_LOGGER_TIME), triggerDataLogger);
    pushToTalkTimer
        = node->create_wall_timer(std::chrono::milliseconds(10000), pushToTalkTimeout); // ToDo: Parameterise this

    missionConfirmTimer = node->create_wall_timer(std::chrono::milliseconds(2000), confirmMission);

    hvEnableClient = node->create_client<std_srvs::srv::Trigger>("vcu_hv_activate");
    hvDisableClient = node->create_client<std_srvs::srv::Trigger>("vcu_hv_deactivate");
    resInitClient = node->create_client<std_srvs::srv::Trigger>("vcu_res_initialize");
    steeringInitClient = node->create_client<std_srvs::srv::Trigger>("vcu_steering_initialize");

    startUp();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
