/**
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * @file vcu_watchdog.hpp
 *
 * @brief Contains the logic for the watchdog
 *
 * @author Niklas Leukroth <niklas.leukroth@elbflorace.de> 2024-2026
 * @author Laurin Hesslich <laurin.hesslich@elbflorace.de> 2025-2026
 *
 * @copyright MIT
 *
 * @version 1.0.0
 *
 * @note  This package is compliant with the rules from 2025.
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

#include "rclcpp/rclcpp.hpp"
#include <chrono>

#include "Watchdog_Data.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"

#include "ix_msgs/msg/energymeter.hpp"
#include "ix_msgs/msg/int8_stamped.hpp"
#include "ix_msgs/msg/inverter.hpp"
#include "ix_msgs/msg/pedals.hpp"

#include "ix_msgs/msg/accumulator.hpp"
#include "vcu_msgs/msg/accel_brake_steer.hpp"
#include "vcu_msgs/msg/assm.hpp"
#include "vcu_msgs/msg/driver_input.hpp"
#include "vcu_msgs/msg/watchdog.hpp"
#include "vcu_msgs/msg/watchdog_debug.hpp"

#include "std_srvs/srv/trigger.hpp"

#include "vcu_shared_lib/params.hpp"

#include <boost/circular_buffer.hpp>

rclcpp::Node::SharedPtr node;
rclcpp::TimerBase::SharedPtr watchdogPeriodicTimer;
rclcpp::TimerBase::SharedPtr pduTimeoutTimer;
rclcpp::TimerBase::SharedPtr accelBrakeSteerTimeoutTimer;
rclcpp::TimerBase::SharedPtr ssbFrontTimeoutTimer;
rclcpp::TimerBase::SharedPtr energymeterTimeoutTimer;
rclcpp::TimerBase::SharedPtr driverInputTimeoutTimer;
rclcpp::TimerBase::SharedPtr assmTimeoutTimer;
rclcpp::TimerBase::SharedPtr inverterTimeoutTimer;
rclcpp::TimerBase::SharedPtr amsTimeoutTimer;
rclcpp::TimerBase::SharedPtr appsImplausibleTimer;
rclcpp::TimerBase::SharedPtr appsErrorTimer;
rclcpp::TimerBase::SharedPtr brakePressureErrorTimer;
rclcpp::TimerBase::SharedPtr brakeForceErrorTimer;
rclcpp::TimerBase::SharedPtr ebsErrorTimer;
rclcpp::TimerBase::SharedPtr ebsReleaseTimer;
rclcpp::TimerBase::SharedPtr recuPedalMoreThanHalfTimer;
rclcpp::TimerBase::SharedPtr AccelPedalFullTimer;
rclcpp::TimerBase::SharedPtr lidarResetTimer;
rclcpp::Publisher<vcu_msgs::msg::Watchdog>::SharedPtr watchdogPublisher;
rclcpp::Publisher<ix_msgs::msg::Pedals>::SharedPtr pedalsPublisher;
rclcpp::Publisher<vcu_msgs::msg::WatchdogDebug>::SharedPtr watchdogDebugPublisher;

Watchdog_Data watchdogData = Watchdog_Data();
LatestErrors latestErrors = LatestErrors();
BrakeforceData brakeforceData = BrakeforceData();

Times vcu_times = Times();

// ToDo: move to header
bool inError = false;
double AMSDerating;

template <typename T> bool isWithinRange(T lower, T upper, T value) { return (value <= upper) && (value >= lower); }

bool checkBrakeForce(float brakeForce)
{
    float lowerBrakeforceLimit = brakeforceData.released - brakeforceData.calibrationOffset;
    float upperBrakeforceLimit = brakeforceData.pressed + brakeforceData.calibrationOffset;
    bool brakeforceImplausibility = !isWithinRange<float>(lowerBrakeforceLimit, upperBrakeforceLimit, brakeForce);

    auto plausibilityData = watchdogData.getPlausibilityDataPointer();
    float recupedal = (brakeForce - brakeforceData.released) / (brakeforceData.pressed - brakeforceData.released);
    plausibilityData->recuperationPedal = std::clamp(recupedal, 0.0f, 1.0f);

    return brakeforceImplausibility;
}

int checkBrakePressure(float brakePressureFront, float brakePressureRear)
{
    float lowerPressureLimit = -2.0f; // might want to have this as a parameter
    float upperPressureLimit = 120.0f; // might want to have this as a parameter
    bool frontPressureImplausibility
        = !isWithinRange<float>(lowerPressureLimit, upperPressureLimit, brakePressureFront);
    bool rearPressureImplausibility = !isWithinRange<float>(lowerPressureLimit, upperPressureLimit, brakePressureRear);

    if (!frontPressureImplausibility && !rearPressureImplausibility)
        return 0; // both valid
    if (frontPressureImplausibility && rearPressureImplausibility)
        return 3; // both in error
    if (frontPressureImplausibility)
        return 1; // only bps1 in error
    if (rearPressureImplausibility)
        return 2; // only bps2 in error
    return -1; // should not be able to get here
}

int checkAPPS(float apps1, float apps2)
{
    auto configApps = watchdogData.getAPPSCalibration();
    auto correctedApps = watchdogData.getCorrectedAppsPointer();
    bool pressedLargerThanReleased = configApps.apps1.pressed > configApps.apps1.released;

    float upper = 1.0f + (pressedLargerThanReleased ? configApps.apps1.pressed : configApps.apps1.released);
    float lower = (pressedLargerThanReleased ? configApps.apps1.released : configApps.apps1.pressed) - 1.0f;
    bool errorApps1 = !isWithinRange<float>(lower, upper, apps1);

    pressedLargerThanReleased = configApps.apps2.pressed > configApps.apps2.released;
    upper = 1.0f + (pressedLargerThanReleased ? configApps.apps2.pressed : configApps.apps2.released);
    lower = (pressedLargerThanReleased ? configApps.apps2.released : configApps.apps2.pressed) - 1.0f;
    bool errorApps2 = !isWithinRange<float>(lower, upper, apps2);

    if (!errorApps1 && !errorApps2)
    {
        correctedApps->apps1 = apps1;
        correctedApps->apps2 = apps2;
        return 0; // both valid
    }
    if (errorApps1 && errorApps2)
        return 3; // both in error
    if (errorApps1)
    {
        correctedApps->apps2 = apps2;
        return 1; // only apps1 in error
    }
    if (errorApps2)
    {
        correctedApps->apps1 = apps1;
        return 2; // only apps2 in error
    }
    return -1; // should not be able to get here
}

bool checkAppsImplausibility(float apps1, float apps2)
{
    //+++ Needs adjustment in case of apps having errors +++//
    auto configApps = watchdogData.getAPPSCalibration();
    auto plausibilityData = watchdogData.getPlausibilityDataPointer();

    double relative_pos_apps1
        = (apps1 - configApps.apps1.released) / (configApps.apps1.pressed - configApps.apps1.released);
    double relative_pos_apps2
        = (apps2 - configApps.apps2.released) / (configApps.apps2.pressed - configApps.apps2.released);

    double unnormalizedAccelPedal = std::min(relative_pos_apps1, relative_pos_apps2);
    plausibilityData->acceleratorPedal = std::clamp(unnormalizedAccelPedal, 0.0, 1.0);

    double apps_implausibility = std::abs(relative_pos_apps1 - relative_pos_apps2);
    plausibilityData->appsImplausiblityValue = apps_implausibility * 100.0;

    return (apps_implausibility > APPS_IMPLAUSIBILITY_LIMIT);
}

int checkEBSImplausibility(float ebsFront, float ebsRear)
{
    float lowerPressureLimit = -2.0f; // might want to have this as a parameter
    float upperPressureLimit = 100.0f; // might want to have this as a parameter

    bool frontPressureImplausibility = !isWithinRange<float>(lowerPressureLimit, upperPressureLimit, ebsFront);
    bool rearPressureImplausibility = !isWithinRange<float>(lowerPressureLimit, upperPressureLimit, ebsRear);

    if (!frontPressureImplausibility && !rearPressureImplausibility)
        return 0; // both valid
    if (frontPressureImplausibility && rearPressureImplausibility)
        return 3; // both in error
    if (frontPressureImplausibility)
        return 1; // only ebsps1 in error
    if (rearPressureImplausibility)
        return 2; // only ebsps2 in error
    return -1; // should not be able to get here
}

void sendWatchdogDebugMessage()
{
    auto msg = vcu_msgs::msg::WatchdogDebug();

    msg.header.stamp = node->now();

    msg.apps_implausibility = watchdogData.getPlausibilityDataPointer()->appsImplausiblityValue;

    msg.recu_state_button = watchdogData.getRecuActivated();
    msg.recu_state_screenshot = watchdogData.getRecuStates()->recuStatePedalPlausability;

    watchdogDebugPublisher->publish(msg);
}

void sendWatchdogMessage()
{
    auto msg = vcu_msgs::msg::Watchdog();
    auto timeouts = watchdogData.getTimeoutsPointer();
    auto errors = watchdogData.getErrorsPointer();

    msg.header.stamp = node->now();

    // Timeouts
    msg.ssb_front_timeout = timeouts->ssbFrontTimeout;
    msg.ams_timeout = timeouts->amsTimeout;
    msg.inverter_timeout = timeouts->inverterTimeout;
    msg.dv_timeout = timeouts->dvTimeout;

    // Errors
    msg.apps_implausible = errors->appsImplausible;
    msg.apps1_error = errors->apps1Error;
    msg.apps2_error = errors->apps2Error;
    msg.brake_pressure_front_error = errors->brakePressureFrontError;
    msg.brake_pressure_rear_error = errors->brakePressureRearError;
    msg.ebs_front_error = errors->ebsPressureFrontError;
    msg.ebs_rear_error = errors->ebsPressureRearError;
    msg.brake_force_error = errors->brakeForceError;
    msg.pedals_implausible = errors->pedalsImplausible;

    // recuState
    msg.reku_state = watchdogData.getRecuStates()->recuState;

    watchdogPublisher->publish(msg);
    sendWatchdogDebugMessage();
}

void callbackPDU(const std_msgs::msg::Header::SharedPtr msg)
{
    (void)msg; // ignore warnings
    auto timeouts = watchdogData.getTimeoutsPointer();
    timeouts->pduTimeout = false;
    pduTimeoutTimer->reset();
}

void triggerPDU()
{
    auto timeouts = watchdogData.getTimeoutsPointer();
    timeouts->pduTimeout = true;
    sendWatchdogMessage();
}

void checkAccelRecuPedal()
{
    auto pData = watchdogData.getPlausibilityDataPointer();
    auto errors = watchdogData.getErrorsPointer();

    if (pData->maxBrakePressure > MAX_BRAKE_PRESSURE_WHILE_ACCELERATING)
        pData->acceleratorPedal = 0;


    watchdogData.getRecuStates()->recuStatePedalPlausability
        = (errors->recuPedalMoreThanHalf || !errors->accelPedalFull);
    watchdogData.getRecuStates()->recuStateButton = watchdogData.getRecuActivated();
    watchdogData.getRecuStates()->recuState = true;

    auto& recuStates = *watchdogData.getRecuStates();
    recuStates.recuStateButton = watchdogData.getRecuActivated();
    recuStates.recuStatePedalPlausability = (errors->recuPedalMoreThanHalf || !errors->accelPedalFull);

    recuStates.recuState = recuStates.recuStatePedalPlausability && recuStates.recuStateButton;

    if (!recuStates.recuState)
        pData->recuperationPedal = 0;
}

void callbackAccelBrakeSteer(const vcu_msgs::msg::AccelBrakeSteer::SharedPtr msg)
{
    // +++ Timeout +++
    auto timeouts = watchdogData.getTimeoutsPointer();
    auto errors = watchdogData.getErrorsPointer();
    auto plausibilityData = watchdogData.getPlausibilityDataPointer();
    auto correctedApps = watchdogData.getCorrectedAppsPointer();

    timeouts->ssbFrontTimeout = false;
    accelBrakeSteerTimeoutTimer->reset();
    ssbFrontTimeoutTimer->reset();

    APPS newAppsConfig;
    newAppsConfig.apps1.pressed = msg->apps1_pressed;
    newAppsConfig.apps1.released = msg->apps1_released;
    newAppsConfig.apps2.pressed = msg->apps2_pressed;
    newAppsConfig.apps2.released = msg->apps2_released;
    watchdogData.setAppsCalibration(newAppsConfig);

    // +++ Plausibility +++
    // BrakePressure
    plausibilityData->maxBrakePressure
        = (msg->brake_pressure_front > msg->brake_pressure_rear) ? msg->brake_pressure_front : msg->brake_pressure_rear;
    latestErrors.brakePressureErrors = checkBrakePressure(msg->brake_pressure_front, msg->brake_pressure_rear);
    if (!latestErrors.brakePressureErrors)
    {
        errors->brakePressureFrontError = false;
        errors->brakePressureRearError = false;
        brakePressureErrorTimer->reset();
    }

    // BrakeForce
    bool brakeForce = checkBrakeForce(msg->brakeforce);
    if (!brakeForce)
    {
        errors->brakeForceError = false;
        brakeForceErrorTimer->reset();
    }

    // APPS
    latestErrors.appsErrors = checkAPPS(msg->apps1, msg->apps2);
    if (!latestErrors.appsErrors)
    {
        errors->apps1Error = false;
        errors->apps2Error = false;
        appsErrorTimer->reset();
    }

    bool appsImplausibility = checkAppsImplausibility(correctedApps->apps1, correctedApps->apps2);
    if (!appsImplausibility)
    {
        errors->appsImplausible = false;
        appsImplausibleTimer->reset();
    }

    // EBS
    latestErrors.ebsErrors = checkEBSImplausibility(msg->ebs_pressure_front, msg->ebs_pressure_rear);
    if (!latestErrors.ebsErrors)
    {
        errors->ebsPressureFrontError = false;
        errors->ebsPressureRearError = false;
        ebsErrorTimer->reset();
    }

    checkAccelRecuPedal();

    // publish message for controls
    auto out_msg = ix_msgs::msg::Pedals();

    out_msg.header.stamp = node->now();
    out_msg.accel_pedal = (plausibilityData->acceleratorPedal);
    out_msg.reku_pedal = (plausibilityData->recuperationPedal);

    pedalsPublisher->publish(out_msg);
}

void triggerAppsBrakeSteer()
{
    auto timeouts = watchdogData.getTimeoutsPointer();
    timeouts->bseTimeout = true;
    timeouts->ssbFrontTimeout = true;
    sendWatchdogMessage();
}

void triggerSSBFrontTimeout()
{
    auto timeouts = watchdogData.getTimeoutsPointer();
    timeouts->bseTimeout = true;
    timeouts->ssbFrontTimeout = true;
    sendWatchdogMessage();
}

void callbackEnergymeter(const ix_msgs::msg::Energymeter::SharedPtr msg)
{
    auto plausibilityData = watchdogData.getPlausibilityDataPointer();
    auto timeouts = watchdogData.getTimeoutsPointer();

    plausibilityData->actualPower = msg->ts_current * msg->ts_voltage / WATT_TO_KW;
    timeouts->energymeterTimeout = false;
    energymeterTimeoutTimer->reset();
}

void triggerEnergymeter()
{
    auto timeouts = watchdogData.getTimeoutsPointer();
    timeouts->energymeterTimeout = true;
    sendWatchdogMessage();
}

void callbackLidarWatchdog(const ix_msgs::msg::Int8Stamped::SharedPtr msg)
{
    auto timeouts = watchdogData.getTimeoutsPointer();
    if (msg->data == 1)
    {
        lidarResetTimer->reset();
        timeouts->dvTimeout = true;
        sendWatchdogMessage();
    }
    else
    {
        timeouts->dvTimeout = false;
    }
}

void resetLidarTimeout()
{
    auto timeouts = watchdogData.getTimeoutsPointer();
    timeouts->dvTimeout = false;
}

void triggerEBSRelease()
{
    auto errors = watchdogData.getErrorsPointer();
    errors->ebsReleaseRearError = true;
}

void callbackDriverInput(const vcu_msgs::msg::DriverInput::SharedPtr msg)
{
    // Dis timeout handling
    auto timeouts = watchdogData.getTimeoutsPointer();
    timeouts->disTimeout = false;
    driverInputTimeoutTimer->reset();

    // recu button handling
    bool buttonRecu = msg->button_reku;

    static bool lastRecuButtonState = false; // to detect edges in recu button
    static bool waitingForRelease = false; // to track a complete press-release

    if (buttonRecu && !lastRecuButtonState) // detect button press
    {
        waitingForRelease = true;
    }

    if (buttonRecu && lastRecuButtonState
        && waitingForRelease) // detect brakeforce release to not toggle recu under braking
    {
        watchdogData.setRecuActivated(!watchdogData.getRecuActivated()); // toggle recu state here
        waitingForRelease = false;
    }
    lastRecuButtonState = buttonRecu;
}

void triggerDriverInput()
{
    auto timeouts = watchdogData.getTimeoutsPointer();
    timeouts->disTimeout = true;
    sendWatchdogMessage();
}

void callbackASSM(const vcu_msgs::msg::ASSM::SharedPtr msg)
{
    (void)msg;
    auto timeouts = watchdogData.getTimeoutsPointer();

    timeouts->assmTimeout = false;
    assmTimeoutTimer->reset();
}

void triggerASSM()
{
    auto timeouts = watchdogData.getTimeoutsPointer();
    timeouts->assmTimeout = true;
    sendWatchdogMessage();
}

void callbackInverter(const ix_msgs::msg::Inverter::SharedPtr msg)
{
    (void)msg;
    auto timeouts = watchdogData.getTimeoutsPointer();
    timeouts->inverterTimeout = false;
    inverterTimeoutTimer->reset();
    timeouts->assmTimeout = false;
}

void triggerInverter()
{
    auto timeouts = watchdogData.getTimeoutsPointer();
    timeouts->inverterTimeout = true;
    sendWatchdogMessage();
}

void callbackAccumulator(const ix_msgs::msg::Accumulator::SharedPtr msg)
{
    (void)msg;
    auto timeouts = watchdogData.getTimeoutsPointer();
    timeouts->amsTimeout = false;
    amsTimeoutTimer->reset();
}

void triggerAMS()
{
    auto timeouts = watchdogData.getTimeoutsPointer();
    timeouts->amsTimeout = true;
    sendWatchdogMessage();
}

void triggerAPPSImplausible()
{
    auto errors = watchdogData.getErrorsPointer();
    errors->appsImplausible = true;
    sendWatchdogMessage();
}

void triggerEBS()
{
    auto errors = watchdogData.getErrorsPointer();
    switch (latestErrors.ebsErrors)
    {
    case 0: // should not get here
        errors->ebsPressureFrontError = false;
        errors->ebsPressureRearError = false;
        ebsErrorTimer->reset();
        return;
    case 1: // ebs front in error
        errors->ebsPressureFrontError = true;
        errors->ebsPressureRearError = false;
        break;
    case 2: // ebs rear in error
        errors->ebsPressureFrontError = false;
        errors->ebsPressureRearError = true;
        break;
    case 3: // both in error
        errors->ebsPressureFrontError = true;
        errors->ebsPressureRearError = true;
        break;
    default:
        std::cout << "Should not get here, ebs watchdog check" << std::endl;
        break;
    }
    sendWatchdogMessage();
}

void triggerAPPSError()
{
    auto errors = watchdogData.getErrorsPointer();
    switch (latestErrors.appsErrors)
    {
    case 0: // should not get here
        errors->apps1Error = false;
        errors->apps2Error = false;
        appsErrorTimer->reset();
        return;
    case 1: // apps front in error
        errors->apps1Error = true;
        errors->apps2Error = false;
        break;
    case 2: // apps rear in error
        errors->apps1Error = false;
        errors->apps2Error = true;
        break;
    case 3: // both in error
        errors->apps1Error = true;
        errors->apps2Error = true;
        break;
    default:
        std::cout << "Should not get here, apps watchdog check" << std::endl;
        break;
    }
    sendWatchdogMessage();
}

void triggerBrakePressure()
{
    auto errors = watchdogData.getErrorsPointer();
    switch (latestErrors.brakePressureErrors)
    {
    case 0: // should not get here
        errors->brakePressureFrontError = false;
        errors->brakePressureRearError = false;
        brakePressureErrorTimer->reset();
        return;
    case 1: // brake pressure front in error
        errors->brakePressureFrontError = true;
        errors->brakePressureRearError = false;
        break;
    case 2: // brake pressure rear in error
        errors->brakePressureFrontError = false;
        errors->brakePressureRearError = true;
        break;
    case 3: // both in error
        errors->brakePressureFrontError = true;
        errors->brakePressureRearError = true;
        break;
    default:
        std::cout << "Should not get here, brakePressure watchdog check" << std::endl;
        break;
    }
    sendWatchdogMessage();
}

void triggerBrakeForce()
{
    auto errors = watchdogData.getErrorsPointer();
    errors->brakeForceError = true;
    sendWatchdogMessage();
}

void triggerAccelPedalError()
{
    auto errors = watchdogData.getErrorsPointer();
    errors->accelPedalFull = true;
}

void triggerRecuPedalError()
{
    auto errors = watchdogData.getErrorsPointer();
    errors->recuPedalMoreThanHalf = true;
}

/**
 * @brief Load all the parameters conserning the base vcu
 * @return void
 */
ParameterTuples getParameterTuples()
{
    ParameterTuples params = {
        /*--- Timeout vcu_times ---*/
        { "timeouts.periodic", &vcu_times.periodicTime },
        { "timeouts.pduTimeout", &vcu_times.pduTime },
        { "timeouts.ssbFrontTimeout", &vcu_times.ssbFrontTime },
        { "timeouts.bseTimeout", &vcu_times.bseTime },
        { "timeouts.amsTimeout", &vcu_times.amsTime },
        { "timeouts.energymeterTimeout", &vcu_times.energymeterTime },
        { "timeouts.disTimeout", &vcu_times.disTime },
        { "timeouts.inverterTimeout", &vcu_times.inverterTime },
        { "timeouts.dvTimeout", &vcu_times.dvTime },
        { "timeouts.assmTimeout", &vcu_times.assmTime },

        /*--- Error vcu_times ---*/
        { "errors.appsImplausibilityTime", &vcu_times.appsImplausibleTime },
        { "errors.appsError", &vcu_times.appsErrorTime },
        { "errors.brakePressureError", &vcu_times.brakePressureTime },
        { "errors.brakeForceError", &vcu_times.brakeForceTime },
        { "errors.ebsPressureError", &vcu_times.ebsPressureTime },

        /*---Brakeforce calibration---*/
        { "brakeforceReleased", &brakeforceData.released },
        { "brakeforcePressed", &brakeforceData.pressed },
        { "brakeforceCaliOffset", &brakeforceData.calibrationOffset },
    };

    /*---Recu parameters*/
    node->declare_parameter("recuState", rclcpp::PARAMETER_BOOL);
    watchdogData.setRecuActivated(node->get_parameter("recuState").as_bool());
    return params;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("vcu_watchdog");
    auto timeouts = watchdogData.getTimeoutsPointer();
    timeouts->dvTimeout = false;
    ParameterTuples paramTuples = getParameterTuples();
    loadParams(node, paramTuples);

    auto pduSub = node->create_subscription<std_msgs::msg::Header>("/vcu/data/pdu", 1, callbackPDU);
    auto accelBrakeSteerSub = node->create_subscription<vcu_msgs::msg::AccelBrakeSteer>(
        "/vcu/data/accelBrakeSteer", 1, callbackAccelBrakeSteer);
    auto energymeterSub
        = node->create_subscription<ix_msgs::msg::Energymeter>("/vcu/data/energymeter", 1, callbackEnergymeter);
    auto driverInputSub
        = node->create_subscription<vcu_msgs::msg::DriverInput>("/vcu/data/driverInput", 1, callbackDriverInput);
    auto assmSub = node->create_subscription<vcu_msgs::msg::ASSM>("/vcu/data/assm", 1, callbackASSM);
    auto inverterSub = node->create_subscription<ix_msgs::msg::Inverter>("/vcu/data/inverter", 1, callbackInverter);
    auto accumulatorSub
        = node->create_subscription<ix_msgs::msg::Accumulator>("/vcu/data/accumulator", 1, callbackAccumulator);
    auto lidarWatchdogSub
        = node->create_subscription<ix_msgs::msg::Int8Stamped>("/watchdog/emergency", 1, callbackLidarWatchdog);

    watchdogPublisher = node->create_publisher<vcu_msgs::msg::Watchdog>("/vcu/watchdog", 1);
    pedalsPublisher = node->create_publisher<ix_msgs::msg::Pedals>("/controls/pedals", 1);
    watchdogDebugPublisher = node->create_publisher<vcu_msgs::msg::WatchdogDebug>("/vcu/watchdog/debug", 1);

    watchdogPeriodicTimer
        = node->create_wall_timer(std::chrono::milliseconds(vcu_times.periodicTime), sendWatchdogMessage);
    pduTimeoutTimer = node->create_wall_timer(std::chrono::milliseconds(vcu_times.pduTime), triggerPDU);
    accelBrakeSteerTimeoutTimer
        = node->create_wall_timer(std::chrono::milliseconds(vcu_times.ssbFrontTime), triggerAppsBrakeSteer);
    energymeterTimeoutTimer
        = node->create_wall_timer(std::chrono::milliseconds(vcu_times.energymeterTime), triggerEnergymeter);
    driverInputTimeoutTimer = node->create_wall_timer(std::chrono::milliseconds(vcu_times.disTime), triggerDriverInput);
    assmTimeoutTimer = node->create_wall_timer(std::chrono::milliseconds(vcu_times.assmTime), triggerASSM);
    inverterTimeoutTimer = node->create_wall_timer(std::chrono::milliseconds(vcu_times.inverterTime), triggerInverter);
    amsTimeoutTimer = node->create_wall_timer(std::chrono::milliseconds(vcu_times.amsTime), triggerAMS);

    appsImplausibleTimer
        = node->create_wall_timer(std::chrono::milliseconds(vcu_times.appsImplausibleTime), triggerAPPSImplausible);
    appsErrorTimer = node->create_wall_timer(std::chrono::milliseconds(vcu_times.appsErrorTime), triggerAPPSError);
    brakePressureErrorTimer
        = node->create_wall_timer(std::chrono::milliseconds(vcu_times.brakePressureTime), triggerBrakePressure);
    brakeForceErrorTimer
        = node->create_wall_timer(std::chrono::milliseconds(vcu_times.brakeForceTime), triggerBrakeForce);
    ssbFrontTimeoutTimer
        = node->create_wall_timer(std::chrono::milliseconds(vcu_times.ssbFrontTime), triggerSSBFrontTimeout);

    ebsErrorTimer = node->create_wall_timer(std::chrono::milliseconds(vcu_times.ebsPressureTime), triggerEBS);
    recuPedalMoreThanHalfTimer = node->create_wall_timer(std::chrono::milliseconds(150), triggerRecuPedalError);
    AccelPedalFullTimer = node->create_wall_timer(std::chrono::milliseconds(150), triggerAccelPedalError);
    lidarResetTimer = node->create_wall_timer(std::chrono::milliseconds(1000), resetLidarTimeout);
    ebsReleaseTimer = node->create_wall_timer(std::chrono::milliseconds(100), triggerEBSRelease);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
