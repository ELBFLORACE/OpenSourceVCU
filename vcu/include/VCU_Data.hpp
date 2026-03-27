#include "ix_msgs/msg/wheels.hpp"
#include <cstdint>

#include "vcu_shared_lib/enums.hpp"
#include "vcu_shared_lib/wheels.hpp"

typedef enum CONVERSION_REQUEST : char
{
    ASSI = 0,
    DATA_LOGGER,
    DISPLAY
} CONVERSION_REQUEST;

struct TorqueLimit
{
    int64_t upper;
    int64_t lower;
};

struct SystemParameters
{
    bool recuperationState;
    bool asrState;
    bool bsrState;
    bool tvState;
};

struct ControlParameters
{
    float accelSlip;
    float brakeSlip;
    int tvFactor;
};

struct StateMachine
{
public:
    VEHICLE_STATE vehicleState;
    bool sdcActivated;
    bool sdcRelayClosed;
    bool tsActivated;
    bool actuatorsAllowed;
    bool ebsValveFront;
    bool ebsValveRear;
    bool ebsPumpRequest;
    EBS ebsState;
    bool watchdogFlag;
    bool first = true;

    bool inAS()
    {
        if ((this->vehicleState == VEHICLE_STATE::Idle) || (this->vehicleState > VEHICLE_STATE::AsEmergency))
            return false;
        return true;
    }

    bool allowedToChangeMission()
    {
        if ((this->vehicleState < VEHICLE_STATE::AsReady) || (this->vehicleState == VEHICLE_STATE::Error))
            return true;
        return false;
    }

    bool allowedToActuate()
    {
        if (this->vehicleState == VEHICLE_STATE::AsDrive || this->vehicleState == VEHICLE_STATE::ManualDrive)
            return actuatorsAllowed;
        return false;
    }

    uint8_t convertVehicleStateTo(CONVERSION_REQUEST request)
    {
        switch (this->vehicleState)
        {
        case VEHICLE_STATE::AsOff:
        {
            if (request == CONVERSION_REQUEST::ASSI)
                return 0;
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 1;
            break;
        }
        case VEHICLE_STATE::AsReady:
        {
            if (request == CONVERSION_REQUEST::ASSI)
                return 1;
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 2;
            break;
        }
        case VEHICLE_STATE::AsDrive:
        {
            if (request == CONVERSION_REQUEST::ASSI)
                return 2;
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 3;
            break;
        }
        case VEHICLE_STATE::AsEmergency:
        {
            if (request == CONVERSION_REQUEST::ASSI)
                return 3;
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 4;
            break;
        }
        case VEHICLE_STATE::AsFinished:
        {
            if (request == CONVERSION_REQUEST::ASSI)
                return 4;
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 5;
            break;
        }
        default:
        {
            if (request == CONVERSION_REQUEST::ASSI)
                return 0;
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 1;
        }
        }
        return 0;
    }

    uint8_t convertEBSStateTo(CONVERSION_REQUEST request)
    {
        switch (this->ebsState)
        {
        case EBS::Activated:
        {
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 3;
            break;
        }
        case EBS::Armed:
        {
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 2;
            break;
        }
        case EBS::Armed_Parking:
        {
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 2;
            break;
        }
        case EBS::Disabled:
        {
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 1;
            break;
        }
        case EBS::Disabled_Manual:
        {
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 1;
            break;
        }
        case EBS::Error:
        {
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 3;
            break;
        }
        case EBS::Startup_Error:
        {
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 3;
            break;
        }
        case EBS::Startup:
        {
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 3;
            break;
        }
        default:
        {
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 1;
        }
        }
        return 1;
    }
};

struct PedalsData
{
    float accel_pedal = 0.0f;
    float reku_pedal = 0.0f;
};

struct AccelBrakeSteerData
{
    float apps1;
    float apps2;
    float brakeforce;
    float brakePressureFront;
    float brakePressureRear;
    float ebsPressureFront;
    float ebsPressureRear;
    float steeringAngle;
};

struct EnergymeterData
{
    float tsCharge;
    float tsCurrent;
    float tsVoltage;
    float lvVoltage;
    float temperatureEnergymeter;
};

struct DriverInput
{
    float knobTV;
    float knobB3;
    bool pedalRight;
    bool pedalLeft;
    bool buttonBack;
    bool buttonPg;
    bool buttonASSM;
    bool buttonReku;
    bool buttonHV;
    bool buttonStart;
    bool buttonMiddle;
    bool buttonCoolingPump;
    bool buttonPushToTalk;

    bool buttonDown;
    bool buttonUp;
    bool buttonRight;
    bool buttonLeft;
    uint8_t currentScreen;
};

struct AccumulatorData
{
    int bmsState;
    int bmsSDCState;
    float accumulatorVoltage;
    float bmsSOC;
    float cellVoltageHigh;
    float cellVoltageLow;
    float cellTemperatureHigh;
    float cellTemperatureLow;
};

struct CoolingRequest
{
    // pwm 0-255
    float accu_cooling;
    float swl_cooling;
    float swr_cooling;
    float cooling_pump;
};

struct Temps
{
    /** Radiator Temps */
    float temp_rad_swl_in;
    float temp_rad_swl_out;
    float temp_rad_swr_in;
    float temp_rad_swr_out;

    /** Inverter Temps */
    // Motor Temps
    float actual_temp_motor1;
    float actual_temp_motor2;
    float actual_temp_motor3;
    float actual_temp_motor4;

    // Actual temperature of power switches
    float actual_temp_pwr_module1;
    float actual_temp_pwr_module2;
    float actual_temp_pwr_module3;
    float actual_temp_pwr_module4;

    // Actual temperature for additonal sensor channel 0 and 5
    float actual_temp_add_sensor0;
    float actual_temp_add_sensor5;

    // Actual temperature of the carrier board
    float actual_temp_carrier;
};

struct CalibrationData
{
    struct AppsValues
    {
        double pressed;
        double released;
    };
    AppsValues apps1;
    AppsValues apps2;
};

struct DataLoggerData
{
    float steeringAngleTarget; // °
    float speedCurrent; // km/h
    float speedTarget; // km/h
    float momentCurrent; // %
    float momentTarget; // %
    float accelerationLongitudinal; // m/s²
    float accelerationLateral; // m/s²
    float yawRate; // °/s
    uint32_t lapcounter;
    uint32_t cones_count_actual;
    uint32_t cones_count_all;
};

struct WatchdogData
{
    bool rekuState;
};

class VCU_Data
{
public:
    // --- Getter ---
    VEHICLE_MISSION getVehicleMission()
    {
        if (this->stateMachine.inAS())
            return VEHICLE_MISSION(this->mission_counter);
        if (this->mission_counter == 0)
            return VEHICLE_MISSION::SCRUTI;
        if (this->mission_counter == 5)
            return VEHICLE_MISSION::ENDURANCE;
        if (this->mission_counter == 1)
            return VEHICLE_MISSION::BRAKETEST;
        return VEHICLE_MISSION(this->mission_counter);
    }
    bool isMissionConfirmed() { return this->missionConfirmed; }
    Wheels<bool> getInverterEnabledStruct() { return this->inverterEnabled; }
    int64_t getRPMLimit() { return this->rpmLimit; }
    TorqueLimit getInverterTorqueLimit() { return this->torqueLimit; }
    SystemParameters getSystemParameters() { return this->systemParameters; }
    ControlParameters getControlParameters() { return this->controlParameters; }
    StateMachine* getStateMachinePointer() { return &this->stateMachine; }
    DataLoggerData* getDataLoggerDataPointer() { return &this->dataLoggerData; }
    AccelBrakeSteerData* getAccelBrakeSteerPointer() { return &this->accelBrakeSteerData; }
    PedalsData* getPedalsDataPointer() { return &this->pedalsData; }
    WatchdogData* getWatchdogDataPointer() { return &this->watchdogData; }

    EnergymeterData* getEnergymeterPointer() { return &this->energymeterData; }
    DriverInput getDriverInputData() { return this->driverInputData; }
    AccumulatorData getAccumulatorData() { return this->accumulatorData; }
    CoolingRequest getCoolingRequestData() { return this->coolingRequestData; }
    Temps* getTempsData() { return &this->temps; }
    CalibrationData getCalibrationData() { return this->calibrationData; }
    CalibrationData* getCalibrationDataPointer() { return &this->calibrationData; }
    bool getR2DSound() { return this->playR2DSound; }
    bool getEmergencySound() { return this->playEmergencySound; }
    bool isBraking() { return this->brakingState; }
    bool getPushToTalkState() { return this->pushToTalkState; }

    int getEmergencySoundCounter() { return this->emergencyIterationCounter; }
    uint32_t getLastValidBrakePressureFrontTime() { return this->lastValidBrakePressureFrontTime; }
    uint32_t getLastValidBrakePressureRearTime() { return this->lastValidBrakePressureRearTime; }
    int convertVehicleMissionTo(CONVERSION_REQUEST request)
    {
        switch (this->getVehicleMission())
        {
        case VEHICLE_MISSION::INSPECTION:
        {
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 5;
            if (request == CONVERSION_REQUEST::DISPLAY)
                return 0;
            break;
        }
        case VEHICLE_MISSION::EBS_TEST:
        {
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 4;
            if (request == CONVERSION_REQUEST::DISPLAY)
                return 1;
            break;
        }
        case VEHICLE_MISSION::SKIDPAD:
        {
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 2;
            if (request == CONVERSION_REQUEST::DISPLAY)
                return 2;
            break;
        }
        case VEHICLE_MISSION::ACCELERATION:
        {
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 1;
            if (request == CONVERSION_REQUEST::DISPLAY)
                return 3;
            break;
        }
        case VEHICLE_MISSION::AUTOCROSS:
        {
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 6;
            if (request == CONVERSION_REQUEST::DISPLAY)
                return 4;
            break;
        }
        case VEHICLE_MISSION::TRACKDRIVE:
        {
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 3;
            if (request == CONVERSION_REQUEST::DISPLAY)
                return 5;
            break;
        }
        case VEHICLE_MISSION::SCRUTI:
        {
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 0;
            if (request == CONVERSION_REQUEST::DISPLAY)
                return 6;
            break;
        }
        case VEHICLE_MISSION::BRAKETEST:
        {
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 0;
            if (request == CONVERSION_REQUEST::DISPLAY)
                return 7;
            break;
        }
        case VEHICLE_MISSION::ENDURANCE:
        {
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 0;
            if (request == CONVERSION_REQUEST::DISPLAY)
                return 8;
            break;
        }
        case VEHICLE_MISSION::TEAMDRIVING:
        {
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 0;
            if (request == CONVERSION_REQUEST::DISPLAY)
                return 9;
            break;
        }
        default:
            if (request == CONVERSION_REQUEST::DATA_LOGGER)
                return 0;
            if (request == CONVERSION_REQUEST::DISPLAY)
                return 0;
        }
        return 0;
    }

    // --- Setter ---
    void setVehicleMission(VEHICLE_MISSION newMission)
    {
        switch (newMission)
        {
        case VEHICLE_MISSION::SCRUTI:
            newMission = VEHICLE_MISSION::INSPECTION;
            break;
        case VEHICLE_MISSION::BRAKETEST:
            newMission = VEHICLE_MISSION::EBS_TEST;
            break;
        case VEHICLE_MISSION::ENDURANCE:
            newMission = VEHICLE_MISSION::TRACKDRIVE;
            break;

        default:
            if (newMission > 9) // highest standard enum in missions
            {
                newMission = VEHICLE_MISSION::INSPECTION;
            }
            break;
        }
        this->mission_counter = newMission;
    }
    void setInverterEnabled(Wheels<bool> inverterEnabled) { this->inverterEnabled = inverterEnabled; }
    int64_t* getRPMLimitPointer() { return &this->rpmLimit; }
    TorqueLimit* getTorqueLimitPointer() { return &this->torqueLimit; }
    void setSystemParameters(SystemParameters systemParameters) { this->systemParameters = systemParameters; }
    void setControlParameters(ControlParameters controlParameters) { this->controlParameters = controlParameters; }
    void setAccelBrakeSteerData(AccelBrakeSteerData data) { this->accelBrakeSteerData = data; }
    void setEnergymeterData(EnergymeterData data) { this->energymeterData = data; }
    void setDriverInputData(DriverInput data) { this->driverInputData = data; }
    void setAccumulatorData(AccumulatorData data) { this->accumulatorData = data; }
    void setCoolingRequestData(CoolingRequest data) { this->coolingRequestData = data; }
    void setTemps(Temps data) { this->temps = data; }

    void incrementVehicleMission()
    {
        if (!this->stateMachine.allowedToChangeMission())
            return;
        this->mission_counter += 1;
        if (this->mission_counter == 6)
            this->mission_counter = 0;
    }
    void decrementVehicleMission()
    {
        if (!this->stateMachine.allowedToChangeMission())
            return;
        this->mission_counter -= 1;
        if (this->mission_counter == -1)
            this->mission_counter = 5;
    }

    void enablePushToTalk() { this->pushToTalkState = true; }
    void disablePushToTalk() { this->pushToTalkState = false; }
    void setR2DSound(bool r2d) { this->playR2DSound = r2d; }
    void setEmergencySound(bool emergency) { this->playEmergencySound = emergency; }
    void setBrakingState(bool braking) { this->brakingState = braking; }
    void incrementEmergencySoundCounter() { this->emergencyIterationCounter += 1; }
    void resetEmergencySoundCounter() { this->emergencyIterationCounter = 0; }
    void resetLastValidBrakePressureFrontTime(uint32_t time) { this->lastValidBrakePressureFrontTime = time; }
    void resetLastValidBrakePressureRearTime(uint32_t time) { this->lastValidBrakePressureRearTime = time; }
    void markRESasInitialized() { this->resInitialized = true; }
    bool isRESInitialized() { return this->resInitialized; }
    bool isSteeringInitialized() { return this->steeringInitialized; }
    void setSteeringInitState(bool state) { this->steeringInitialized = state; }
    void setIsMissionConfirmed(bool state) { this->missionConfirmed = state; }

private:
    VEHICLE_MISSION vehicleMission;
    Wheels<bool> inverterEnabled;
    int64_t rpmLimit;
    TorqueLimit torqueLimit;
    SystemParameters systemParameters;
    ControlParameters controlParameters;
    StateMachine stateMachine;
    AccelBrakeSteerData accelBrakeSteerData;
    PedalsData pedalsData;
    EnergymeterData energymeterData;
    DriverInput driverInputData;
    AccumulatorData accumulatorData;
    CoolingRequest coolingRequestData;
    CalibrationData calibrationData;
    DataLoggerData dataLoggerData;
    Temps temps;
    WatchdogData watchdogData;

    int mission_counter = 0;
    bool missionConfirmed = false;
    int lastMissionTimestamp = 0;
    bool playR2DSound = false;
    bool playEmergencySound = false;
    bool brakingState = false;
    int emergencyIterationCounter = 0;
    uint32_t lastValidBrakePressureFrontTime;
    uint32_t lastValidBrakePressureRearTime;
    bool rekuState = true;
    bool resInitialized = false;
    bool steeringInitialized = false;
    bool pushToTalkState = false;
};

void startUp();
