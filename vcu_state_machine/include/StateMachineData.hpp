/**
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * @file StateMachineData.hpp
 *
 * @brief Defining the state machine and structs for internal use
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

#ifndef STATEMACHINEDATA_H
#define STATEMACHINEDATA_H

#include "EBSStateMachine.hpp"
#include "vcu_shared_lib/enums.hpp"
#include <list>
using namespace std;

/* ------------------------- Constants ------------------------------------- */

/** Defined by FSG rules 2025: T 14.9.5 */
#define WAIT_AFTER_EMERGENCY 10000
/** Inverter wheelspeeds are fluctuating at stand still*/
#define WHEELSPEEDS_AT_STANDSTILL 10
/** Inverter IC Voltages are fluctuating */
#define MAX_DISCHARGED_IC_VOLTAGE 5.0
/** Discharge voltage demanded by rules */
#define RULES_DISCHARGE 60.0
/** Time the ts_activation is delayed for the AIRs */
#define TS_DEACTIVATION_DELAY 25 // ms

/**
 *  Type that is set when a state transition is performed or fails
 */
typedef enum StateTransitionFailureReason
{
    NoFailure,
    SDCNotClosed,
    NoMissionSelected,
    ASMSNotRight,
    EBSStartupFailed,
    ErrorsExist,
    AMSNotDrive,
    InverterNotReady,
    VehicleNotInStandstill,
    BrakeTestFailed,
    NotWaitedForASDrive,
    AlreadyInState,
    FalseInputState,
    EBSIsPressureized,
    EBSError
} STFR;

/**
 * Internal config data structure for the state machine
 */
struct Config
{
    double maxPrechargeTime;
    double normalPrechargeTime;
    double brakePressureForR2D;
    double maxReleasedEBSPressureFront;
    double maxReleasedEBSPressureRear;
    double minPressurizedEBSPressureFront;
    double minPressurizedEBSPressureRear;
    double maxEBSPressureFront;
    double maxEBSPressureRear;
    double minEBSPressureForAsOff;
    double maxReleasedBrakePressureFront;
    double maxReleasedBrakePressureRear;
    double minPressurizedBrakePressureFront;
    double minPressurizedBrakePressureRear;
};

typedef enum ASMS
{
    ASMS_Off,
    ASMS_On
} ASMS;

typedef enum MissionState
{
    MS_NotSelected,
    MS_Selected
} MissionState;

typedef struct Inverter
{
    bool is_ready;
    int rpm;
} Inverter;

typedef enum AMS
{
    AMS_Idle,
    AMS_Precharge,
    AMS_Drive,
    AMS_reserved,
    AMS_Precharge_Failed,
    AMS_Data_Error,
    AMS_Relays_Stuck
} AMS;

typedef enum Error
{
    E_None,
    E_PDUTimeout,
    E_APPSImplausible,
    E_Apps1,
    E_Apps2,
    E_BrakePressureFront,
    E_BrakePressureRear,
    E_BrakeForceSensor,
    E_SSBFrontTimeout,
    E_SSBRearTimeout,
    E_BSETimeout,
    E_AMSTimeout,
    E_EnergymeterTimeout,
    E_DISTimeout,
    E_InverterTimeout,
    E_ASSMTimeout,
    E_DVTimeout,
    E_InvNotReady,
    E_EBSFront,
    E_EBSRear
} Error;

struct VehicleStateMsg
{
    VEHICLE_STATE state;
    bool sdc_activated;
    bool sdc_relay_closed;
    bool ts_activated;
    bool actuators_allowed;
    bool ebs_valve_front_activated;
    bool ebs_valve_rear_activated;
    bool watchdog;

    EBS ebs_state;
    std::list<Error> errors;
    STFR stfr;
};

class VehicleStateVector
{
private:
    VehicleStateMsg stateMsg;

    uint32_t timeStamp;
    uint32_t lastTimeStamp;
    uint32_t timseSinceLastStateChange;
    uint32_t tsDeactivationDelayTime;

    ASMS asmsState;
    MissionState missionState;
    Inverter inverter1;
    Inverter inverter2;
    Inverter inverter3;
    Inverter inverter4;
    AMS amsState;

    bool bmcSdcState;
    bool isFinished;
    float icVoltage;

    EBSStateMachine ebs;

    Config config;

public:
    VehicleStateVector();

    uint32_t getTimeStamp() { return timeStamp; }

    EBSStateMachine* getEBS() { return &ebs; }

    void setConfig(Config config)
    {
        this->config = config;
        EBSConfig ebsConfig;
        ebsConfig.minPressurizedEBSPressureFront = config.minPressurizedEBSPressureFront;
        ebsConfig.minPressurizedEBSPressureRear = config.minPressurizedEBSPressureRear;
        ebsConfig.minPressurizedBrakePressureFront = config.minPressurizedBrakePressureFront;
        ebsConfig.minPressurizedBrakePressureRear = config.minPressurizedBrakePressureRear;
        ebsConfig.maxEBSPressureFront = config.maxEBSPressureFront;
        ebsConfig.maxEBSPressureRear = config.maxEBSPressureRear;
        ebsConfig.maxReleasedBrakePressureFront = config.maxReleasedBrakePressureFront;
        ebsConfig.maxReleasedBrakePressureRear = config.maxReleasedBrakePressureRear;
        ebsConfig.maxReleasedEBSPressureFront = config.maxReleasedEBSPressureFront;
        ebsConfig.maxReleasedEBSPressureRear = config.maxReleasedEBSPressureRear;
        ebs.setConfig(ebsConfig);
    }

    Config* getConfig() { return &config; }

    void setAllPressures(double ebsFront, double ebsRear, double brakeFront, double brakeRear)
    {
        ebs.setAllPressures(ebsFront, ebsRear, brakeFront, brakeRear);
    }

    void setIsFinished(bool b) { this->isFinished = b; }

    bool getIsFinished() { return isFinished; }

    void setBrakePressureFront(double v) { ebs.setBrakePressureFront(v); }

    void setBrakePressureRear(double v) { ebs.setBrakePressureRear(v); }

    void setEBSPressureFront(double v) { ebs.setEBSPressureFront(v); }

    void setEBSPressureRear(double v) { ebs.setEBSPressureRear(v); }

    void setEBSState(EBS state) { ebs.setEBSState(state); }

    void setSDCReady(bool sdcReady) { this->ebs.setSDCReady(sdcReady); }

    void setbmcSdcState(bool bmcSdcState) { this->bmcSdcState = bmcSdcState; }

    bool getbmcSdcState() { return bmcSdcState; }

    void setASMSState(ASMS asmsState);

    ASMS getASMSState() { return asmsState; }

    void setIcVoltage(float icVoltage) { this->icVoltage = icVoltage; }

    float getIcVoltage() { return icVoltage; }

    bool compareASMSState(ASMS asmsState) { return this->asmsState == asmsState; }

    void setMissionState(MissionState missionState) { this->missionState = missionState; }

    MissionState getMissionState() { return missionState; }

    bool compareMissionState(MissionState missionState) { return this->missionState == missionState; }

    void setInverterFLState(Inverter inverter1State) { this->inverter1 = inverter1State; }

    Inverter getInverter1State() { return inverter1; }

    void setInverterFRState(Inverter inverter2State) { this->inverter2 = inverter2State; }

    Inverter getInverter2State() { return inverter2; }

    void setInverterRLState(Inverter inverter3State) { this->inverter3 = inverter3State; }

    Inverter getInverter3State() { return inverter3; }

    void setInverterRRState(Inverter inverter4State) { this->inverter4 = inverter4State; }

    Inverter getInverter4State() { return inverter4; }

    bool isVehicleAtStandstill()
    {
        return inverter1.rpm <= WHEELSPEEDS_AT_STANDSTILL && inverter2.rpm <= WHEELSPEEDS_AT_STANDSTILL
            && inverter3.rpm <= WHEELSPEEDS_AT_STANDSTILL && inverter4.rpm <= WHEELSPEEDS_AT_STANDSTILL;
    }

    void setAMSState(AMS amsState) { this->amsState = amsState; }

    AMS getAMSState() { return amsState; }

    bool compareAMSState(AMS amsState) { return this->amsState == amsState; }

    EBS getEBSState() { return stateMsg.ebs_state; }

    bool compareEBSState(EBS ebsstate) { return stateMsg.ebs_state == ebsstate; }

    void addError(Error error);

    void setErrors(std::list<Error> errors) { this->stateMsg.errors = errors; }

    void resetErrors();

    void removeError(Error error);

    void setSTFR(STFR stfr) { this->stateMsg.stfr = stfr; }

    STFR getSTFR() { return this->stateMsg.stfr; }

    std::list<Error> getErrors() { return stateMsg.errors; }

    VEHICLE_STATE getVehicleState() { return stateMsg.state; }

    VehicleStateMsg getVehicleStateMsg() { return stateMsg; }

    void setAllowedToActuate(bool allowed) { stateMsg.actuators_allowed = allowed && vehicleInDriveState(); }

    bool vehicleInDriveState()
    {
        return getVehicleState() == VEHICLE_STATE::AsDrive || getVehicleState() == VEHICLE_STATE::ManualDrive;
    }

    void updateState(uint32_t milliseconds);

    void checkEBS();

    void updateEBS();

    void changeToAsOff();

    void changeToAsReady();

    void changeToAsDrive();

    void changeToAsFinished();

    void changeToIdle();

    void changeToAsEmergency();

    void changeToManualReady();

    void changeToManualDrive();

    void changeToError();

    STFR triggerVS_IdleToVS_AsOff();

    STFR triggerVS_AsOffToVS_AsReady();

    STFR triggerVS_AsReadyToVS_AsDrive();

    STFR triggerVS_AsReadyToVS_AsEmergency();

    STFR triggerVS_AsDriveToVS_AsEmergency();

    STFR triggerVS_AsFinishedToVS_AsEmergency();

    STFR triggerVS_IdleToVS_ManualReady();

    STFR triggerVS_ManualReadyToVS_ManualDrive();

    STFR triggerVS_ManualDriveToVS_ManualReady();

    STFR triggerVS_ManualDriveToVS_Idle();

    STFR triggerVS_ErrorToVS_Idle();

    STFR triggerVS_ErrorToVS_ManualReady();

    STFR triggerVS_ManualReadyToVS_Idle();

    bool accuFailureTest();

    bool containsError(Error error);

    bool checkErrors();

    bool checkForCrucialErrors();

    bool checkForCrucialDVErrors();

    void checkDischarge();

    void checkTsDeactivation();
};

#endif
