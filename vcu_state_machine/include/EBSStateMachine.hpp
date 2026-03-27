/**
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * @file EBSStateMachine.hpp
 *
 * @brief Defining the ebs state machine and structs for internal use
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

#ifndef EBS_STATE_MACHINE_H
#define EBS_STATE_MACHINE_H

#include <cstdint>
#include <iostream>
#include <vcu_shared_lib/enums.hpp>

/* ------------------------- Constants ------------------------------------- */

/** Maximum time to wait (in ms) after the watchdog is turned off */
#define WAIT_FOR_WATCHDOG 1000
/**
 * Maximum time to wait (in ms) until the brakes should be released
 * -- not defined in the rules
 */
#define WAIT_FOR_BOTH_BRAKE_RELEASE 2000
/**
 * Maximum time to wait (in ms) until the front brakes should have
 * enough brake pressure
 * -- FSG Rules 2025: T 15.4.1
 */
#define WAIT_FOR_FRONT_BRAKE_ACTIVE 200
/**
 * Maximum time to wait (in ms) until the rear brakes should have
 * enough brake pressure
 * -- FSG Rules 2025: T 15.4.1
 */
#define WAIT_FOR_REAR_BRAKE_ACTIVE 200
/** Timeout for EBS start up sequence */
#define TIMEOUT_ESUS 60000

struct EBSConfig
{
    double minPressurizedEBSPressureFront;
    double minPressurizedEBSPressureRear;
    double maxEBSPressureFront;
    double maxEBSPressureRear;
    double minPressurizedBrakePressureFront;
    double minPressurizedBrakePressureRear;
    double maxReleasedBrakePressureFront;
    double maxReleasedBrakePressureRear;
    double maxReleasedEBSPressureFront;
    double maxReleasedEBSPressureRear;
};

/**
 * Changes while going through the start up sequence and
 * indicates the location of its termination
 */
typedef enum EBSStartupSequencePosition
{
    WaitForASSM,
    CheckWatchdog,
    CheckAllPressure,
    CheckFrontBrakeRelease,
    CheckFrontBrakeActivation,
    CheckRearBrakeRelease,
    CheckRearBrakeActivation,
    ESUS_Ok,
    ESUS_Error
} ESUS;

class EBSStateMachine
{
private:
    EBS ebsState;
    ESUS esusp;
    EBSConfig ebsConfig;

    double brakePressureFront;
    double brakePressureRear;
    double ebsPressureFront;
    double ebsPressureRear;

    bool ebsValveFront;
    bool ebsValveRear;

    bool esusError;

    bool deactivateWatchdog;
    bool sdcReady;

    uint32_t timeStamp;
    uint32_t lastTimeStamp;
    uint32_t timeSinceLastTransition;

public:
    EBSStateMachine()
    {
        ebsState = EBS::Disabled_Manual;
        esusp = ESUS::WaitForASSM;

        brakePressureFront = 0.0;
        brakePressureRear = 0.0;
        ebsPressureFront = 0.0;
        ebsPressureRear = 0.0;

        ebsValveFront = false;
        ebsValveRear = false;

        esusError = false;

        deactivateWatchdog = false;
        sdcReady = false;

        timeStamp = 0;
        lastTimeStamp = 0;
        timeSinceLastTransition = 0;
    }

    void setEBSState(EBS state) { this->ebsState = state; }

    EBS getEBSState() { return ebsState; }

    ESUS getESUSP() { return esusp; }

    void setConfig(EBSConfig ebsConfig);

    void updateTime(uint32_t actualTime)
    {
        lastTimeStamp = timeStamp;
        timeStamp = actualTime;
        timeSinceLastTransition += timeStamp - lastTimeStamp;
    }

    void setSDCReady(bool v);

    bool isWatchdogDeactivated() { return deactivateWatchdog; }

    void setESUSError() { this->esusError = true; }

    void setESUSP(ESUS esusp) { this->esusp = esusp; }

    void setAllPressures(double ebsFront, double ebsRear, double brakeFront, double brakeRear);

    void setBrakePressureFront(double v) { this->brakePressureFront = v; }

    double getBrakePressureFront() { return brakePressureFront; }

    void setBrakePressureRear(double v) { this->brakePressureRear = v; }

    double getBrakePressureRear() { return brakePressureRear; }

    void setEBSPressureFront(double v) { this->ebsPressureFront = v; }

    double getEBSPressureFront() { return ebsPressureFront; }

    void setEBSPressureRear(double v) { this->ebsPressureRear = v; }

    double getEBSPressureRear() { return ebsPressureRear; }

    bool getEBSValveFront() { return ebsValveFront; }

    bool getEBSValveRear() { return ebsValveRear; }

    void startESUS(int id);

    void resetEBS();

    void check();

    void triggerEmergency();

    void changeToAsDrive();

    void changeToAsOff();

    void changeToAsFinished();

    bool isEbsDeactivated();

    bool isEBSPressureTooLow();

    bool isSDCReady() { return sdcReady; }

    void setDeactivateWatchdog(bool v) { deactivateWatchdog = v; }

    void resetTimeSinceLastTransition() { timeSinceLastTransition = 0; }

    uint32_t getTimeSinceLastTransition() { return timeSinceLastTransition; }

    EBSConfig getEbsConfig() { return ebsConfig; }

    void setEbsValveFront(bool v) { ebsValveFront = v; }

    void setEbsValveRear(bool v) { ebsValveRear = v; }

    bool getSDCReady() { return sdcReady; }

    bool checkEBSTanksEmpty();
};

void esusCycle(EBSStateMachine* esm, int id);

#endif
