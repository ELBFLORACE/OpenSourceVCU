/**
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * @file stateMachineData.cpp
 *
 * @brief Implementing the state machine
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

#include <StateMachineData.hpp>
#include <chrono>
#include <iostream>
#include <thread>

/**
 * Initialization of the vehicle state vector
 */
VehicleStateVector::VehicleStateVector()
{
    ebs = EBSStateMachine();

    VEHICLE_STATE state = VEHICLE_STATE::Idle;
    stateMsg.state = state;
    stateMsg.sdc_activated = true;
    stateMsg.sdc_relay_closed = false;
    stateMsg.ts_activated = false;
    stateMsg.ebs_valve_front_activated = true;
    stateMsg.ebs_valve_rear_activated = true;
    stateMsg.watchdog = false;

    stateMsg.ebs_state = ebs.getEBSState();
    stateMsg.stfr = STFR::NoFailure;
    stateMsg.errors = { Error::E_None };

    Inverter initInverter;
    initInverter.is_ready = false;
    initInverter.rpm = 0;

    bmcSdcState = false;
    asmsState = ASMS::ASMS_Off;
    missionState = MissionState::MS_NotSelected;
    inverter1 = initInverter;
    inverter2 = initInverter;
    inverter3 = initInverter;
    inverter4 = initInverter;
    amsState = AMS::AMS_Idle;
    isFinished = false;
    icVoltage = 0;

    timeStamp = 0;
    lastTimeStamp = 0;
    timseSinceLastStateChange = 0;
    tsDeactivationDelayTime = 0;
}

/**
 * @brief Triggers the state change number 01 (Idle to ManualReady)
 *
 * Triggered through the internal TS button
 */
STFR VehicleStateVector::triggerVS_IdleToVS_ManualReady()
{
    if (getVehicleState() == VEHICLE_STATE::ManualReady)
    {
        return STFR::AlreadyInState;
    }
    if (getVehicleState() != VEHICLE_STATE::Idle)
    {
        return STFR::FalseInputState;
    }
    if (!bmcSdcState)
    {
        return STFR::SDCNotClosed;
    }
    if (missionState != MissionState::MS_Selected)
    {
        return STFR::NoMissionSelected;
    }
    if (asmsState != ASMS::ASMS_Off)
    {
        return STFR::ASMSNotRight;
    }
    if (!ebs.checkEBSTanksEmpty())
    {
        return STFR::EBSIsPressureized;
    }
    if (checkErrors())
    {
        return STFR::ErrorsExist;
    }
    changeToManualReady();
    return STFR::NoFailure;
}

/**
 * @brief Triggers the State Change number 2 (AS Off to AS Ready)
 *
 * Triggered through press of external TS button
 */
STFR VehicleStateVector::triggerVS_AsOffToVS_AsReady()
{
    if (getVehicleState() == VEHICLE_STATE::AsReady)
    {
        return STFR::AlreadyInState;
    }
    if (getVehicleState() != VEHICLE_STATE::AsOff)
    {
        return STFR::FalseInputState;
    }
    if (!bmcSdcState)
    {
        return STFR::SDCNotClosed;
    }
    if (missionState != MissionState::MS_Selected)
    {
        return STFR::NoMissionSelected;
    }
    if (asmsState != ASMS::ASMS_On)
    {
        return STFR::ASMSNotRight;
    }
    if (stateMsg.ebs_state != EBS::Armed_Parking)
    {
        return STFR::EBSStartupFailed;
    }
    if (checkForCrucialDVErrors())
    {
        return STFR::ErrorsExist;
    }

    changeToAsReady();
    return STFR::NoFailure;
}

/**
 * @brief Triggers the state change number 04 (ManualReady to Idle)
 *
 * Triggered through extented press of internal TS button
 */
STFR VehicleStateVector::triggerVS_ManualReadyToVS_Idle()
{
    if (getVehicleState() == VEHICLE_STATE::Idle)
    {
        return STFR::AlreadyInState;
    }
    if (getVehicleState() != VEHICLE_STATE::ManualReady)
    {
        return STFR::FalseInputState;
    }
    changeToIdle();
    return STFR::NoFailure;
}

/**
 * @brief Triggers the state change number 05 (ManualReady to ManualDrive)
 *
 * Triggered through the R2D button
 */
STFR VehicleStateVector::triggerVS_ManualReadyToVS_ManualDrive()
{
    if (getVehicleState() == VEHICLE_STATE::ManualDrive)
    {
        return STFR::AlreadyInState;
    }
    if (getVehicleState() != VEHICLE_STATE::ManualReady)
    {
        return STFR::FalseInputState;
    }
    if (amsState != AMS::AMS_Drive)
    {
        return STFR::AMSNotDrive;
    }
    if (!inverter1.is_ready || !inverter2.is_ready || !inverter3.is_ready || !inverter4.is_ready)
    {
        return STFR::InverterNotReady;
    }
    if (inverter1.rpm > WHEELSPEEDS_AT_STANDSTILL || inverter2.rpm > WHEELSPEEDS_AT_STANDSTILL
        || inverter3.rpm > WHEELSPEEDS_AT_STANDSTILL || inverter4.rpm > WHEELSPEEDS_AT_STANDSTILL)
    {
        return STFR::VehicleNotInStandstill;
    }
    if (ebs.getBrakePressureFront() < config.brakePressureForR2D
        || ebs.getBrakePressureRear() < config.brakePressureForR2D)
    {
        return STFR::BrakeTestFailed;
    }
    changeToManualDrive();
    return STFR::NoFailure;
}

/**
 * @brief Triggers the state change number 07 (ManualDrive to Idle)
 *
 * Triggered through long internal TS button press
 */
STFR VehicleStateVector::triggerVS_ManualDriveToVS_Idle()
{
    if (getVehicleState() == VEHICLE_STATE::Idle)
    {
        return STFR::AlreadyInState;
    }
    if (getVehicleState() != VEHICLE_STATE::ManualDrive)
    {
        return STFR::FalseInputState;
    }
    changeToIdle();
    return STFR::NoFailure;
}

/**
 * @brief Triggers the state change number 08 (ManualDrive to ManualReady)
 *
 * Triggered through press of R2D button
 */
STFR VehicleStateVector::triggerVS_ManualDriveToVS_ManualReady()
{
    if (getVehicleState() == VEHICLE_STATE::ManualReady)
    {
        return STFR::AlreadyInState;
    }
    if (getVehicleState() != VEHICLE_STATE::ManualDrive)
    {
        return STFR::FalseInputState;
    }
    changeToManualReady();
    return STFR::NoFailure;
}

/**
 * @brief Triggers the state change number 10 (AS Ready to AS Drive)
 *
 * Triggered through the press of the RES Go button
 */
STFR VehicleStateVector::triggerVS_AsReadyToVS_AsDrive()
{
    if (getVehicleState() == VEHICLE_STATE::AsDrive)
    {
        return STFR::AlreadyInState;
    }
    if (getVehicleState() != VEHICLE_STATE::AsReady)
    {
        return STFR::FalseInputState;
    }
    if (timseSinceLastStateChange < 5000)
    {
        return STFR::NotWaitedForASDrive;
    }
    changeToAsDrive();
    return STFR::NoFailure;
}

/**
 * @brief Triggers the state change number 11 (AS Ready to AS Emergency)
 *
 * Triggered through the RES Stop button
 */
STFR VehicleStateVector::triggerVS_AsReadyToVS_AsEmergency()
{
    if (getVehicleState() == VEHICLE_STATE::AsEmergency)
    {
        return STFR::AlreadyInState;
    }
    if (getVehicleState() != VEHICLE_STATE::AsReady)
    {
        return STFR::FalseInputState;
    }
    changeToAsEmergency();
    return STFR::NoFailure;
}

/**
 * @brief Triggers the state change number 12 (AS Drive to AS Emergency)
 *
 * Triggered through the RES Stop button
 */
STFR VehicleStateVector::triggerVS_AsDriveToVS_AsEmergency()
{
    if (getVehicleState() == VEHICLE_STATE::AsEmergency)
    {
        return STFR::AlreadyInState;
    }
    if (getVehicleState() != VEHICLE_STATE::AsDrive)
    {
        return STFR::FalseInputState;
    }
    changeToAsEmergency();
    return STFR::NoFailure;
}

/**
 * @brief Triggers the state change number 14 (AS Finished to AS Emergency)
 *
 * Triggered through the RES Stop button
 */
STFR VehicleStateVector::triggerVS_AsFinishedToVS_AsEmergency()
{
    if (getVehicleState() == VEHICLE_STATE::AsEmergency)
    {
        return STFR::AlreadyInState;
    }
    if (getVehicleState() != VEHICLE_STATE::AsFinished)
    {
        return STFR::FalseInputState;
    }
    changeToAsEmergency();
    return STFR::NoFailure;
}

/**
 * @brief Triggers the state change number 17 (Error to Idle)
 *
 * Triggered through press of R2D button
 */
STFR VehicleStateVector::triggerVS_ErrorToVS_Idle()
{
    // If errors are still existing we are jumping back to the error state in the next iteration
    resetErrors();

    if (getVehicleState() == VEHICLE_STATE::Idle)
    {
        return STFR::AlreadyInState;
    }
    if (getVehicleState() != VEHICLE_STATE::Error)
    {
        return STFR::FalseInputState;
    }
    if (checkErrors())
    {
        return STFR::ErrorsExist;
    }
    changeToIdle();
    return STFR::NoFailure;
}

/**
 * @brief Triggers the state change number 18 (Error to ManualReady)
 *
 * Triggered through press of TS button
 */
STFR VehicleStateVector::triggerVS_ErrorToVS_ManualReady()
{
    if (getVehicleState() == VEHICLE_STATE::ManualReady)
    {
        return STFR::AlreadyInState;
    }
    if (getVehicleState() != VEHICLE_STATE::Error)
    {
        return STFR::FalseInputState;
    }
    if (checkForCrucialErrors())
    {
        return STFR::ErrorsExist;
    }
    resetErrors();
    changeToManualReady();
    return STFR::NoFailure;
}

/**
 * @brief Triggers the state change number 20 (Idle to AS Off)
 *
 * Triggered through pressure in the ebs valves
 */
STFR VehicleStateVector::triggerVS_IdleToVS_AsOff()
{
    changeToAsOff();
    missionState = MissionState::MS_NotSelected;
    return STFR::NoFailure;
}

/**
 * @brief Periodically updates the actual state
 *
 * Gets triggered every 5ms.
 *
 * @param milliseconds The actual time
 */
void VehicleStateVector::updateState(uint32_t milliseconds)
{
    lastTimeStamp = timeStamp;
    timeStamp = milliseconds;
    timseSinceLastStateChange += timeStamp - lastTimeStamp;

    bool ebsTanksReleased = ebs.getEBSPressureFront() < config.maxReleasedEBSPressureFront
        && ebs.getEBSPressureRear() < config.maxReleasedEBSPressureRear;

    bool ebsTanksPressureized = ebs.getEBSPressureFront() > config.minPressurizedEBSPressureFront
        && ebs.getEBSPressureRear() > config.minPressurizedEBSPressureRear;

    bool errorsExist = checkErrors();

    bool brakesReleased = ebs.getBrakePressureFront() < config.maxReleasedBrakePressureFront
        && ebs.getBrakePressureRear() < config.maxReleasedBrakePressureRear;

    // At least one brake must be engaged
    bool brakesBraking = ebs.getBrakePressureFront() > config.minPressurizedBrakePressureFront
        || ebs.getBrakePressureRear() > config.minPressurizedBrakePressureRear;

    switch (stateMsg.state)
    {
    case (VEHICLE_STATE::Idle):
        checkTsDeactivation();
        stateMsg.sdc_relay_closed = compareMissionState(MissionState::MS_Selected) && ebsTanksReleased
            && compareEBSState(EBS::Disabled_Manual);

        stateMsg.sdc_activated = (compareASMSState(ASMS::ASMS_Off) && !stateMsg.ts_activated);

        if (ebsTanksPressureized)
        {
            stateMsg.stfr = triggerVS_IdleToVS_AsOff();
        }

        if (errorsExist)
        {
            setSTFR(STFR::ErrorsExist);
            changeToError();
        }
        checkDischarge();
        break;
    case (VEHICLE_STATE::AsOff):
        if (getEBSState() == EBS::Startup)
        {
            stateMsg.sdc_relay_closed = true;
        }
        else
        {
            stateMsg.sdc_relay_closed
                = (compareMissionState(MissionState::MS_Selected) && compareASMSState(ASMS::ASMS_On) && brakesBraking);
        }

        if (ebsTanksReleased)
        {
            setSTFR(STFR::NoFailure);
            changeToIdle();
        }
        if (checkForCrucialDVErrors())
        {
            setSTFR(STFR::ErrorsExist);
            changeToError();
        }
        if (getEBSState() == EBS::Armed_Parking)
        {
            checkEBS();
        }
        break;
    case (VEHICLE_STATE::AsReady):
        if (!compareASMSState(ASMS::ASMS_On) || checkForCrucialDVErrors() || accuFailureTest() || !getbmcSdcState())
        {
            setSTFR(STFR::ErrorsExist);
            changeToAsEmergency();
        }
        checkEBS();
        break;
    case (VEHICLE_STATE::AsDrive):
        if (!compareAMSState(AMS::AMS_Drive) || !compareASMSState(ASMS::ASMS_On) || checkForCrucialDVErrors()
            || !getbmcSdcState())
        {
            setSTFR(STFR::ErrorsExist);
            changeToAsEmergency();
        }
        if (isVehicleAtStandstill() && compareEBSState(EBS::Armed) && getIsFinished())
        {
            setSTFR(STFR::NoFailure);
            changeToAsFinished();
        }
        checkEBS();
        break;
    case VEHICLE_STATE::AsFinished:
        checkTsDeactivation();
        if (compareASMSState(ASMS::ASMS_Off) && brakesReleased)
        {
            setSTFR(STFR::NoFailure);
            changeToIdle();
        }
        if (compareASMSState(ASMS::ASMS_On))
            checkEBS();
        break;
    case VEHICLE_STATE::AsEmergency:
        checkTsDeactivation();
        if (compareASMSState(ASMS::ASMS_Off) && brakesReleased && timseSinceLastStateChange > WAIT_AFTER_EMERGENCY)
        {
            setSTFR(STFR::NoFailure);
            changeToIdle();
        }
        break;
    case VEHICLE_STATE::ManualReady:
        if ((timseSinceLastStateChange > config.normalPrechargeTime && compareAMSState(AMS::AMS_Idle))
            || ((timseSinceLastStateChange > config.normalPrechargeTime) && (!getbmcSdcState())))
        {
            setSTFR(STFR::NoFailure);
            changeToIdle();
        }
        if (accuFailureTest())
        {
            setSTFR(STFR::AMSNotDrive); // Important for faster TS deactivation in case of accumulator failure
            changeToError();
        }
        if (compareASMSState(ASMS::ASMS_On) || checkForCrucialErrors())
        {
            setSTFR(STFR::ErrorsExist);
            changeToError();
        }
        break;
    case VEHICLE_STATE::ManualDrive:
        if (!getbmcSdcState())
        {
            setSTFR(STFR::NoFailure);
            changeToIdle();
        }
        if (!compareAMSState(AMS::AMS_Drive) || compareASMSState(ASMS::ASMS_On) || checkForCrucialErrors())
        {
            if (compareAMSState(AMS::AMS_Idle))
            {
                setSTFR(STFR::NoFailure);
                changeToIdle();
            }
            else
            {
                changeToError();
                if (!compareAMSState(AMS::AMS_Drive))
                    setSTFR(STFR::AMSNotDrive);
                if (compareASMSState(ASMS::ASMS_On))
                    setSTFR(STFR::ASMSNotRight);
                if (checkForCrucialErrors())
                    setSTFR(STFR::ErrorsExist);
            }
        }
        break;
    case VEHICLE_STATE::Error:
        checkTsDeactivation();
        break;
    default:
        std::cout << "[ERROR] Invalid actual vehicle state: " << stateMsg.state << std::endl;
        break;
    }

    updateEBS();
    if (ebs.isWatchdogDeactivated())
        stateMsg.watchdog = false;
    else
        stateMsg.watchdog = !stateMsg.watchdog;
}

void VehicleStateVector::changeToIdle()
{
    stateMsg.state = VEHICLE_STATE::Idle;
    stateMsg.sdc_activated = true;
    stateMsg.sdc_relay_closed = false;
    // stateMsg.ts_activated = false; This kills the AIRs, must be delayed
    tsDeactivationDelayTime = timeStamp;
    stateMsg.actuators_allowed = false;

    ebs.resetEBS();
    checkEBS();

    stateMsg.errors = { Error::E_None };

    missionState = MissionState::MS_NotSelected;
    isFinished = false;

    timseSinceLastStateChange = 0;
}

void VehicleStateVector::changeToAsOff()
{
    stateMsg.state = VEHICLE_STATE::AsOff;
    stateMsg.sdc_activated = true;
    stateMsg.sdc_relay_closed = false;
    stateMsg.actuators_allowed = false;

    ebs.changeToAsOff();
    checkEBS();

    stateMsg.errors = { Error::E_None };

    timseSinceLastStateChange = 0;
}

void VehicleStateVector::changeToAsReady()
{
    stateMsg.state = VEHICLE_STATE::AsReady;
    stateMsg.sdc_activated = true;
    stateMsg.sdc_relay_closed = true;
    stateMsg.ts_activated = true;
    stateMsg.actuators_allowed = false;

    stateMsg.ebs_valve_front_activated = ebs.getEBSValveFront();
    stateMsg.ebs_valve_rear_activated = ebs.getEBSValveRear();

    timseSinceLastStateChange = 0;
}

void VehicleStateVector::changeToAsDrive()
{
    stateMsg.state = VEHICLE_STATE::AsDrive;
    stateMsg.sdc_activated = true;
    stateMsg.sdc_relay_closed = true;
    stateMsg.ts_activated = true;

    ebs.changeToAsDrive();
    checkEBS();

    timseSinceLastStateChange = 0;
}

void VehicleStateVector::changeToAsFinished()
{
    stateMsg.state = VEHICLE_STATE::AsFinished;
    stateMsg.sdc_activated = true;
    stateMsg.sdc_relay_closed = false;
    // stateMsg.ts_activated = false; This kills the AIRs, must be delayed
    tsDeactivationDelayTime = timeStamp;
    stateMsg.actuators_allowed = false;

    ebs.changeToAsFinished();
    checkEBS();

    timseSinceLastStateChange = 0;
}

void VehicleStateVector::changeToAsEmergency()
{
    stateMsg.state = VEHICLE_STATE::AsEmergency;
    stateMsg.sdc_activated = false;
    stateMsg.sdc_relay_closed = false;
    // stateMsg.ts_activated = false; This kills the AIRs, must be delayed
    tsDeactivationDelayTime = timeStamp;
    stateMsg.actuators_allowed = false;

    // Most important here!
    ebs.triggerEmergency();
    checkEBS();

    timseSinceLastStateChange = 0;
}

void VehicleStateVector::changeToManualReady()
{
    stateMsg.state = VEHICLE_STATE::ManualReady;
    stateMsg.sdc_activated = true;
    stateMsg.sdc_relay_closed = true;
    stateMsg.ts_activated = true;
    stateMsg.actuators_allowed = false;

    timseSinceLastStateChange = 0;
}

void VehicleStateVector::changeToManualDrive()
{
    stateMsg.state = VEHICLE_STATE::ManualDrive;
    stateMsg.sdc_activated = true;
    stateMsg.sdc_relay_closed = true;
    stateMsg.ts_activated = true;

    timseSinceLastStateChange = 0;
}

void VehicleStateVector::changeToError()
{
    stateMsg.state = VEHICLE_STATE::Error;
    stateMsg.sdc_activated = false;
    stateMsg.sdc_relay_closed = false;
    // stateMsg.ts_activated = false; This kills the AIRs, must be delayed
    tsDeactivationDelayTime = timeStamp;
    stateMsg.actuators_allowed = false;

    setMissionState(MissionState::MS_NotSelected);
    ebs.resetEBS();
    checkEBS();

    timseSinceLastStateChange = 0;
}

/**
 * @brief Checking the state of the EBS and setting the state message accordingly
 */
void VehicleStateVector::checkEBS()
{
    ebs.check();
    stateMsg.ebs_state = ebs.getEBSState();
    stateMsg.ebs_valve_front_activated = ebs.getEBSValveFront();
    stateMsg.ebs_valve_rear_activated = ebs.getEBSValveRear();

    if (stateMsg.ebs_state == EBS::Activated && stateMsg.state != VEHICLE_STATE::AsEmergency)
    {
        setSTFR(STFR::EBSError);
        changeToAsEmergency();
        return;
    }
}

void VehicleStateVector::updateEBS()
{
    ebs.updateTime(timeStamp);
    stateMsg.ebs_state = ebs.getEBSState();
    stateMsg.ebs_valve_front_activated = ebs.getEBSValveFront();
    stateMsg.ebs_valve_rear_activated = ebs.getEBSValveRear();
}

bool VehicleStateVector::accuFailureTest()
{
    bool amsIsPrecharging
        = compareAMSState(AMS::AMS_Precharge) || compareAMSState(AMS::AMS_Drive) || compareAMSState(AMS::AMS_Idle);

    if (timseSinceLastStateChange <= config.maxPrechargeTime && !amsIsPrecharging)
    {
        return true;
    }
    if (timseSinceLastStateChange > config.maxPrechargeTime && !compareAMSState(AMS::AMS_Drive))
    {
        return true;
    }
    return false;
}

bool VehicleStateVector::containsError(Error error)
{

    for (Error e : stateMsg.errors)
    {
        if (e == error)
            return true;
    }
    return false;
}

bool VehicleStateVector::checkErrors()
{
    if (stateMsg.errors.size() == 1)
    {
        if (stateMsg.errors.front() == Error::E_None || stateMsg.errors.front() == Error::E_InvNotReady)
            return false;
    }
    return true;
}

bool VehicleStateVector::checkForCrucialErrors()
{
    for (Error e : stateMsg.errors)
    {
        switch (e)
        {
        case Error::E_BrakePressureFront:
            return true;
        case Error::E_BrakePressureRear:
            return true;
        case Error::E_SSBFrontTimeout:
            return true;
        case Error::E_AMSTimeout:
            setSTFR(STFR::AMSNotDrive); // Faster TS deactivation
            return true;
        default:
            continue;
        }
    }
    return false;
}

bool VehicleStateVector::checkForCrucialDVErrors()
{
    for (Error e : stateMsg.errors)
    {
        switch (e)
        {
        case Error::E_BrakePressureFront:
            return true;
        case Error::E_BrakePressureRear:
            return true;
        case Error::E_EBSFront:
            return true;
        case Error::E_EBSRear:
            return true;
        case Error::E_SSBFrontTimeout:
            return true; // Really bad for DV
        case Error::E_AMSTimeout:
            setSTFR(STFR::AMSNotDrive); // Faster TS deactivation
            return true;
        case Error::E_ASSMTimeout:
            return true;
        case Error::E_DVTimeout:
            return true;
        default:
            continue;
        }
    }
    return false;
}

/**
 * @brief Activate discharge when TS not activated but IC voltage over 60 V
 */
void VehicleStateVector::checkDischarge()
{
    if (stateMsg.ts_activated)
        return;
    bool activateDischarge = false;
    if (stateMsg.sdc_activated && icVoltage > RULES_DISCHARGE)
    {
        activateDischarge = true;
    }
    if (icVoltage < MAX_DISCHARGED_IC_VOLTAGE)
    {
        activateDischarge = false;
    }
    if (activateDischarge)
        stateMsg.sdc_activated = false;
}

void VehicleStateVector::addError(Error error)
{

    stateMsg.errors.remove(Error::E_None);
    this->stateMsg.errors.push_back(error);
    stateMsg.errors.sort();
    stateMsg.errors.unique();
}

void VehicleStateVector::removeError(Error error) { stateMsg.errors.remove(error); }

void VehicleStateVector::resetErrors()
{
    this->stateMsg.errors.clear();
    stateMsg.errors.push_back(Error::E_None);
}

void VehicleStateVector::setASMSState(ASMS asmsState)
{
    this->asmsState = asmsState;
    if (asmsState == ASMS::ASMS_Off)
        return;

    // Starting EBS Startup Sequence when ASMS is switched on
    if (getVehicleState() == VEHICLE_STATE::AsOff && getEBSState() == EBS::Disabled)
        getEBS()->startESUS(0);
}

/**
 * @brief Function that delays the TS deactivation by some time to prevent the AIRs from welding themself up
 *
 * @details In the case of Accumulator failures or errors the TS is only delayed for TS_DEACTIVATION_DELAY (Std: 25ms)
 *          With other failures the TS is deactivated after bmcSdcState from the accumulator gets low +
 *          TS_DEACTIVATION_DELAY (Std: 25ms)
 */
void VehicleStateVector::checkTsDeactivation()
{
    if (getSTFR() == STFR::AMSNotDrive)
    {
        if (timeStamp - tsDeactivationDelayTime > TS_DEACTIVATION_DELAY)
        {
            this->stateMsg.ts_activated = false;
            tsDeactivationDelayTime = 0;
        }
    }
    else
    {
        if (getbmcSdcState())
        {
            tsDeactivationDelayTime = timeStamp;
        }
        if (timeStamp - tsDeactivationDelayTime > TS_DEACTIVATION_DELAY)
        {
            this->stateMsg.ts_activated = false;
            tsDeactivationDelayTime = 0;
        }
    }
}
