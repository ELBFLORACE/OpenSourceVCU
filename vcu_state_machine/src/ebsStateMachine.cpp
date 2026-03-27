/**
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * @file ebsStateMachine.cpp
 *
 * @brief Implementing the logic for the EBS state machine
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

#include <EBSStateMachine.hpp>
#include <chrono>
#include <iostream>
#include <thread>

/**
 * @brief Starting the EBS start up sequence in an own thread
 *
 * @param id from where the function is called
 */
void EBSStateMachine::startESUS(int id)
{
    std::cout << "[DEBUG] Starting EBS Startup Sequence. [ID" << id << "]" << std::endl;
    std::thread thread_esus(esusCycle, this, id);

    thread_esus.detach();
}

/**
 * @brief The actual ebs start up
 *
 * @param esm The EBS state machine to control the valves
 * @param id The if from where the function is called
 */
void esusCycle(EBSStateMachine* esm, int id)
{

    std::cout << "[DEBUG] Starting ebs startup cycle thread [ID" << id << "]" << std::endl;
    // --- Step 0: Set EBS state to Startup --- //
    esm->setESUSP(ESUS::WaitForASSM);
    esm->setEBSState(EBS::Startup);
    esm->setEbsValveFront(false);
    esm->setEbsValveRear(false);
    esm->resetTimeSinceLastTransition();
    int timeout = 0;
    // Loop checking for to low ebs pressures and the sequence states
    while (esm->getESUSP() != ESUS::ESUS_Ok)
    {
        // Continious check for enough ebs pressure in the ebs valves
        timeout++;
        if (esm->isEBSPressureTooLow())
        {
            esm->setESUSP(ESUS::ESUS_Error);
            esm->setEBSState(EBS::Error);
            return;
        }
        switch (esm->getESUSP())
        {

        // --- Step 1: Deactivate Watchdog --- //
        case ESUS::WaitForASSM:
            if (esm->isSDCReady())
            {
                esm->setDeactivateWatchdog(true);
                esm->setESUSP(ESUS::CheckWatchdog);
                esm->resetTimeSinceLastTransition();
                timeout = 0;
            }
            break;

        // --- Step 2: Check if Watchdog is deactivated (blocking for 1s) --- //
        case ESUS::CheckWatchdog:
            if (esm->getTimeSinceLastTransition() > WAIT_FOR_WATCHDOG)
            {
                esm->setEBSState(EBS::Startup_Error);
                return;
            }
            if (!esm->isSDCReady())
            {
                esm->setESUSP(ESUS::CheckAllPressure);
                esm->setDeactivateWatchdog(false);
                esm->resetTimeSinceLastTransition();
                timeout = 0;
            }
            break;

        // --- Step 3: Check if brakes and ebs tanks are pressurized --- //
        case ESUS::CheckAllPressure:
            if (!(esm->getBrakePressureFront() > esm->getEbsConfig().minPressurizedBrakePressureFront
                    && esm->getEBSPressureFront() > esm->getEbsConfig().minPressurizedEBSPressureFront
                    && esm->getBrakePressureRear() > esm->getEbsConfig().minPressurizedBrakePressureRear
                    && esm->getEBSPressureRear() > esm->getEbsConfig().minPressurizedEBSPressureRear))
            {
                esm->setEBSState(EBS::Startup_Error);
                return;
            }
            esm->setESUSP(ESUS::CheckFrontBrakeRelease);
            esm->resetTimeSinceLastTransition();

            // --- Step 3: Activate front valve --- //
            esm->setEbsValveFront(true);
            timeout = 0;
            break;

        // --- Step 4: Check if brake pressure front released -- //
        case ESUS::CheckFrontBrakeRelease:
            if (esm->getTimeSinceLastTransition() > WAIT_FOR_BOTH_BRAKE_RELEASE)
            {
                esm->setEBSState(EBS::Startup_Error);
                return;
            }
            if (esm->getBrakePressureFront() < esm->getEbsConfig().maxReleasedBrakePressureFront
                && esm->getBrakePressureRear() > esm->getEbsConfig().minPressurizedBrakePressureRear)
            {
                esm->setESUSP(ESUS::CheckFrontBrakeActivation);
                esm->resetTimeSinceLastTransition();
                timeout = 0;
                esm->setEbsValveFront(false);
            }
            break;

        // --- Step 5: Check if it brakes at the front --- //
        case ESUS::CheckFrontBrakeActivation:
            if (esm->getTimeSinceLastTransition() > WAIT_FOR_FRONT_BRAKE_ACTIVE)
            {
                esm->setEBSState(EBS::Startup_Error);
                return;
            }
            if (esm->getBrakePressureFront() > esm->getEbsConfig().minPressurizedBrakePressureFront)
            {
                esm->setESUSP(ESUS::CheckRearBrakeRelease);
                esm->resetTimeSinceLastTransition();
                timeout = 0;
                esm->setEbsValveRear(true);
            }
            break;

        // --- Step 6: Check if rear brakes are released --- //
        case ESUS::CheckRearBrakeRelease:
            if (esm->getTimeSinceLastTransition() > WAIT_FOR_BOTH_BRAKE_RELEASE)
            {
                esm->setEBSState(EBS::Startup_Error);
                return;
            }
            if (esm->getBrakePressureFront() > esm->getEbsConfig().minPressurizedBrakePressureFront
                && esm->getBrakePressureRear() < esm->getEbsConfig().maxReleasedBrakePressureRear)
            {
                esm->setESUSP(ESUS::CheckRearBrakeActivation);
                esm->resetTimeSinceLastTransition();
                timeout = 0;
                esm->setEbsValveRear(false);
            }
            break;

        // --- Step 7: Check if brake rear activated -- //
        case ESUS::CheckRearBrakeActivation:
            if (esm->getTimeSinceLastTransition() > WAIT_FOR_REAR_BRAKE_ACTIVE)
            {
                esm->setEBSState(EBS::Startup_Error);
                return;
            }
            if (esm->getBrakePressureRear() > esm->getEbsConfig().minPressurizedBrakePressureRear)
            {
                esm->setESUSP(ESUS::ESUS_Ok);
                esm->setEBSState(EBS::Armed_Parking);
            }
            break;

        default:
            esm->setESUSP(ESUS::ESUS_Error);
            esm->setEBSState(EBS::Error);
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (timeout > TIMEOUT_ESUS)
        {
            std::cout << "[ERROR] Timeout [ID" << id << "]" << std::endl;
            esm->setESUSP(ESUS::ESUS_Error);
            return;
        }
    }
    return;
}

/**
 *  @brief Check if EBS Pressure is beneath the config values
 */
bool EBSStateMachine::isEBSPressureTooLow()
{
    return ebsPressureFront < ebsConfig.minPressurizedEBSPressureFront
        || ebsPressureRear < ebsConfig.minPressurizedBrakePressureRear;
    {
    }
}

/**
 *  @brief Check if the EBS is still in the right state
 */
void EBSStateMachine::check()
{
    if (ebsState == EBS::Armed || ebsState == EBS::Armed_Parking)
    {
        if (ebsPressureFront <= ebsConfig.minPressurizedEBSPressureFront
            || ebsPressureRear <= ebsConfig.minPressurizedEBSPressureRear)
        {
            ebsState = EBS::Activated;
        }
    }
    if (ebsState == EBS::Startup)
    {
        if (ebsPressureFront <= ebsConfig.minPressurizedEBSPressureFront
            || ebsPressureRear <= ebsConfig.minPressurizedEBSPressureRear)
        {
            ebsState = EBS::Error;
        }
    }

    // Controlling the Valves
    switch (ebsState)
    {
    case EBS::Disabled:
        ebsValveFront = false;
        ebsValveRear = false;
        esusp = ESUS::WaitForASSM;
        break;
    case EBS::Disabled_Manual:
        ebsValveFront = true;
        ebsValveRear = true;
        esusp = ESUS::WaitForASSM;
        break;
    case EBS::Activated:
        ebsValveFront = false;
        ebsValveRear = false;
        break;
    case EBS::Armed:
        ebsValveFront = true;
        ebsValveRear = true;
        break;
    case EBS::Armed_Parking:
        ebsValveFront = true;
        ebsValveRear = false; // Parking brake
        break;
    case EBS::Armed_Finished:
        ebsValveFront = false;
        ebsValveRear = false; // active brake before sdc is off
        break;
    case EBS::Error:
        ebsValveFront = false;
        ebsValveRear = false;
        break;
    case EBS::Startup_Error:
        ebsValveFront = false;
        ebsValveRear = false;
        break;
    default:
        ebsValveFront = false;
        ebsValveRear = false;
        esusp = ESUS::ESUS_Error;
        break;
    }
}

void EBSStateMachine::triggerEmergency() { ebsState = EBS::Activated; }

void EBSStateMachine::resetEBS()
{
    ebsState = EBS::Disabled_Manual;
    esusp = ESUS::WaitForASSM;
}

void EBSStateMachine::changeToAsDrive() { ebsState = EBS::Armed; }

void EBSStateMachine::changeToAsOff() { ebsState = EBS::Disabled; }

void EBSStateMachine::changeToAsFinished() { ebsState = EBS::Armed_Finished; }

void EBSStateMachine::setSDCReady(bool v) { this->sdcReady = v; }

void EBSStateMachine::setConfig(EBSConfig ebsConfig) { this->ebsConfig = ebsConfig; }

void EBSStateMachine::setAllPressures(double ebsFront, double ebsRear, double brakeFront, double brakeRear)
{
    ebsPressureFront = ebsFront;
    ebsPressureRear = ebsRear;
    brakePressureFront = brakeFront;
    brakePressureRear = brakeRear;
}

bool EBSStateMachine::isEbsDeactivated() { return ebsState == EBS::Disabled || ebsState == EBS::Disabled_Manual; }

bool EBSStateMachine::checkEBSTanksEmpty()
{
    return ebsPressureFront < ebsConfig.maxReleasedEBSPressureFront
        && ebsPressureRear < ebsConfig.maxReleasedEBSPressureRear;
}
