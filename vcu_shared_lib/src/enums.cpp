/**
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * @file enums.cpp
 *
 * @brief Implementation of the enum operations for the vcu_shared_lib
 *
 * @author Marvin Jacob <marvin.jacob@elbflorace.de> 2025-2026
 * @author Laurin Hesslich <laurin.hesslich@elbflorace.de> 2026
 *
 * @copyright MIT
 *
 * @version 1.0.0
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */
#include "vcu_shared_lib/enums.hpp"
#include <iostream>

std::string vehicleStateEnumToString(VEHICLE_STATE state)
{
    std::string stateString;
    switch (state)
    {
    case VEHICLE_STATE::Idle:
        stateString = "0_Idle";
        break;
    case VEHICLE_STATE::AsOff:
        stateString = "1_AsOff";
        break;
    case VEHICLE_STATE::AsReady:
        stateString = "2_AsReady";
        break;
    case VEHICLE_STATE::AsDrive:
        stateString = "3_AsDrive";
        break;
    case VEHICLE_STATE::AsFinished:
        stateString = "4_AsFinished";
        break;
    case VEHICLE_STATE::AsEmergency:
        stateString = "5_AsEmergency";
        break;
    case VEHICLE_STATE::ManualReady:
        stateString = "6_ManualReady";
        break;
    case VEHICLE_STATE::ManualDrive:
        stateString = "7_ManualDrive";
        break;
    case VEHICLE_STATE::Error:
        stateString = "8_Error";
        break;
    default:
        std::cout << "[ERROR] No valide vehicle state (VSEnumToString)" << std::endl;
        stateString = "9_Invalid";
    }
    return stateString;
}

std::string ebsStateEnumToString(EBS state)
{
    std::string stateString;
    switch (state)
    {
    case EBS::Activated:
        stateString = "0_Activated";
        break;
    case EBS::Armed:
        stateString = "1_Armed";
        break;
    case EBS::Armed_Parking:
        stateString = "2_ArmedParking";
        break;
    case EBS::Armed_Finished:
        stateString = "3_ArmedFinished";
        break;
    case EBS::Disabled:
        stateString = "4_Disabled";
        break;
    case EBS::Disabled_Manual:
        stateString = "5_DisabledManual";
        break;
    case EBS::Error:
        stateString = "6_Error";
        break;
    case EBS::Startup_Error:
        stateString = "7_StartupError";
        break;
    case EBS::Startup:
        stateString = "8_Startup";
        break;
    default:
        std::cout << "[ERROR] No valid ebs state (vcu_shared_lib::ebsStateEnumToString)" << std::endl;
        stateString = "9_Invalid";
    }
    return stateString;
}
