/**
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * @file Utils.hpp
 *
 * @brief Utils for the debug messages of the state machine
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

#ifndef UTILS_H
#define UTILS_H

#include <StateMachineData.hpp>
#include <math.h>
#include <vector>

std::vector<std::string> errorsConverter(std::list<Error> errors)
{
    std::vector<std::string> out;

    for (Error e : errors)
    {
        switch (e)
        {
        case E_None:
            out.push_back("0_None");
            break;
        case E_PDUTimeout:
            out.push_back("1_PDUTimeout");
            break;
        case E_APPSImplausible:
            out.push_back("2_AppsImplausible");
            break;
        case E_Apps1:
            out.push_back("3_Apps1");
            break;
        case E_Apps2:
            out.push_back("4_Apps2");
            break;
        case E_BrakePressureFront:
            out.push_back("5_BrakePressureFront");
            break;
        case E_BrakePressureRear:
            out.push_back("6_BrakePressureRear");
            break;
        case E_BrakeForceSensor:
            out.push_back("7_BrakeForceSensor");
            break;
        case E_SSBFrontTimeout:
            out.push_back("8_SSBFrontTimeout");
            break;
        case E_SSBRearTimeout:
            out.push_back("9_SSBRearTimeout");
            break;
        case E_BSETimeout:
            out.push_back("10_BSETimeout");
            break;
        case E_AMSTimeout:
            out.push_back("11_AMSTimeout");
            break;
        case E_EnergymeterTimeout:
            out.push_back("12_EnergymeterTimeout");
            break;
        case E_DISTimeout:
            out.push_back("13_DISTimeout");
            break;
        case E_InverterTimeout:
            out.push_back("14_InverterTimeout");
            break;
        case E_ASSMTimeout:
            out.push_back("15_ASSMTimeout");
            break;
        case E_DVTimeout:
            out.push_back("16_DVTimeout");
            break;
        case E_InvNotReady:
            out.push_back("17_InvNotReady");
            break;
        default:
            std::cout << "[ERROR] No valid error value (PubDebug)" << std::endl;
            out.push_back("Invalid");
        }
    }

    return out;
}

std::string stfrConverter(STFR stfr)
{
    std::string out;
    switch (stfr)
    {
    case STFR::NoFailure:
        out = "0_NoFailure";
        break;
    case STFR::SDCNotClosed:
        out = "1_SDCNotClosed";
        break;
    case STFR::NoMissionSelected:
        out = "2_NoMissionSelected";
        break;
    case STFR::ASMSNotRight:
        out = "3_ASMSNotRight";
        break;
    case STFR::EBSStartupFailed:
        out = "4_EBSStartupFailed";
        break;
    case STFR::ErrorsExist:
        out = "5_ErrorsExist";
        break;
    case STFR::AMSNotDrive:
        out = "6_AMSNotDrive";
        break;
    case STFR::InverterNotReady:
        out = "7_InverterNotReady";
        break;
    case STFR::VehicleNotInStandstill:
        out = "8_VehicleNotInStandstill";
        break;
    case STFR::BrakeTestFailed:
        out = "9_BrakeTestFailed";
        break;
    case STFR::NotWaitedForASDrive:
        out = "10_NotWaitedForASDrive";
        break;
    case STFR::AlreadyInState:
        out = "11_AlreadyInState";
        break;
    case STFR::FalseInputState:
        out = "12_FalseInputState";
        break;
    case STFR::EBSIsPressureized:
        out = "13_EBSIsPressureized";
        break;
    case STFR::EBSError:
        out = "14_EBSError";
        break;
    default:
        std::cout << "[ERROR] (stfrConverter) No valid STFR state: " << stfr << std::endl;
        out = "Invalid";
    }
    return out;
}

string esusStateEnumToString(ESUS esus)
{

    std::string esusString;
    switch (esus)
    {
    case WaitForASSM:
        esusString = "0_WaitForASSM";
        break;
    case CheckWatchdog:
        esusString = "1_CheckWatchdog";
        break;
    case CheckAllPressure:
        esusString = "2_CheckAllPressure";
        break;
    case CheckFrontBrakeRelease:
        esusString = "3_CheckFrontBrakeRelease";
        break;
    case CheckFrontBrakeActivation:
        esusString = "4_CheckFrontBrakeActivation";
        break;
    case CheckRearBrakeRelease:
        esusString = "5_CheckFrontBrakeRelease";
        break;
    case CheckRearBrakeActivation:
        esusString = "6_CheckRearBrakeActivation";
        break;
    case ESUS_Ok:
        esusString = "7_ESUS_Ok";
        break;
    case ESUS_Error:
        esusString = "8_ESUS_Error";
        break;
    default:
        std::cout << "[ERROR] No valid esus state (ESUSEnumToString)" << std::endl;
        esusString = "9_Invalid";
    }
    return esusString;
}

uint32_t errorsIntConverter(std::list<Error> errors)
{
    uint32_t out = 0;
    for (Error error : errors)
        out += pow(2.0, (double)error);

    return out;
}

#endif
