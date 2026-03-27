/**
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * @file enums.hpp
 *
 * @brief Defining the enums for the vcu_shared_lib
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

#ifndef vcu_shared_lib_ENUMS_HPP
#define vcu_shared_lib_ENUMS_HPP

#include <cstdint>
#include <string>

namespace vcu_shared_lib
{
// Sub-namespace only necessary to overcome namespace pollution
// of the Error variant in vehicle state and ebs state
namespace ev_states
{
    enum VEHICLE_MISSION : uint8_t
    {
        INSPECTION = 0,
        EBS_TEST,
        SKIDPAD,
        ACCELERATION,
        AUTOCROSS,
        TRACKDRIVE,
        SCRUTI,
        BRAKETEST,
        ENDURANCE,
        TEAMDRIVING,
    };

    enum VEHICLE_STATE : uint8_t
    {
        Idle = 0,
        AsOff,
        AsReady,
        AsDrive,
        AsFinished,
        AsEmergency,
        ManualReady,
        ManualDrive,
        Error
    };
}

enum EBS_STATE : uint8_t
{
    Activated = 0,
    Armed,
    Armed_Parking,
    Armed_Finished,
    Disabled,
    Disabled_Manual,
    Error,
    Startup_Error,
    Startup
};
}

using VEHICLE_MISSION = vcu_shared_lib::ev_states::VEHICLE_MISSION;
using VM = VEHICLE_MISSION;

using VEHICLE_STATE = vcu_shared_lib::ev_states::VEHICLE_STATE;
using VS = VEHICLE_STATE;
std::string vehicleStateEnumToString(VEHICLE_STATE state);

using EBS_STATE = vcu_shared_lib::EBS_STATE;
using EBS = EBS_STATE;
std::string ebsStateEnumToString(EBS state);

#endif
