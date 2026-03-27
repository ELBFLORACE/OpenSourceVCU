#include "testHelper.cpp"
#include <StateMachineData.hpp>
#include <gtest/gtest.h>

// Normal Idle State:
//         - SDC on
//         - SDC relay on
//         - TS disabled
//         - ebs valve 1 activated (for brake pressure 0)
//         - ebs valve 2 activated
//         _ ebs state disabled
//         _ ebs start up sequence 0
//         _ no errors

//         - missionSelected = can be set immediately
//         - SDC ready = false as long as mission not set
//         - SDC End = false as long as mission not set
//         - !!! ASMS off
//         - brake pressure front = dnc
//         - brake pressure rear = dnc
//         - inverters = dnc | 0
//         - amsState = Idle?
//         - IC Voltage = 0 | dnc ?

// Error State:
//         - SDC off
//         - SDC relay off
//         - TS disabled
//         - ebs valve 1 deactivated (for brake pressure 0)
//         - ebs valve 2 deactivated
//         _ ebs state disabled manual
//         _ ebs start up sequence 0
//         _ no errors

//         - missionSelected = false
//         - SDC ready = false
//         - SDC End = false
//         - ASMS dnc
//         - brake pressure front = dnc
//         - brake pressure rear = dnc
//         - inverters = dnc | 0
//         - amsState = Idle
//         - IC Voltage = 0

TEST(state_transition_21, idle_to_error)
{
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(0);
    vsv.setEBSPressureRear(0);

    vsv.changeToIdle();

    // Normal Idle State:
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::Idle);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled_Manual);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    //         - missionSelected = can be set immediately
    vsv.setMissionState(MissionState::MS_Selected);
    vsv.updateState(5);

    // Only if mission is set and EBS Pressure 0 possible
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);

    vsv.setSDCReady(true);
    vsv.setbmcSdcState(true);
    vsv.setASMSState(ASMS::ASMS_Off);

    vsv.updateState(10);

    // State Transition
    vsv.addError(Error::E_AMSTimeout);
    vsv.updateState(15);

    // Posttests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::Error);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled_Manual);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    list<Error> p_errors = { Error::E_AMSTimeout };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, p_errors);
    ASSERT_EQ(vsv.getMissionState(), MissionState::MS_NotSelected);
}
