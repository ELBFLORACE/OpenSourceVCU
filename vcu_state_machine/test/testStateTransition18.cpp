#include "testHelper.cpp"
#include <StateMachineData.hpp>
#include <gtest/gtest.h>

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

//     Manual Ready State:
//         - SDC on
//         - SDC relay on
//         - TS activated
//         - ebs valve 1 activated (for brake pressure = ebs pressure)
//         - ebs valve 2 activated
//         _ ebs state disabled_manual
//         _ ebs start up sequence 0
//         _ no errors

//         - SDC ready = true
//         - SDC End = true
//         - !!! ASMS off
//         - !!! ebs pressure 1 = 0
//         - !!! ebs pressure 2 = 0
//         - brake pressure front = 0
//         - brake pressure rear = 0
//         - missionSelected = manual drive mission
//         - inverters = dnc | 0
//         - amsState = Drive
//         - IC Voltage = > 0

TEST(state_transition_18, error_to_manualReady)
{
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.changeToError();

    // Pretests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::Error);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled_Manual);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    list<Error> p_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, p_errors);
    ASSERT_EQ(vsv.getMissionState(), MissionState::MS_NotSelected);

    /* Preparing for switch to ManualReady */
    vsv.triggerVS_ErrorToVS_ManualReady();

    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::ManualReady);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled_Manual);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, p_errors);
}
