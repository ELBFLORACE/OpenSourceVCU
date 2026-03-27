#include "testHelper.cpp"
#include <StateMachineData.hpp>
#include <gtest/gtest.h>

// AS Off State:
//         - SDC on
//         - SDC relay off -> on with set mission, asms, engaged brakes
//         - TS deactivated
//         - ebs valve 1 deactivated
//         - ebs valve 2 deactivated
//         _ ebs state disabled
//         _ ebs start up sequence 0
//         _ no errors

//         - SDC ready = true
//         - SDC End = false
//         - !!! ASMS on
//         - !!! ebs pressure 1 = > 0
//         - !!! ebs pressure 2 = > 0
//         - brake pressure front = ebs front
//         - brake pressure rear = ebs rear
//         - missionSelected = dnc
//         - inverters = dnc | 0
//         - amsState = Idle
//         - IC Voltage = 0

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

TEST(state_transition_03, asOff_to_error)
{
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);

    vsv.changeToAsOff();

    vsv.setBrakePressureFront(vsv.getConfig()->minPressurizedBrakePressureFront + 5);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedBrakePressureRear + 5);

    // Pretests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsOff);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
    ASSERT_EQ(vsv.getMissionState(), MissionState::MS_NotSelected);

    // State Transition
    vsv.addError(Error::E_AMSTimeout);
    vsv.updateState(5);

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
