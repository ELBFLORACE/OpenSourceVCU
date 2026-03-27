#include "testHelper.cpp"
#include <StateMachineData.hpp>
#include <gtest/gtest.h>

// Normal Idle State:
//         - SDC on
//         - SDC relay off
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

//     AS Off State:
//         - SDC on
//         - !!! SDC relay on -> only if mission selected
//         - TS disabled
//         - ebs valve 1 disabled (for brake pressure = ebs pressure)
//         - ebs valve 2 disabled
//         _ ebs state disabled
//         _ ebs start up sequence 0
//         _ no errors

//         - SDC ready = dnc -> can be true
//         - SDC End = dnc -> can be true
//         - !!! ASMS off
//         - !!! ebs pressure 1 = > 0
//         - !!! ebs pressure 2 = > 0
//         - brake pressure front = > 0
//         - brake pressure rear = > 0
//         - missionSelected = dnc?
//         - inverters = dnc | 0
//         - amsState = Idle?
//         - IC Voltage = 0 | dnc ?

TEST(state_transition_20, vs_idle_to_vs_as_off)
{

    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    //         - !!! ebs pressure 1 = 0
    //         - !!! ebs pressure 2 = 0
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

    //         - SDC ready = false as long as mission not set
    vsv.setSDCReady(true);
    //         - SDC End = false as long as mission not set
    vsv.setbmcSdcState(true);
    //         - !!! ASMS off
    vsv.setASMSState(ASMS::ASMS_Off);
    //         - brake pressure front = dnc
    //         - brake pressure rear = dnc
    //         - inverters = dnc | 0
    //         - amsState = Idle?
    //         - IC Voltage = 0 | dnc ?

    /* Preparing for switch to VS_AsOff */

    vsv.setEBSPressureFront(vsv.getConfig()->minPressurizedEBSPressureFront + 0.1);
    vsv.setEBSPressureRear(vsv.getConfig()->minPressurizedEBSPressureRear + 0.1);

    vsv.updateState(10);

    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsOff);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    ASSERT_EQ(vsv.getMissionState(), MissionState::MS_NotSelected);
}

TEST(state_transition_20, st20_ebs_pressure_to_low)
{

    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    //         - !!! ebs pressure 1 = 0
    //         - !!! ebs pressure 2 = 0
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

    //         - SDC ready = false as long as mission not set
    vsv.setSDCReady(true);
    //         - SDC End = false as long as mission not set
    vsv.setbmcSdcState(true);
    //         - !!! ASMS off
    vsv.setASMSState(ASMS::ASMS_Off);
    //         - brake pressure front = dnc
    //         - brake pressure rear = dnc
    //         - inverters = dnc | 0
    //         - amsState = Idle?
    //         - IC Voltage = 0 | dnc ?

    /* Preparing for switch to VS_AsOff */

    vsv.setEBSPressureFront(vsv.getConfig()->minPressurizedEBSPressureFront);
    vsv.setEBSPressureRear(vsv.getConfig()->minPressurizedEBSPressureRear - 1);

    vsv.updateState(10);

    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::Idle);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled_Manual);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
}

TEST(state_transition_20, activate_asms)
{

    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    //         - !!! ebs pressure 1 = 0
    //         - !!! ebs pressure 2 = 0
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

    //         - SDC ready = false as long as mission not set
    vsv.setSDCReady(true);
    //         - SDC End = false as long as mission not set
    vsv.setbmcSdcState(true);
    //         - !!! ASMS off
    vsv.setASMSState(ASMS::ASMS_On);
    vsv.updateState(10);
    //         - brake pressure front = dnc
    //         - brake pressure rear = dnc
    //         - inverters = dnc | 0
    //         - amsState = Idle?
    //         - IC Voltage = 0 | dnc ?
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, false);

    /* Preparing for switch to VS_AsOff */

    vsv.setEBSPressureFront(vsv.getConfig()->minPressurizedEBSPressureFront + 0.1);
    vsv.setEBSPressureRear(vsv.getConfig()->minPressurizedEBSPressureRear + 0.1);
    vsv.setBrakePressureFront(vsv.getConfig()->minPressurizedEBSPressureFront + 0.1);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedEBSPressureRear + 0.1);

    vsv.updateState(15);

    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsOff);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    ASSERT_EQ(vsv.getMissionState(), MissionState::MS_NotSelected);

    vsv.setMissionState(MissionState::MS_Selected);
    vsv.updateState(20);

    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsOff);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
}
