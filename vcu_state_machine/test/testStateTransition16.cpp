#include "testHelper.cpp"
#include <StateMachineData.hpp>
#include <gtest/gtest.h>

// AS Finished
//         - SDC on
//         - SDC relay off
//         - TS deactivated
//         - ebs valve 1 deactivated
//         - ebs valve 2 deactivated
//         _ ebs state armed
//         _ ebs start up sequence 7?
//         _ no errors

//         - missionSelected = dnc
//         - SDC ready = true
//         - SDC End = false
//         - ASMS on
//         - brake pressure front = ebs 1
//         - brake pressure rear = ebs 2
//         - ebs pressure 1 = > x bar (5)
//         - ebs pressure 2 = > x bar (5)
//         - inverters = 0 and ready
//         - amsState = Idle
//         - IC Voltage = 0

// Idle State:
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

TEST(state_transition_16, vs_asEmergency_to_vs_idle)
{
    // Data Setup
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(4);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(4);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedBrakePressureRear + 5);

    vsv.setMissionState(MissionState::MS_Selected);
    vsv.setASMSState(ASMS::ASMS_On);
    vsv.setAMSState(AMS::AMS_Idle);
    vsv.setbmcSdcState(false);
    vsv.setSDCReady(true);

    Inverter inv;
    inv.is_ready = true;
    inv.rpm = 0;

    vsv.setInverterFLState(inv);
    vsv.setInverterFRState(inv);
    vsv.setInverterRLState(inv);
    vsv.setInverterRRState(inv);

    vsv.setEBSState(EBS::Armed);
    vsv.setIsFinished(true);

    vsv.changeToAsEmergency();

    vsv.updateState(5);

    // Pretests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsEmergency);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Activated);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    // Start with the state transition
    vsv.setASMSState(ASMS::ASMS_Off);

    vsv.setEBSPressureFront(0);
    vsv.setBrakePressureFront(0);
    vsv.setEBSPressureRear(0);
    vsv.setBrakePressureRear(0);

    vsv.updateState(10005);
    // Posttests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::Idle);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled_Manual);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
}

TEST(state_transition_16, st_16_error_to_fast_released)
{
    // Data Setup
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(4);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(4);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedBrakePressureRear + 5);

    vsv.setMissionState(MissionState::MS_Selected);
    vsv.setASMSState(ASMS::ASMS_On);
    vsv.setAMSState(AMS::AMS_Idle);
    vsv.setbmcSdcState(false);
    vsv.setSDCReady(true);

    Inverter inv;
    inv.is_ready = true;
    inv.rpm = 0;

    vsv.setInverterFLState(inv);
    vsv.setInverterFRState(inv);
    vsv.setInverterRLState(inv);
    vsv.setInverterRRState(inv);

    vsv.setEBSState(EBS::Armed);
    vsv.setIsFinished(true);

    vsv.changeToAsEmergency();

    vsv.updateState(5);

    // Pretests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsEmergency);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Activated);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    // Start with the state transition
    vsv.setASMSState(ASMS::ASMS_Off);

    vsv.setEBSPressureFront(0);
    vsv.setBrakePressureFront(0);
    vsv.setEBSPressureRear(0);
    vsv.setBrakePressureRear(0);

    vsv.updateState(10);
    // Posttests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsEmergency);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Activated);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
}
