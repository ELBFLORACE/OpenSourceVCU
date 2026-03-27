#include "testHelper.cpp"
#include <StateMachineData.hpp>
#include <gtest/gtest.h>

// Manual Drive State:
//         - SDC on
//         - SDC relay on
//         - TS activated
//         - ebs valve 1 activated (for brake pressure 0)
//         - ebs valve 2 activated
//         _ ebs state disabled manual
//         _ ebs start up sequence 0
//         _ no errors

//         - missionSelected = true
//         - SDC ready = true
//         - SDC End = true
//         - !!! ASMS off
//         - brake pressure front = dnc
//         - brake pressure rear = dnc
//         - inverters = ready | 0
//         - amsState = Drive
//         - IC Voltage = dnc

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

TEST(state_transition_08, vs_manualDrive_to_vs_manualReady)
{

    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(0);
    vsv.setEBSPressureRear(0);

    vsv.setSDCReady(true);
    vsv.setbmcSdcState(true);
    vsv.setASMSState(ASMS::ASMS_Off);
    vsv.setMissionState(MissionState::MS_Selected);
    vsv.setEBSState(EBS::Disabled_Manual);
    vsv.checkEBS();

    Inverter inv;
    inv.is_ready = true;
    inv.rpm = 0;

    vsv.setInverterFLState(inv);
    vsv.setInverterFRState(inv);
    vsv.setInverterRLState(inv);
    vsv.setInverterRRState(inv);

    vsv.setAMSState(AMS::AMS_Drive);
    vsv.setBrakePressureFront(5);

    vsv.changeToManualDrive();
    vsv.updateState(5);

    // Normal Idle State:
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::ManualDrive);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled_Manual);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    /* Preparing for switch to ManualReady */
    vsv.triggerVS_ManualDriveToVS_ManualReady();

    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::ManualReady);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled_Manual);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
}
