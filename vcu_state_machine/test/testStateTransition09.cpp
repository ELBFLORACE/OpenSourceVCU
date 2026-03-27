#include "testHelper.cpp"
#include <StateMachineData.hpp>
#include <gtest/gtest.h>
#include <thread>

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

TEST(state_transition_09, vs_manualDrive_to_vs_error)
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

    // State Transition
    vsv.addError(Error::E_AMSTimeout);
    vsv.updateState(10);

    // Posttests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::Error);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled_Manual);

    // Test the AIRSchoner
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    int timeToWait = vsv.getConfig()->normalPrechargeTime + TS_DEACTIVATION_DELAY + 10;
    vsv.setbmcSdcState(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(TS_DEACTIVATION_DELAY + 5));
    vsv.updateState(timeToWait);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);

    list<Error> p_errors = { Error::E_AMSTimeout };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, p_errors);
    ASSERT_EQ(vsv.getMissionState(), MissionState::MS_NotSelected);
}

TEST(state_transition_09, st_9_AMS_Error)
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

    // State Transition
    vsv.setAMSState(AMS::AMS_Data_Error);
    vsv.updateState(10);

    // Posttests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::Error);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled_Manual);

    // Test the AIRSchoner
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);

    vsv.updateState(15 + TS_DEACTIVATION_DELAY);
    std::this_thread::sleep_for(std::chrono::milliseconds(TS_DEACTIVATION_DELAY + 5));

    // Posttests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::Error);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled_Manual);

    // Test the AIRSchoner
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);

    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
    ASSERT_EQ(vsv.getMissionState(), MissionState::MS_NotSelected);
}
