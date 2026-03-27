#include "testHelper.cpp"
#include <StateMachineData.hpp>
#include <gtest/gtest.h>
#include <thread>

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

TEST(state_transition_06, vs_manualReady_to_vs_error)
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

    vsv.changeToManualReady();
    vsv.updateState(5);

    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::ManualReady);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled_Manual);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    /* Preparing for switch to ManualReady */
    Inverter inv;
    inv.is_ready = true;
    inv.rpm = 0;

    vsv.setInverterFLState(inv);
    vsv.setInverterFRState(inv);
    vsv.setInverterRLState(inv);
    vsv.setInverterRRState(inv);

    vsv.setAMSState(AMS::AMS_Drive);
    vsv.updateState(10);

    // State Transition
    vsv.addError(Error::E_AMSTimeout);
    vsv.updateState(20);

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

TEST(state_transition_06, st06_precharging_to_long)
{

    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.changeToIdle();

    vsv.setEBSPressureFront(0);
    vsv.setEBSPressureRear(0);

    vsv.setSDCReady(true);
    vsv.setbmcSdcState(true);
    vsv.setASMSState(ASMS::ASMS_Off);
    vsv.setMissionState(MissionState::MS_Selected);
    vsv.setEBSState(EBS::Disabled_Manual);
    vsv.checkEBS();

    vsv.updateState(5);

    // Normal Idle State:
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::Idle);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled_Manual);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    // Trying to change to Manual Ready
    auto res = vsv.triggerVS_IdleToVS_ManualReady();
    vsv.updateState(10);

    ASSERT_EQ(res, STFR::NoFailure);
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::ManualReady);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled_Manual);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    /* Accu not in drive or precharge after 50ms */
    vsv.setAMSState(AMS::AMS_Precharge);
    vsv.updateState(61);

    ASSERT_EQ(res, STFR::NoFailure);
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::ManualReady);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled_Manual);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    // Precharging longer than precharge max
    vsv.updateState(vsv.getConfig()->maxPrechargeTime + 11);

    // Error state
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

    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
    ASSERT_EQ(vsv.getMissionState(), MissionState::MS_NotSelected);
}

TEST(state_transition_06, st6_accu_error_after_precharge)
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

    vsv.changeToManualReady();
    vsv.updateState(5);

    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::ManualReady);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled_Manual);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    vsv.setAMSState(AMS::AMS_Drive);
    vsv.updateState(vsv.getConfig()->normalPrechargeTime + 6); // After normal precharge time

    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::ManualReady);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled_Manual);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    /* Preparing for switch to Error */
    vsv.setAMSState(AMS::AMS_Data_Error);
    vsv.updateState(vsv.getConfig()->normalPrechargeTime + 100);

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
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
    ASSERT_EQ(vsv.getMissionState(), MissionState::MS_NotSelected);
}

TEST(state_transition_06, st6_asms_on)
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

    vsv.changeToManualReady();
    vsv.updateState(5);

    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::ManualReady);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled_Manual);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    /* Preparing for switch to ManualReady */
    Inverter inv;
    inv.is_ready = true;
    inv.rpm = 0;

    vsv.setInverterFLState(inv);
    vsv.setInverterFRState(inv);
    vsv.setInverterRLState(inv);
    vsv.setInverterRRState(inv);

    vsv.setAMSState(AMS::AMS_Drive);
    vsv.updateState(vsv.getConfig()->normalPrechargeTime + 100);

    // State Transition
    vsv.setASMSState(ASMS::ASMS_On);
    vsv.updateState(vsv.getConfig()->normalPrechargeTime + 10000);

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
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
    ASSERT_EQ(vsv.getMissionState(), MissionState::MS_NotSelected);
}
