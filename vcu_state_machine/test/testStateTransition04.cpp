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

TEST(state_transition_04, vs_manualReady_to_idle_trough_sdc_open)
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
    vsv.setbmcSdcState(false);
    vsv.updateState(vsv.getConfig()->normalPrechargeTime + 10);

    // Normal Idle State:
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::Idle);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled_Manual);

    // Test the AIRSchoner
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    int timeToWait = vsv.getConfig()->normalPrechargeTime + TS_DEACTIVATION_DELAY + 10;
    vsv.setbmcSdcState(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(TS_DEACTIVATION_DELAY + 5));
    vsv.updateState(vsv.getConfig()->normalPrechargeTime + 10 + timeToWait);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);

    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
}

TEST(state_transition_04, st04_precharge_not_started)
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
    vsv.setAMSState(AMS::AMS_Idle);
    vsv.updateState(vsv.getConfig()->normalPrechargeTime * 1000 + 6);

    // Normal Idle State:
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::Idle);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
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
}
