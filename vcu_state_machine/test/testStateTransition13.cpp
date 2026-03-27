#include "testHelper.cpp"
#include <StateMachineData.hpp>
#include <gtest/gtest.h>
#include <thread>

// AS Drive State:
//         - SDC on
//         - SDC relay on
//         - TS activated
//         - ebs valve 1 activated
//         - ebs valve 2 activated
//         _ ebs state armed
//         _ ebs start up sequence 7?
//         _ no errors

//         - SDC ready = true
//         - SDC End = true
//         - ASMS on
//         - ebs pressure 1 = > x (5)
//         - ebs pressure 2 = > x (5)
//         - brake pressure front = < 0.5
//         - brake pressure rear = < 0.5
//         - missionSelected = true
//         - inverters = dnc | ready
//         - amsState = Drive
//         - IC Voltage = dnc

// AS Emergency
//         - SDC off
//         - SDC relay off
//         - TS deactivated
//         - ebs valve 1 deactivated
//         - ebs valve 2 deactivated
//         _ ebs state activated
//         _ ebs start up sequence 7
//         _ no errors

//         - missionSelected = dnc
//         - SDC ready = false
//         - SDC End = false
//         - ASMS on
//         - brake pressure front = ebs 1
//         - brake pressure rear = ebs 2
//         - ebs pressure 1 = > x bar (5)
//         - ebs pressure 2 = > x bar (5)
//         - inverters = 0 and ready
//         - amsState = Idle
//         - IC Voltage = 0

TEST(state_transition_13, vs_asDrive_to_vs_asEmergency_through_RES)
{
    // Data Setup
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(0);
    vsv.setBrakePressureRear(0);
    vsv.setMissionState(MissionState::MS_Selected);
    vsv.setASMSState(ASMS::ASMS_On);
    vsv.setAMSState(AMS::AMS_Drive);
    vsv.setbmcSdcState(true);
    vsv.setSDCReady(true);

    vsv.changeToAsDrive();

    Inverter inv;
    inv.is_ready = true;
    inv.rpm = 10000;

    vsv.setInverterFLState(inv);
    vsv.setInverterFRState(inv);
    vsv.setInverterRLState(inv);
    vsv.setInverterRRState(inv);

    vsv.getEBS()->setEBSState(EBS::Armed);
    vsv.updateState(5);

    // Pretests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsDrive);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    // Start with the state transition
    vsv.triggerVS_AsDriveToVS_AsEmergency();

    // Posttests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsEmergency);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Activated);

    // Test the AIRSchoner
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    int timeToWait = vsv.getConfig()->normalPrechargeTime + TS_DEACTIVATION_DELAY + 10;
    vsv.setbmcSdcState(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(TS_DEACTIVATION_DELAY + 5));
    vsv.updateState(timeToWait);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);

    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
}

TEST(state_transition_13, vs_asDrive_to_vs_asEmergency_through_EBS_pressure_loss)
{
    // Data Setup
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(0);
    vsv.setBrakePressureRear(0);
    vsv.setMissionState(MissionState::MS_Selected);
    vsv.setASMSState(ASMS::ASMS_On);
    vsv.setAMSState(AMS::AMS_Drive);
    vsv.setbmcSdcState(true);
    vsv.setSDCReady(true);

    vsv.changeToAsDrive();

    Inverter inv;
    inv.is_ready = true;
    inv.rpm = 10000;

    vsv.setInverterFLState(inv);
    vsv.setInverterFRState(inv);
    vsv.setInverterRLState(inv);
    vsv.setInverterRRState(inv);

    vsv.getEBS()->setEBSState(EBS::Armed);

    vsv.updateState(10);

    // Pretests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsDrive);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    // Start with the state transition
    vsv.setEBSPressureFront(5);

    vsv.updateState(15);

    // Posttests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsEmergency);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Activated);

    // Test the AIRSchoner
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    int timeToWait = vsv.getConfig()->normalPrechargeTime + TS_DEACTIVATION_DELAY + 10;
    vsv.setbmcSdcState(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(TS_DEACTIVATION_DELAY + 5));
    vsv.updateState(timeToWait);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);

    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
}

TEST(state_transition_13, vs_asDrive_to_vs_asEmergency_through_EBS_complete_pressure_loss)
{
    // Data Setup
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(0);
    vsv.setBrakePressureRear(0);
    vsv.setMissionState(MissionState::MS_Selected);
    vsv.setASMSState(ASMS::ASMS_On);
    vsv.setAMSState(AMS::AMS_Drive);
    vsv.setbmcSdcState(true);
    vsv.setSDCReady(true);

    vsv.changeToAsDrive();

    Inverter inv;
    inv.is_ready = true;
    inv.rpm = 10000;

    vsv.setInverterFLState(inv);
    vsv.setInverterFRState(inv);
    vsv.setInverterRLState(inv);
    vsv.setInverterRRState(inv);

    vsv.getEBS()->setEBSState(EBS::Armed);

    vsv.updateState(5);

    // Pretests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsDrive);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    // Start with the state transition
    vsv.setEBSPressureFront(5);
    vsv.setEBSPressureFront(4.5);

    vsv.updateState(10);

    // Posttests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsEmergency);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Activated);

    // Test the AIRSchoner
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    int timeToWait = vsv.getConfig()->normalPrechargeTime + TS_DEACTIVATION_DELAY + 10;
    vsv.setbmcSdcState(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(TS_DEACTIVATION_DELAY + 5));
    vsv.updateState(timeToWait);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);

    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
}

TEST(state_transition_13, st_13_ams_can_timeout)
{
    // Data Setup
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(0);
    vsv.setBrakePressureRear(0);
    vsv.setMissionState(MissionState::MS_Selected);
    vsv.setASMSState(ASMS::ASMS_On);
    vsv.setAMSState(AMS::AMS_Drive);
    vsv.setbmcSdcState(true);
    vsv.setSDCReady(true);

    vsv.changeToAsDrive();

    Inverter inv;
    inv.is_ready = true;
    inv.rpm = 10000;

    vsv.setInverterFLState(inv);
    vsv.setInverterFRState(inv);
    vsv.setInverterRLState(inv);
    vsv.setInverterRRState(inv);

    vsv.getEBS()->setEBSState(EBS::Armed);

    vsv.updateState(5);

    // Pretests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsDrive);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    // Start with the state transition
    vsv.addError(Error::E_AMSTimeout);
    vsv.updateState(10);

    // Posttests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsEmergency);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Activated);

    // Test the AIRSchoner
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    int timeToWait = vsv.getConfig()->normalPrechargeTime + TS_DEACTIVATION_DELAY + 10;
    vsv.setbmcSdcState(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(TS_DEACTIVATION_DELAY + 5));
    vsv.updateState(timeToWait);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);

    list<Error> p_errors = { Error::E_AMSTimeout };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, p_errors);
}

TEST(state_transition_13, st_13_ams_not_driving)
{
    // Data Setup
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(0);
    vsv.setBrakePressureRear(0);
    vsv.setMissionState(MissionState::MS_Selected);
    vsv.setASMSState(ASMS::ASMS_On);
    vsv.setAMSState(AMS::AMS_Drive);
    vsv.setbmcSdcState(true);
    vsv.setSDCReady(true);

    vsv.changeToAsDrive();

    Inverter inv;
    inv.is_ready = true;
    inv.rpm = 10000;

    vsv.setInverterFLState(inv);
    vsv.setInverterFRState(inv);
    vsv.setInverterRLState(inv);
    vsv.setInverterRRState(inv);

    vsv.getEBS()->setEBSState(EBS::Armed);

    vsv.updateState(5);

    // Pretests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsDrive);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    // Start with the state transition
    vsv.setAMSState(AMS::AMS_Data_Error);
    vsv.updateState(10);

    // Posttests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsEmergency);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Activated);

    // Test the AIRSchoner
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    int timeToWait = vsv.getConfig()->normalPrechargeTime + TS_DEACTIVATION_DELAY + 10;
    vsv.setbmcSdcState(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(TS_DEACTIVATION_DELAY + 5));
    vsv.updateState(timeToWait);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);

    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
}

TEST(state_transition_13, st_13_asms_off)
{
    // Data Setup
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(0);
    vsv.setBrakePressureRear(0);
    vsv.setMissionState(MissionState::MS_Selected);
    vsv.setASMSState(ASMS::ASMS_On);
    vsv.setAMSState(AMS::AMS_Drive);
    vsv.setbmcSdcState(true);
    vsv.setSDCReady(true);

    vsv.changeToAsDrive();

    Inverter inv;
    inv.is_ready = true;
    inv.rpm = 10000;

    vsv.setInverterFLState(inv);
    vsv.setInverterFRState(inv);
    vsv.setInverterRLState(inv);
    vsv.setInverterRRState(inv);

    vsv.getEBS()->setEBSState(EBS::Armed);

    vsv.updateState(5);

    // Pretests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsDrive);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    // Start with the state transition
    vsv.setASMSState(ASMS::ASMS_Off);
    vsv.updateState(10);

    // Posttests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsEmergency);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Activated);

    // Test the AIRSchoner
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    int timeToWait = vsv.getConfig()->normalPrechargeTime + TS_DEACTIVATION_DELAY + 10;
    vsv.setbmcSdcState(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(TS_DEACTIVATION_DELAY + 5));
    vsv.updateState(timeToWait);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);

    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
}

TEST(state_transition_13, st_13_sdc_off)
{
    // Data Setup
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(0);
    vsv.setBrakePressureRear(0);
    vsv.setMissionState(MissionState::MS_Selected);
    vsv.setASMSState(ASMS::ASMS_On);
    vsv.setAMSState(AMS::AMS_Drive);
    vsv.setbmcSdcState(true);
    vsv.setSDCReady(true);

    vsv.changeToAsDrive();

    Inverter inv;
    inv.is_ready = true;
    inv.rpm = 10000;

    vsv.setInverterFLState(inv);
    vsv.setInverterFRState(inv);
    vsv.setInverterRLState(inv);
    vsv.setInverterRRState(inv);

    vsv.getEBS()->setEBSState(EBS::Armed);

    vsv.updateState(5);

    // Pretests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsDrive);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    // Start with the state transition
    vsv.setbmcSdcState(false);
    vsv.updateState(10);

    // Posttests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsEmergency);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Activated);

    // Test the AIRSchoner
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    int timeToWait = vsv.getConfig()->normalPrechargeTime + TS_DEACTIVATION_DELAY + 10;
    vsv.setbmcSdcState(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(TS_DEACTIVATION_DELAY + 5));
    vsv.updateState(timeToWait);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);

    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
}

TEST(state_transition_13, st_13_dv_timeout)
{
    // Data Setup
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(0);
    vsv.setBrakePressureRear(0);
    vsv.setMissionState(MissionState::MS_Selected);
    vsv.setASMSState(ASMS::ASMS_On);
    vsv.setAMSState(AMS::AMS_Drive);
    vsv.setbmcSdcState(true);
    vsv.setSDCReady(true);

    vsv.changeToAsDrive();

    Inverter inv;
    inv.is_ready = true;
    inv.rpm = 10000;

    vsv.setInverterFLState(inv);
    vsv.setInverterFRState(inv);
    vsv.setInverterRLState(inv);
    vsv.setInverterRRState(inv);

    vsv.getEBS()->setEBSState(EBS::Armed);

    vsv.updateState(5);

    // Pretests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsDrive);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    // Start with the state transition
    vsv.addError(Error::E_DVTimeout);
    vsv.updateState(10);

    // Posttests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsEmergency);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Activated);

    // Test the AIRSchoner
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    int timeToWait = vsv.getConfig()->normalPrechargeTime + TS_DEACTIVATION_DELAY + 10;
    vsv.setbmcSdcState(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(TS_DEACTIVATION_DELAY + 5));
    vsv.updateState(timeToWait);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);

    list<Error> p_errors = { Error::E_DVTimeout };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, p_errors);
}
