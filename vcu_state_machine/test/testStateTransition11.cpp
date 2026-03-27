#include "testHelper.cpp"
#include <StateMachineData.hpp>
#include <gtest/gtest.h>
#include <thread>

TEST(state_transition_11, vs_asReady_to_vs_asEmergency)
{
    // Data Setup
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(0);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedBrakePressureRear + 5);
    vsv.setMissionState(MissionState::MS_Selected);
    vsv.setASMSState(ASMS::ASMS_On);
    vsv.setAMSState(AMS::AMS_Drive);
    vsv.setbmcSdcState(true);
    vsv.setSDCReady(true);

    Inverter inv;
    inv.is_ready = true;
    inv.rpm = 0;

    vsv.setInverterFLState(inv);
    vsv.setInverterFRState(inv);
    vsv.setInverterRLState(inv);
    vsv.setInverterRRState(inv);

    vsv.setEBSState(EBS::Armed_Parking);

    vsv.changeToAsReady();

    vsv.updateState(5);

    // Pretests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsReady);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed_Parking);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    // Start with the state transition
    vsv.setEBSPressureFront(5);

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

TEST(state_transition_11, st_11_AMS_Error)
{
    // Data Setup
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(0);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedBrakePressureRear + 5);
    vsv.setMissionState(MissionState::MS_Selected);
    vsv.setASMSState(ASMS::ASMS_On);
    vsv.setAMSState(AMS::AMS_Drive);
    vsv.setbmcSdcState(true);
    vsv.setSDCReady(true);

    Inverter inv;
    inv.is_ready = true;
    inv.rpm = 0;

    vsv.setInverterFLState(inv);
    vsv.setInverterFRState(inv);
    vsv.setInverterRLState(inv);
    vsv.setInverterRRState(inv);

    vsv.setEBSState(EBS::Armed_Parking);

    vsv.changeToAsReady();

    vsv.updateState(5);

    // Pretests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsReady);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed_Parking);
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

TEST(state_transition_11, st_11_errors_exist)
{
    // Data Setup
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(0);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedBrakePressureRear + 5);
    vsv.setMissionState(MissionState::MS_Selected);
    vsv.setASMSState(ASMS::ASMS_On);
    vsv.setAMSState(AMS::AMS_Drive);
    vsv.setbmcSdcState(true);
    vsv.setSDCReady(true);

    Inverter inv;
    inv.is_ready = true;
    inv.rpm = 0;

    vsv.setInverterFLState(inv);
    vsv.setInverterFRState(inv);
    vsv.setInverterRLState(inv);
    vsv.setInverterRRState(inv);

    vsv.setEBSState(EBS::Armed_Parking);

    vsv.changeToAsReady();

    vsv.updateState(5);

    // Pretests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsReady);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed_Parking);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    // Start with the state transition
    vsv.addError(Error::E_AMSTimeout);
    vsv.updateState(20);

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

TEST(state_transition_11, st_11_asms_off)
{
    // Data Setup
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(0);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedBrakePressureRear + 5);
    vsv.setMissionState(MissionState::MS_Selected);
    vsv.setASMSState(ASMS::ASMS_On);
    vsv.setAMSState(AMS::AMS_Drive);
    vsv.setbmcSdcState(true);
    vsv.setSDCReady(true);

    Inverter inv;
    inv.is_ready = true;
    inv.rpm = 0;

    vsv.setInverterFLState(inv);
    vsv.setInverterFRState(inv);
    vsv.setInverterRLState(inv);
    vsv.setInverterRRState(inv);

    vsv.setEBSState(EBS::Armed_Parking);

    vsv.changeToAsReady();

    vsv.updateState(5);

    // Pretests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsReady);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed_Parking);
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

TEST(state_transition_11, st11_precharging_to_long)
{

    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(0);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedBrakePressureRear + 5);
    vsv.setMissionState(MissionState::MS_Selected);
    vsv.setASMSState(ASMS::ASMS_On);
    vsv.setAMSState(AMS::AMS_Drive);
    vsv.setbmcSdcState(true);
    vsv.setSDCReady(true);

    Inverter inv;
    inv.is_ready = true;
    inv.rpm = 0;

    vsv.setInverterFLState(inv);
    vsv.setInverterFRState(inv);
    vsv.setInverterRLState(inv);
    vsv.setInverterRRState(inv);

    vsv.setEBSState(EBS::Armed_Parking);

    vsv.changeToAsReady();

    vsv.updateState(5);

    // Pretests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsReady);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed_Parking);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    /* Accu not in drive or precharge after 50ms */
    vsv.setAMSState(AMS::AMS_Precharge);
    vsv.updateState(61);

    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsReady);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed_Parking);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    // Precharging longer than precharge max
    std::cout << "TEST Timeupdate: " << vsv.getConfig()->maxPrechargeTime + 11 << std::endl;
    vsv.updateState(vsv.getConfig()->maxPrechargeTime + 11);

    // Emergency state
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

TEST(state_transition_11, st_11_open_sdc)
{
    // Data Setup
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(0);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedBrakePressureRear + 5);
    vsv.setMissionState(MissionState::MS_Selected);
    vsv.setASMSState(ASMS::ASMS_On);
    vsv.setAMSState(AMS::AMS_Drive);
    vsv.setbmcSdcState(true);
    vsv.setSDCReady(true);

    Inverter inv;
    inv.is_ready = true;
    inv.rpm = 0;

    vsv.setInverterFLState(inv);
    vsv.setInverterFRState(inv);
    vsv.setInverterRLState(inv);
    vsv.setInverterRRState(inv);

    vsv.setEBSState(EBS::Armed_Parking);

    vsv.changeToAsReady();

    vsv.updateState(5);

    // Pretests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsReady);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed_Parking);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    // Start with the state transition
    vsv.setbmcSdcState(false);
    vsv.updateState(vsv.getConfig()->maxPrechargeTime + 100);

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
