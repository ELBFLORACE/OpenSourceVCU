#include "testHelper.cpp"
#include <StateMachineData.hpp>
#include <gtest/gtest.h>

// AS Ready
//         - SDC on
//         - SDC relay on
//         - TS activated
//         - ebs valve 1 activated
//         - ebs valve 2 deactivated (parking brake)
//         _ ebs state armed
//         _ ebs start up sequence 7?
//         _ no errors

//         - missionSelected = true
//         - SDC ready = true
//         - SDC End = true
//         - ASMS on
//         - brake pressure front = 0
//         - brake pressure rear = ebs pressure 2
//         - ebs pressure 1 = > x bar (5)
//         - ebs pressure 2 = > x bar (5)
//         - inverters = 0 and ready for state transition
//         - amsState = Drive
//         - IC Voltage = dnc

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

TEST(state_transition_10, vs_asReady_to_vs_asDrive)
{
    // Data Setup
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.changeToAsReady();

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
    vsv.updateState(5005);
    auto res = vsv.triggerVS_AsReadyToVS_AsDrive();
    // Posttests
    ASSERT_EQ(res, STFR::NoFailure);
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsDrive);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
}

TEST(state_transition_10, st_10_failed_less_than_5_sec_waited)
{
    // Data Setup
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.changeToAsReady();

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
    auto res = vsv.triggerVS_AsReadyToVS_AsDrive();

    // Posttests
    ASSERT_EQ(res, STFR::NotWaitedForASDrive);
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsReady);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed_Parking);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
}
