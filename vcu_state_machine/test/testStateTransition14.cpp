#include "testHelper.cpp"
#include <StateMachineData.hpp>
#include <gtest/gtest.h>

TEST(state_transition_14, vs_asFinished_to_vs_asEmergency_through_EBS_complete_pressure_loss)
{
    /// Data Setup
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.changeToAsFinished();

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(vsv.getConfig()->minPressurizedBrakePressureFront + 5);
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
    vsv.updateState(5);

    // Pretests
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsFinished);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed);
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
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
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
}
