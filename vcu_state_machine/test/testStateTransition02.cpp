#include "testHelper.cpp"
#include <StateMachineData.hpp>
#include <gtest/gtest.h>
#include <thread>

// ############################################################################
// Testing State Transitions and States
// ############################################################################

// ----------- Test state transition ASOff to ASReady -------------------------
// Test working state transitions without errors
TEST(state_transition_2, asOff_to_asReady)
{
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    // Updating vs and vsm according to the state vector
    vsv.changeToAsOff();

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);

    vsv.updateState(5);

    /* --- Test Preconditions --- */
    // PreTest Vehicle State
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsOff);
    // -- PreTest in State AS_Off --
    // SDC is activated
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    // SDC Relais is open because no mission is set
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    // EBS Valves are deactivated constant braking
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    // EBS is in AS Off disabled
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled);
    // TS Off
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    // No errors
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    /* --- Preconditions --- */
    // Mission Selected
    vsv.setMissionState(MissionState::MS_Selected);
    // ASMS on
    vsv.setASMSState(ASMS::ASMS_On);

    // Doing stuff for a failed ebs startup
    std::this_thread::sleep_for(std::chrono::duration(std::chrono::milliseconds(1000)));
    vsv.setEBSState(EBS::Armed_Parking);
    vsv.checkEBS();

    // SDC closed
    vsv.setSDCReady(true);
    vsv.setbmcSdcState(true);
    // No errors (later with some errors)
    vsv.resetErrors();

    /* --- State Change Triger --- */
    // Press external TS Button
    int8_t res = vsv.triggerVS_AsOffToVS_AsReady();

    // When we successfully switched to state AS Ready (Car side) the following things will change (in this order)
    // AMS State changes from AMS Idle to AMS Drive
    vsv.setAMSState(AMS::AMS_Drive);

    // Eventuelly have to trigger updateState() (is done automatically (every 5ms??))
    vsv.updateState(10);

    /* --- Test Postconditions --- */
    ASSERT_EQ(res, 0);
    // Post test Vehicle State
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsReady);
    // -- Post tests in Vehicle State AS Ready --
    // SDC closed
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    // SDC Relais is open because no mission is set
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    // EBS Valves programmed for parking brake
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    // Startup sequence not started so its state 0
    // ASSERT_EQ(vsv.getVehicleStateMsg().ebs_startup_sequence, 7);
    // EBS is in AS Off disabled
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed_Parking);
    // TS activated
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, true);
    // No errors
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
}

// Try to trigger state transition 2 with broken SDC
TEST(state_transition_2, st2_failed_sdc_on)
{
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    // Updating vs and vsm according to the state vector
    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(vsv.getConfig()->minPressurizedBrakePressureFront + 5);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedBrakePressureRear + 5);

    vsv.changeToAsOff();

    /* --- Test Preconditions --- */
    // PreTest Vehicle State
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsOff);
    // -- PreTest in State AS_Off --
    // SDC is activated
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    // SDC Relay is closed
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    // EBS Valves are deactivated for constant braking
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    // EBS is in AS Off disabled
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled);
    // TS Off
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    // No errors
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    /* --- Preconditions --- */
    // Mission Selected
    vsv.setMissionState(MissionState::MS_Selected);
    // ASMS on
    vsv.setASMSState(ASMS::ASMS_On);

    // Doing stuff for a failed ebs startup
    std::this_thread::sleep_for(std::chrono::duration(std::chrono::milliseconds(1000)));
    vsv.setEBSState(EBS::Armed_Parking);
    vsv.checkEBS();

    // SDC closed
    vsv.setSDCReady(true);
    vsv.setbmcSdcState(false);
    // No errors (later with some errors)
    vsv.resetErrors();

    /* --- State Change Triger --- */
    // Press external TS Button
    int8_t res = vsv.triggerVS_AsOffToVS_AsReady();

    // When we successfully switched to state AS Ready (Car side) the following things will change (in this order)
    // AMS State changes from AMS Idle to AMS Drive
    vsv.setAMSState(AMS::AMS_Idle);

    // Eventuelly have to trigger updateState() (is done automatically (every 5ms??))
    vsv.updateState(5);

    /* --- Test Postconditions --- */
    ASSERT_EQ(res, 1);
    // Post test Vehicle State
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsOff);
    // -- Post tests in Vehicle State AS Ready --
    // SDC closed
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    // SDC Relay is closed
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);

    // EBS is in AS Off disabled
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed_Parking);

    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    // TS activated
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    // No errors
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
}

// Mission not set so SDC Relay not closed
TEST(state_transition_2, st2_failed_no_mission)
{
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    // Updating vs and vsm according to the state vector
    vsv.setBrakePressureFront(vsv.getConfig()->minPressurizedBrakePressureFront + 5);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedBrakePressureRear + 5);

    vsv.changeToAsOff();

    /* --- Test Preconditions --- */
    // PreTest Vehicle State
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsOff);
    // -- PreTest in State AS_Off --
    // SDC is activated
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    // SDC Relais is open because no mission is set
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    // EBS Valves are deactivated constant braking
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    // EBS is in AS Off disabled
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled);
    // TS Off
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    // No errors
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    /* --- Preconditions --- */
    // Mission Selected
    vsv.setMissionState(MissionState::MS_NotSelected);
    // ASMS on
    vsv.setASMSState(ASMS::ASMS_On);

    // Doing stuff for a failed ebs startup
    std::this_thread::sleep_for(std::chrono::duration(std::chrono::milliseconds(1000)));
    vsv.setEBSState(EBS::Armed_Parking);
    vsv.checkEBS();

    vsv.updateState(5);

    // SDC Relais is open because no mission is set
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
}

// Mission set but no ASMS
TEST(state_transition_2, st2_failed_no_asms)
{
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    // Updating vs and vsm according to the state vector
    vsv.setBrakePressureFront(vsv.getConfig()->minPressurizedBrakePressureFront + 5);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedBrakePressureRear + 5);

    vsv.changeToAsOff();

    /* --- Test Preconditions --- */
    // PreTest Vehicle State
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsOff);
    // -- PreTest in State AS_Off --
    // SDC is activated
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    // SDC Relais is open because no mission is set
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    // EBS Valves are deactivated constant braking
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    // EBS is in AS Off disabled
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled);
    // TS Off
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    // No errors
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    /* --- Preconditions --- */
    // Mission Selected
    vsv.setMissionState(MissionState::MS_Selected);
    // ASMS on
    vsv.setASMSState(ASMS::ASMS_Off);

    vsv.updateState(5);

    // SDC Relais is open because no mission is set
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
}

// EBS Startup Sequence fails

TEST(state_transition_2, st2_failed_ebs_startup_failure)
{
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());
    vsv.getEBS()->setESUSError();

    // Updating vs and vsm according to the state vector
    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(vsv.getConfig()->minPressurizedBrakePressureFront + 5);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedBrakePressureRear + 5);

    vsv.changeToAsOff();

    /* --- Test Preconditions --- */
    // PreTest Vehicle State
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsOff);
    // -- PreTest in State AS_Off --
    // SDC is activated
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    // SDC Relais is open because no mission is set
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    // EBS Valves are deactivated constant braking
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    // EBS is in AS Off disabled
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled);
    // TS Off
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    // No errors
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    vsv.updateState(5);

    /* --- Preconditions --- */
    // Mission Selected
    vsv.setMissionState(MissionState::MS_Selected);
    // ASMS on
    vsv.setASMSState(ASMS::ASMS_On);

    // Doing stuff for a failed ebs startup
    std::this_thread::sleep_for(std::chrono::duration(std::chrono::milliseconds(7500)));

    // SDC closed
    vsv.setSDCReady(true);
    vsv.setbmcSdcState(true);
    // No errors (later with some errors)
    vsv.resetErrors();

    /* --- State Change Triger --- */
    // Press external TS Button
    int8_t res = vsv.triggerVS_AsOffToVS_AsReady();

    vsv.setAMSState(AMS::AMS_Idle);
    vsv.updateState(60010);

    // Eventuelly have to trigger updateState() (is done automatically (every 5ms??))
    vsv.updateState(60015);

    /* --- Test Postconditions --- */
    ASSERT_EQ(res, 4);
    // Post test Vehicle State
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsOff);
    // -- Post tests in Vehicle State AS Ready --
    // SDC closed
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    // SDC Relay is closed
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);
    // EBS Valves not programmed for parking brake
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    // EBS is in AS Off disabled
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Startup); // +++ Todo: Not the right state
    // TS activated
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    // No errors
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);
}

TEST(state_transition_2, losing_brake_pressure_after_st)
{
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureFront(vsv.getConfig()->minPressurizedBrakePressureFront + 5);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedBrakePressureRear + 5);
    // Updating vs and vsm according to the state vector
    vsv.changeToAsOff();

    /* --- Test Preconditions --- */
    // PreTest Vehicle State
    ASSERT_EQ(vsv.getVehicleState(), VEHICLE_STATE::AsOff);
    // -- PreTest in State AS_Off --
    // SDC is activated
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_activated, true);
    // SDC Relais is open because no mission is set
    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
    // EBS Valves are deactivated constant braking
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);
    // EBS is in AS Off disabled
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Disabled);
    // TS Off
    ASSERT_EQ(vsv.getVehicleStateMsg().ts_activated, false);
    // No errors
    list<Error> s_errors = { Error::E_None };
    ASSERT_EQ(vsv.getVehicleStateMsg().errors, s_errors);

    /* --- Preconditions --- */
    // Mission Selected
    vsv.setMissionState(MissionState::MS_Selected);
    // ASMS on
    vsv.setASMSState(ASMS::ASMS_On);

    // Doing stuff for a failed ebs startup
    std::this_thread::sleep_for(std::chrono::duration(std::chrono::milliseconds(1000)));
    vsv.setEBSState(EBS::Armed_Parking);

    // No errors (later with some errors)
    vsv.resetErrors();

    vsv.updateState(5);

    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, true);

    vsv.setEBSPressureFront(vsv.getConfig()->minPressurizedEBSPressureFront + 5);
    vsv.setEBSPressureRear(vsv.getConfig()->minPressurizedEBSPressureRear - 5);

    vsv.updateState(10);

    ASSERT_EQ(vsv.getVehicleStateMsg().state, VEHICLE_STATE::AsEmergency);

    ASSERT_EQ(vsv.getVehicleStateMsg().sdc_relay_closed, false);
}
