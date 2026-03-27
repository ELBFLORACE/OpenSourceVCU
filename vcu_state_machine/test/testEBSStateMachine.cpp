#include "testHelper.cpp"
#include <StateMachineData.hpp>
#include <gtest/gtest.h>

#include <thread>

TEST(ebs_state_machine, check_init)
{
    EBSStateMachine esm = EBSStateMachine();

    ASSERT_EQ(esm.getEBSState(), EBS::Disabled_Manual);
    ASSERT_EQ(esm.getESUSP(), ESUS::WaitForASSM);
    ASSERT_EQ(esm.getBrakePressureFront(), 0.0);
    ASSERT_EQ(esm.getBrakePressureRear(), 0.0);
    ASSERT_EQ(esm.getEBSPressureFront(), 0.0);
    ASSERT_EQ(esm.getEBSPressureRear(), 0.0);
}

TEST(ebs_state_machine, check_startup_deactivate_watchdog)
{
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setSDCReady(true);

    vsv.changeToAsOff();

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);

    vsv.updateState(1);
    vsv.updateState(2);

    // Testing that the watchdog alternates
    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.updateState(5);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, true);

    vsv.updateState(10);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    EBSStateMachine* esm = vsv.getEBS();

    esm->startESUS(1);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(15);

    // check that the watchdog is disabled
    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.updateState(20);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.updateState(WAIT_FOR_WATCHDOG + 100);

    // Do not simulate that the ASSM disables SDCReady because of the switched off watchdog
    // --> Failure in step ESUS::CheckWatchdog
    ASSERT_EQ(esm->getESUSP(), ESUS::CheckWatchdog);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Startup);
}

TEST(ebs_state_machine, check_not_all_pressurized)
{
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setSDCReady(true);

    vsv.changeToAsOff();

    vsv.setSDCReady(true);

    // EBS is pressurized, EBS valves off,
    // Front brake pressurized (as should), Rear brake not pressurized (should fail)
    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setBrakePressureFront(vsv.getConfig()->minPressurizedEBSPressureFront + 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedBrakePressureRear - 1);

    vsv.updateState(1);
    vsv.updateState(2);

    // Testing that the watchdog alternates
    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.updateState(5);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, true);

    vsv.updateState(10);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    EBSStateMachine* esm = vsv.getEBS();

    esm->startESUS(2);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    vsv.updateState(15);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.updateState(20);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    // Simulate the ASSM watchdog
    vsv.setSDCReady(false);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(30);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, true);

    ASSERT_EQ(esm->getESUSP(), ESUS::CheckAllPressure);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Startup_Error);
}

TEST(ebs_state_machine, check_all_pressurized_not_releasing_front)
{
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setSDCReady(true);

    vsv.changeToAsOff();

    vsv.setSDCReady(true);

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setBrakePressureFront(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedEBSPressureRear + 1);

    vsv.updateState(1);
    vsv.updateState(2);

    // Testing that the watchdog alternates
    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.updateState(5);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, true);

    vsv.updateState(10);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    EBSStateMachine* esm = vsv.getEBS();

    esm->startESUS(3);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    vsv.updateState(15);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.updateState(20);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    // Simulate the ASSM watchdog
    vsv.setSDCReady(false);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(30);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, true);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(40);

    // EBS valve front should be activate rear not
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);

    std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_BOTH_BRAKE_RELEASE));
    vsv.updateState(40 + WAIT_FOR_BOTH_BRAKE_RELEASE);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    vsv.updateState(345);
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));

    ASSERT_EQ(esm->getESUSP(), ESUS::CheckFrontBrakeRelease);
    std::cout << "[DEBUG] Asking for ebs state" << std::endl;
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Startup_Error);
}

TEST(ebs_state_machine, check_all_releasing_front_not_braking)
{
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setSDCReady(true);

    vsv.changeToAsOff();

    vsv.setSDCReady(true);

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setBrakePressureFront(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedEBSPressureRear + 1);

    vsv.updateState(1);
    vsv.updateState(2);

    // Testing that the watchdog alternates
    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.updateState(5);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, true);

    vsv.updateState(10);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    EBSStateMachine* esm = vsv.getEBS();

    esm->startESUS(4);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    vsv.updateState(15);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.updateState(20);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    // Simulate the ASSM watchdog
    vsv.setSDCReady(false);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(30);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, true);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(40);

    // EBS valve front should be activate rear not
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Simulate brake and ebs pressure behavior
    vsv.setAllPressures(vsv.getConfig()->maxEBSPressureFront - 5, vsv.getConfig()->maxEBSPressureRear - 5,
        vsv.getConfig()->maxReleasedBrakePressureFront - 0.2, vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.updateState(140);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(150);

    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    vsv.updateState(450);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(410);

    // Front brake does not get pressurized again after release --> Failure
    ASSERT_EQ(esm->getESUSP(), ESUS::CheckFrontBrakeActivation);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Startup_Error);
}

TEST(ebs_state_machine, check_front_solo_braking_and_not_release_rear)
{
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setSDCReady(true);

    vsv.changeToAsOff();

    vsv.setSDCReady(true);

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setBrakePressureFront(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedEBSPressureRear + 1);

    vsv.updateState(1);
    vsv.updateState(2);

    // Testing that the watchdog alternates
    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.updateState(5);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, true);

    vsv.updateState(10);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    EBSStateMachine* esm = vsv.getEBS();

    esm->startESUS(5);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    vsv.updateState(15);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.updateState(20);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    // Simulate the ASSM watchdog
    vsv.setSDCReady(false);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(30);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, true);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(40);

    // EBS valve front should be activate rear not
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    vsv.setAllPressures(vsv.getConfig()->maxEBSPressureFront - 5, vsv.getConfig()->maxEBSPressureRear - 5,
        vsv.getConfig()->maxReleasedBrakePressureFront - 0.2, vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.updateState(140);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(150);

    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    vsv.setAllPressures(vsv.getConfig()->maxEBSPressureFront - 5, vsv.getConfig()->maxEBSPressureRear - 5,
        vsv.getConfig()->minPressurizedBrakePressureFront + 5, vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.updateState(250);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(260);

    // Activate EBS valve rear -> Release rear brake (Step6)
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);

    std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_FOR_BOTH_BRAKE_RELEASE));
    vsv.setAllPressures(vsv.getConfig()->maxEBSPressureFront - 5, vsv.getConfig()->maxEBSPressureRear - 5,
        vsv.getConfig()->minPressurizedBrakePressureFront + 5, vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.updateState(260 + WAIT_FOR_BOTH_BRAKE_RELEASE);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    vsv.updateState(565);

    // Rear brake does not get released --> Failure
    ASSERT_EQ(esm->getESUSP(), ESUS::CheckRearBrakeRelease);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Startup_Error);
}

TEST(ebs_state_machine, check_front_braking_rear_not_braking_after_release)
{
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setSDCReady(true);

    vsv.changeToAsOff();

    vsv.setSDCReady(true);

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setBrakePressureFront(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedEBSPressureRear + 1);

    vsv.updateState(1);
    vsv.updateState(2);

    // Testing that the watchdog alternates
    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.updateState(5);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, true);

    vsv.updateState(10);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    EBSStateMachine* esm = vsv.getEBS();

    esm->startESUS(5);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    vsv.updateState(15);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.updateState(20);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    // Simulate the ASSM watchdog
    vsv.setSDCReady(false);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(30);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, true);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(40);

    // EBS valve front should be activate rear not
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    vsv.setAllPressures(vsv.getConfig()->maxEBSPressureFront - 5, vsv.getConfig()->maxEBSPressureRear - 5,
        vsv.getConfig()->maxReleasedBrakePressureFront - 0.2, vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.updateState(140);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(150);

    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    vsv.setAllPressures(vsv.getConfig()->maxEBSPressureFront - 5, vsv.getConfig()->maxEBSPressureRear - 5,
        vsv.getConfig()->minPressurizedBrakePressureFront + 5, vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.updateState(250);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(260);

    // Activate EBS valve rear -> Release rear brake (Step6)
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);

    // Both released in time
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    vsv.setAllPressures(vsv.getConfig()->maxEBSPressureFront - 5, vsv.getConfig()->maxEBSPressureRear - 5,
        vsv.getConfig()->minPressurizedBrakePressureFront + 5, vsv.getConfig()->maxReleasedBrakePressureRear - 0.2);
    vsv.updateState(360);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(370);

    // Both brakes activated again (Step 7)
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    vsv.setAllPressures(vsv.getConfig()->maxEBSPressureFront - 5, vsv.getConfig()->maxEBSPressureRear - 5,
        vsv.getConfig()->minPressurizedBrakePressureFront + 5, vsv.getConfig()->maxReleasedBrakePressureRear - 0.2);
    vsv.updateState(670);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    vsv.updateState(675);

    // Rear not braking in time --> Failure
    ASSERT_EQ(esm->getESUSP(), ESUS::CheckRearBrakeActivation);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Startup_Error);
}

TEST(ebs_state_machine, check_rear_activated_esus_ok)
{
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setSDCReady(true);

    vsv.changeToAsOff();

    vsv.setSDCReady(true);

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setBrakePressureFront(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedEBSPressureRear + 1);

    vsv.updateState(1);
    vsv.updateState(2);

    // Testing that the watchdog alternates
    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.updateState(5);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, true);

    vsv.updateState(10);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    EBSStateMachine* esm = vsv.getEBS();

    esm->startESUS(5);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    vsv.updateState(15);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.updateState(20);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    // Simulate the ASSM watchdog
    vsv.setSDCReady(false);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(30);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, true);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(40);

    // EBS valve front should be activate rear not
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    vsv.setAllPressures(vsv.getConfig()->maxEBSPressureFront - 5, vsv.getConfig()->maxEBSPressureRear - 5,
        vsv.getConfig()->maxReleasedBrakePressureFront - 0.2, vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.updateState(140);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(150);

    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    vsv.setAllPressures(vsv.getConfig()->maxEBSPressureFront - 5, vsv.getConfig()->maxEBSPressureRear - 5,
        vsv.getConfig()->minPressurizedBrakePressureFront + 5, vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.updateState(250);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(260);

    // Activate EBS valve rear -> Release rear brake (Step6)
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);

    // Both released in time
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    vsv.setAllPressures(vsv.getConfig()->maxEBSPressureFront - 5, vsv.getConfig()->maxEBSPressureRear - 5,
        vsv.getConfig()->minPressurizedBrakePressureFront + 5, vsv.getConfig()->maxReleasedBrakePressureRear - 0.2);
    vsv.updateState(360);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(370);

    // Both brakes activated again (Step 7)
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);

    // Rear not braking in time
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    vsv.setAllPressures(vsv.getConfig()->maxEBSPressureFront - 5, vsv.getConfig()->maxEBSPressureRear - 5,
        vsv.getConfig()->minPressurizedBrakePressureFront + 5, vsv.getConfig()->minPressurizedBrakePressureRear + 5);
    vsv.updateState(470);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(480);

    ASSERT_EQ(esm->getESUSP(), ESUS::ESUS_Ok);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed_Parking);
}

TEST(ebs_state_machine, while_esus_ebs_pressure_to__low)
{
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());

    vsv.setSDCReady(true);

    vsv.changeToAsOff();

    vsv.setSDCReady(true);

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setBrakePressureFront(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedEBSPressureRear + 1);

    vsv.updateState(1);
    vsv.updateState(2);

    // Testing that the watchdog alternates
    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.updateState(5);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, true);

    vsv.updateState(10);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    EBSStateMachine* esm = vsv.getEBS();

    esm->startESUS(5);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    vsv.updateState(15);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.updateState(20);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    // Simulate the ASSM watchdog
    vsv.setSDCReady(false);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(30);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, true);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(40);

    // EBS valve front should be activate rear not
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    vsv.setAllPressures(vsv.getConfig()->maxEBSPressureFront - 5, vsv.getConfig()->maxEBSPressureRear - 5,
        vsv.getConfig()->maxReleasedBrakePressureFront - 0.2, vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.updateState(140);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(150);

    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);

    // Front activated in time
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    vsv.setAllPressures(vsv.getConfig()->minPressurizedEBSPressureFront - 0.5, vsv.getConfig()->maxEBSPressureRear - 5,
        vsv.getConfig()->minPressurizedBrakePressureFront + 5, vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.updateState(250);

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    vsv.updateState(252);

    ASSERT_EQ(esm->getESUSP(), ESUS::ESUS_Error);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Error);
}

TEST(ebs_state_machine, check_ebs_startup_asms_on)
{
    VehicleStateVector vsv = VehicleStateVector();
    vsv.setConfig(readConfigForTests());
    EBSStateMachine* esm = vsv.getEBS();

    vsv.setSDCReady(true);

    vsv.changeToAsOff();

    vsv.setSDCReady(true);

    vsv.setEBSPressureFront(vsv.getConfig()->maxEBSPressureFront - 5);
    vsv.setBrakePressureFront(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setEBSPressureRear(vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.setBrakePressureRear(vsv.getConfig()->minPressurizedEBSPressureRear + 1);

    vsv.updateState(1);
    vsv.updateState(2);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.updateState(5);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, true);

    vsv.updateState(10);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.setASMSState(ASMS::ASMS_On);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    vsv.updateState(15);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    vsv.updateState(20);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, false);

    // Simulate the ASSM watchdog
    vsv.setSDCReady(false);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(30);

    ASSERT_EQ(vsv.getVehicleStateMsg().watchdog, true);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(40);

    // EBS valve front should be activate rear not
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, true);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    vsv.setAllPressures(vsv.getConfig()->maxEBSPressureFront - 5, vsv.getConfig()->maxEBSPressureRear - 5,
        vsv.getConfig()->maxReleasedBrakePressureFront - 0.2, vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.updateState(140);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(150);

    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    vsv.setAllPressures(vsv.getConfig()->maxEBSPressureFront - 5, vsv.getConfig()->maxEBSPressureRear - 5,
        vsv.getConfig()->minPressurizedBrakePressureFront + 5, vsv.getConfig()->maxEBSPressureRear - 5);
    vsv.updateState(250);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(260);

    // Activate EBS valve rear -> Release rear brake (Step6)
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, true);

    // Both released in time
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    vsv.setAllPressures(vsv.getConfig()->maxEBSPressureFront - 5, vsv.getConfig()->maxEBSPressureRear - 5,
        vsv.getConfig()->minPressurizedBrakePressureFront + 5, vsv.getConfig()->maxReleasedBrakePressureRear - 0.2);
    vsv.updateState(360);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(370);

    // Both brakes activated again (Step 7)
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_front_activated, false);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_valve_rear_activated, false);

    // Rear not braking in time
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    vsv.setAllPressures(vsv.getConfig()->maxEBSPressureFront - 5, vsv.getConfig()->maxEBSPressureRear - 5,
        vsv.getConfig()->minPressurizedBrakePressureFront + 5, vsv.getConfig()->minPressurizedBrakePressureRear + 5);
    vsv.updateState(470);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    vsv.updateState(480);

    ASSERT_EQ(esm->getESUSP(), ESUS::ESUS_Ok);
    ASSERT_EQ(vsv.getVehicleStateMsg().ebs_state, EBS::Armed_Parking);
}
