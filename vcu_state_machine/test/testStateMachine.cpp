#include <StateMachineData.hpp>
#include <gtest/gtest.h>

// Including other test suits
#include "testStateTransition01.cpp"
#include "testStateTransition02.cpp"
#include "testStateTransition03.cpp"
#include "testStateTransition04.cpp"
#include "testStateTransition05.cpp"
#include "testStateTransition06.cpp"
#include "testStateTransition07.cpp"
#include "testStateTransition08.cpp"
#include "testStateTransition09.cpp"
#include "testStateTransition10.cpp"
#include "testStateTransition11.cpp"
#include "testStateTransition12.cpp"
#include "testStateTransition13.cpp"
#include "testStateTransition14.cpp"
#include "testStateTransition15.cpp"
#include "testStateTransition16.cpp"
#include "testStateTransition17.cpp"
#include "testStateTransition18.cpp"
#include "testStateTransition20.cpp"
#include "testStateTransition21.cpp"
#include "testStateTransition22.cpp"

#include "testEBSStateMachine.cpp"

TEST(state_machine, first_test) { ASSERT_EQ(42, 2 * 21); }

TEST(state_machine, setbmcSdcState_test)
{
    VehicleStateVector vehicleStateVector = VehicleStateVector();
    ASSERT_EQ(vehicleStateVector.getbmcSdcState(), false);

    vehicleStateVector.setbmcSdcState(true);
    ASSERT_EQ(vehicleStateVector.getbmcSdcState(), true);
}

TEST(state_machine, setASMSOn_test)
{
    VehicleStateVector vehicleStateVector = VehicleStateVector();
    ASSERT_EQ(vehicleStateVector.getASMSState(), ASMS::ASMS_Off);

    vehicleStateVector.setASMSState(ASMS::ASMS_On);
    ASSERT_EQ(vehicleStateVector.getASMSState(), ASMS::ASMS_On);
}

TEST(state_machine, setMissionStateSelected_test)
{
    VehicleStateVector vehicleStateVector = VehicleStateVector();
    ASSERT_EQ(vehicleStateVector.getMissionState(), MissionState::MS_NotSelected);

    vehicleStateVector.setMissionState(MissionState::MS_Selected);
    ASSERT_EQ(vehicleStateVector.getMissionState(), MissionState::MS_Selected);
}

TEST(state_machine, setAMSDrive_test)
{
    VehicleStateVector vehicleStateVector = VehicleStateVector();
    ASSERT_EQ(vehicleStateVector.getAMSState(), AMS::AMS_Idle);
    vehicleStateVector.setAMSState(AMS::AMS_Drive);
    ASSERT_EQ(vehicleStateVector.getAMSState(), AMS::AMS_Drive);
}

TEST(state_machine, addError_test)
{
    VehicleStateVector vehicleStateVector = VehicleStateVector();

    std::list<Error> f_errors = { Error::E_None };
    ASSERT_EQ(vehicleStateVector.getErrors(), f_errors);

    vehicleStateVector.addError(Error::E_APPSImplausible);

    std::list<Error> s_errors = { Error::E_APPSImplausible };
    ASSERT_EQ(vehicleStateVector.getErrors(), s_errors);
}

TEST(state_machine, setErrors_test)
{
    VehicleStateVector vehicleStateVector = VehicleStateVector();
    std::list<Error> f_errors = { Error::E_None };

    ASSERT_EQ(vehicleStateVector.getErrors(), f_errors);

    std::list<Error> s_errors = { Error::E_APPSImplausible, Error::E_InverterTimeout };
    vehicleStateVector.setErrors(s_errors);

    ASSERT_EQ(vehicleStateVector.getErrors(), s_errors);
}

TEST(state_machine, resetErrors_test)
{
    VehicleStateVector vehicleStateVector = VehicleStateVector();
    std::list<Error> f_errors = { Error::E_None };

    ASSERT_EQ(vehicleStateVector.getErrors(), f_errors);

    std::list<Error> s_errors = { Error::E_APPSImplausible, Error::E_InverterTimeout };
    vehicleStateVector.setErrors(s_errors);

    ASSERT_EQ(vehicleStateVector.getErrors(), s_errors);

    vehicleStateVector.resetErrors();
    ASSERT_EQ(vehicleStateVector.getErrors(), f_errors);
}
