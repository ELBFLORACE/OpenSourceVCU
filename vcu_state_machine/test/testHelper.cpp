#ifndef TEST_HELPER_CPP
#define TEST_HELPER_CPP
#include <StateMachineData.hpp>
#include <TEST_CONFIG.hpp>
#include <iostream>

#define S_TO_MS 1000

static Config readConfigForTests()
{
    Config config;

    config.maxPrechargeTime = TCFG_MAX_PRECHARGE_TIME * S_TO_MS;
    config.normalPrechargeTime = TCFG_NORMAL_PRECHARGE_TIME * S_TO_MS;

    config.brakePressureForR2D = TCFG_BRAKE_PRESSURE_FOR_R2D;
    config.maxReleasedEBSPressureFront = TCFG_MAX_RELEASED_EBS_PRESSURE_FRONT;
    config.maxReleasedEBSPressureRear = TCFG_MAX_RELEASED_EBS_PRESSURE_REAR;
    config.minPressurizedEBSPressureFront = TCFG_MIN_PRESSURIZED_EBS_PRESSURE_FRONT;
    config.minPressurizedEBSPressureRear = TCFG_MIN_PRESSURIZED_EBS_PRESSURE_REAR;
    config.maxEBSPressureFront = TCFG_MAX_EBS_PRESSURE_FRONT;
    config.maxEBSPressureRear = TCFG_MAX_EBS_PRESSURE_REAR;
    config.minEBSPressureForAsOff = TCFG_MIN_EBS_PRESSURE_FOR_ASOFF;
    config.maxReleasedBrakePressureFront = TCFG_MAX_RELEASED_BRAKE_PRESSURE_FRONT;
    config.maxReleasedBrakePressureRear = TCFG_MAX_RELEASED_BRAKE_PRESSURE_REAR;
    config.minPressurizedBrakePressureFront = TCFG_MIN_PRESSURIZED_BRAKE_PRESSURE_FRONT;
    config.minPressurizedBrakePressureRear = TCFG_MIN_PRESSURIZED_BRAKE_PRESSURE_REAR;

    return config;
}

#endif
