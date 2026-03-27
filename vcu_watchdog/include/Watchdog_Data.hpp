/**
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * @file Watchdog_Data.hpp
 *
 * @brief Defining the structs for the watchdog
 *
 * @author Niklas Leukroth <niklas.leukroth@elbflorace.de> 2024-2026
 * @author Laurin Hesslich <laurin.hesslich@elbflorace.de> 2025-2026
 *
 * @copyright MIT
 *
 * @version 1.0.0
 *
 * @note  This package is compliant with the rules from 2025.
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

#include "rclcpp/rclcpp.hpp"
#include <cstdint>

#define APPS_IMPLAUSIBILITY_LIMIT 0.1
#define ACCEL_PEDAL_IMPLAUSIBLITY_RESET 0.05
#define TH_ACCEL_PEDAL 0.25
#define TH_ACTUAL_POWER 5
#define TH_BRAKEPRESSURE 10
#define WATT_TO_KW 1000
#define MAX_BRAKE_PRESSURE_WHILE_ACCELERATING 15

/**
 * @brief Struct for timeout of PCBs with System Critical Signals,
 *          that must be checked for timeouts.
 *          Needs to get adapted for your specific car!
 */
struct Timeouts
{
    bool pduTimeout;
    bool ssbFrontTimeout;
    bool bseTimeout;
    bool amsTimeout;
    bool energymeterTimeout;
    bool disTimeout;
    bool inverterTimeout;
    bool assmTimeout;
    bool dvTimeout;
};

/**
 * @brief Struct for Errors of System Critical Signals,
 *          that must be checked for plausibility.
 *          Needs to get adapted for your specific car!
 */
struct Errors
{
    bool appsImplausible;
    bool apps1Error;
    bool apps2Error;
    bool brakePressureFrontError;
    bool brakePressureRearError;
    bool brakeForceError;
    bool pedalsImplausible;
    bool ebsPressureFrontError;
    bool ebsPressureRearError;
    bool ebsReleaseFrontError;
    bool ebsReleaseRearError;
    bool recuPedalMoreThanHalf;
    bool accelPedalFull;
};

struct APPS
{
    struct appsValues
    {
        double pressed;
        double released;
    };

    appsValues apps1;
    appsValues apps2;
};

struct CorrectedApps
{
    double apps1;
    double apps2;
};

/**
 * @brief Struct for the times of System Critical Signals,
 *          that must be checked for timeouts.
 *          Needs to get adapted for your specific car, see Timeout struct!
 */
struct Times
{
    // Timeouts
    int64_t periodicTime;
    int64_t pduTime;
    int64_t ssbFrontTime;
    int64_t bseTime;
    int64_t amsTime;
    int64_t energymeterTime;
    int64_t disTime;
    int64_t inverterTime;
    int64_t dvTime;
    int64_t assmTime;

    // Errors
    int64_t appsImplausibleTime;
    int64_t appsErrorTime;
    int64_t brakePressureTime;
    int64_t brakeForceTime;
    int64_t ebsPressureTime;
};

struct LatestErrors
{
    int64_t appsErrors;
    int64_t brakePressureErrors;
    int64_t ebsErrors;
    int64_t ebsReleaseErrors;
};

struct PlausibilityData
{
    double acceleratorPedal;
    double recuperationPedal;
    double actualPower;
    double maxBrakePressure;
    double appsImplausiblityValue;
};

struct RecuStates
{
    bool recuStateButton;
    bool recuStatePedalPlausability;
    bool recuState;
};


struct BrakeforceData
{
    double released;
    double pressed;
    double calibrationOffset;
};

/**
 * @brief Classes saves all the data for the watchdog calculations.
 */
class Watchdog_Data
{
public:
    Watchdog_Data() { };
    Watchdog_Data(Timeouts timeouts, Errors errors)
        : timeouts(timeouts)
        , errors(errors)
    {
    }
    Timeouts* getTimeoutsPointer() { return &this->timeouts; }
    Errors* getErrorsPointer() { return &this->errors; }
    PlausibilityData* getPlausibilityDataPointer() { return &this->plausibilityData; }
    APPS getAPPSCalibration() { return this->appsCalibration; }
    CorrectedApps* getCorrectedAppsPointer() { return &this->correctedAppsValues; }
    RecuStates* getRecuStates() { return &this->recuStates; }
    void setAppsCalibration(APPS appsCalibration) { this->appsCalibration = appsCalibration; }
    bool getRecuActivated() { return recuStates.recuStateButton; }
    void setRecuActivated(bool activatRecu) { recuStates.recuStateButton = activatRecu; }

private:
    Timeouts timeouts;
    Errors errors;
    APPS appsCalibration;
    CorrectedApps correctedAppsValues;
    PlausibilityData plausibilityData;
    RecuStates recuStates;
};
