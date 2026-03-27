#include <ctime>

class DataLoggerData
{
public:
    std::time_t t;
    bool time_inited = false;
    float speedActual;
    float speedTarget;
    float steeringAngleActual;
    float steeringAngleTarget;
    float brakeHydrActual;
    float brakeHydrTarget;
    float momentActual;
    float momentTarget;
    float accelLong;
    float accelLat;
    float yawRate;
    float asState;
    float ebsState;
    float amiState;
    float steeringState;
    float absState;
    float lapCounter;
    float coneCountActual;
    float coneCountAll;
    float resStop;
    float resGoSignal;
    float resQuality;
    float ebsPressureFront;
    float ebsPressureRear;
};

class StatisticData
{
public:
    void setStartTime(int time)
    {
        this->startTime = time;
        std::time_t rawtime;
        std::tm* timeinfo;
        std::time(&rawtime);
        timeinfo = std::localtime(&rawtime);
        std::strftime(this->loggingDateTime, 80, "%Y.%m.%d, %H.%M.%S", timeinfo);
    }
    void beginHV(int time)
    {
        this->hvStartTime = time;
        this->inHV = true;
    }
    void endHV(int time)
    {
        if (this->inHV)
        {
            this->hvRunningTime += time - this->hvStartTime;
            this->inHV = false;
        }
    }
    void increaseDrivenMeters(int meter) { this->drivenMeters += meter; }
    void setLastDataLoggerTimeStamp(int stamp) { this->lastDataLoggerTimeStamp = stamp; }
    void setVehicleMode(bool inAS) { this->inAS = inAS; }

    int getRuntimeHV() { return this->hvRunningTime; }
    int getRuntimeVCU(int time) { return time - this->startTime; }
    int getStartTime() { return this->startTime; }
    int getDrivenMeters() { return this->drivenMeters; }
    int getLastDataLoggerTimeStamp() { return this->lastDataLoggerTimeStamp; }
    char* getLoggingDateTime() { return this->loggingDateTime; }
    bool isInAS() { return this->inAS; }
    bool isInHV() { return this->inHV; }

private:
    int startTime;
    int hvStartTime;
    int hvRunningTime = 0;
    int drivenMeters = 0;
    int lastDataLoggerTimeStamp = 0;
    bool inHV = false;
    char loggingDateTime[80];
    bool inAS;
};