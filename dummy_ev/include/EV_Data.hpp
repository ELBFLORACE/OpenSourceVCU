#include "ix_msgs/msg/wheels.hpp"
#include "rclcpp/rclcpp.hpp"

struct APPS
{
    struct appsValues
    {
        float pressed;
        float released;
    };

    appsValues apps1;
    appsValues apps2;
};

template <typename T> struct Wheels
{
public:
    T fl;
    T fr;
    T rl;
    T rr;

    Wheels() { };
    Wheels(const ix_msgs::msg::Wheels msg)
    {
        this->fl = (T)msg.fl;
        this->fr = (T)msg.fr;
        this->rl = (T)msg.rl;
        this->rr = (T)msg.rr;
    };
    Wheels(T fl, T fr, T rl, T rr)
    {
        this->fl = fl;
        this->fr = fr;
        this->rl = rl;
        this->rr = rr;
    }

    void crop(Wheels<T> lower, Wheels<T> upper)
    {
        this->fl = max(lower.fl, min(this->fl, upper.fl));
        this->fr = max(lower.fr, min(this->fr, upper.fr));
        this->rl = max(lower.rl, min(this->rl, upper.rl));
        this->rr = max(lower.rr, min(this->rr, upper.rr));
    }

    void crop(T lower, T upper)
    {
        this->fl = max(lower, min(this->fl, upper));
        this->fr = max(lower, min(this->fr, upper));
        this->rl = max(lower, min(this->rl, upper));
        this->rr = max(lower, min(this->rr, upper));
    }

    ix_msgs::msg::Wheels to_msg()
    {
        auto msg = ix_msgs::msg::Wheels();
        msg.fl = (float)this->fl;
        msg.fr = (float)this->fr;
        msg.rl = (float)this->rl;
        msg.rr = (float)this->rr;
        return msg;
    }
};

class EV_Data
{
public:
    APPS getAPPSCalibration() { return this->appsCalibration; }
    void setAppsCalibration(APPS appsCalibration) { this->appsCalibration = appsCalibration; }

private:
    APPS appsCalibration;
};