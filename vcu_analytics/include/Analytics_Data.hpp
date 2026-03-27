#include <algorithm>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <vector>

#include "ix_msgs/msg/inverter.hpp"
#include "ix_msgs/msg/wheels.hpp"
#include "vcu_shared_lib/wheels.hpp"

class EfficiencyMap
{
public:
    std::vector<double> speeds;
    std::vector<double> torques;
    std::vector<std::vector<double>> efficiency;

    void load_csv()
    {

        std::string package_share_dir = ament_index_cpp::get_package_share_directory("vcu_analytics");
        std::string file_path = package_share_dir + "/data/motor_efficiency.csv";

        std::ifstream file(file_path);
        if (!file.is_open())
            throw std::runtime_error("Cannot open file: " + file_path);

        std::string line;
        bool header_read = false;

        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string value;
            std::vector<double> row;
            if (!header_read)
            {
                // speed values
                std::getline(ss, value, ',');
                while (std::getline(ss, value, ','))
                    speeds.push_back(std::stod(value));
                header_read = true;
            }
            else
            {
                // torque value
                std::getline(ss, value, ',');
                torques.push_back(std::stod(value));

                // efficiency values
                while (std::getline(ss, value, ','))
                    row.push_back(std::stod(value));
                efficiency.push_back(row);
            }
        }
    }

    // Bilinear interpolation
    double get_efficiency(double torque, double speed) const
    {

        // Make torque value positive
        torque = std::abs(torque);
        speed = std::abs(speed);

        int i = find_lower_index(torques, torque);
        int j = find_lower_index(speeds, speed);

        if (i < 0 || j < 0 || i + 1 > (int)torques.size() || j + 1 > (int)speeds.size())
            throw std::out_of_range(
                "Requested values out of range : " + std::to_string(torque) + " : " + std::to_string(speed));

        double t1 = torques[i], t2 = torques[i + 1];
        double s1 = speeds[j], s2 = speeds[j + 1];

        double f11 = efficiency[i][j];
        double f12 = efficiency[i + 1][j];
        double f21 = efficiency[i][j + 1];
        double f22 = efficiency[i + 1][j + 1];

        double t = (torque - t1) / (t2 - t1);
        double u = (speed - s1) / (s2 - s1);

        double result = f11 * (1 - t) * (1 - u) + f21 * (1 - t) * u + f12 * t * (1 - u) + f22 * t * u;

        return result;
    }

private:
    int find_lower_index(const std::vector<double>& arr, double val) const
    {

        double max_value = *std::max_element(arr.begin(), arr.end());
        double min_value = *std::min_element(arr.begin(), arr.end());

        if (val < min_value)
            return 0;

        if (val > max_value)
            return (arr.size() - 1);

        for (size_t i = 0; i < arr.size() - 1; ++i)
        {
            if (val >= arr[i] && val <= arr[i + 1])
                return i;
        }
        return -1;
    }
};

class AnalyticsData
{
public:
    AnalyticsData() { this->effMap.load_csv(); }

    void setCurrentWSPD(const ix_msgs::msg::Wheels::SharedPtr msg)
    {
        this->currentWSPD.fl = msg->fl;
        this->currentWSPD.fr = msg->fr;
        this->currentWSPD.rl = msg->rl;
        this->currentWSPD.rr = msg->rr;
    }

    ix_msgs::msg::Wheels getWheelMotorEfficiency(const ix_msgs::msg::Inverter::SharedPtr msg)
    {

        Wheels<float> out;

        out.fl = this->effMap.get_efficiency(msg->actual_torque.fl, this->currentWSPD.fl);
        out.fr = this->effMap.get_efficiency(msg->actual_torque.fr, this->currentWSPD.fr);
        out.rl = this->effMap.get_efficiency(msg->actual_torque.rl, this->currentWSPD.rl);
        out.rr = this->effMap.get_efficiency(msg->actual_torque.rr, this->currentWSPD.rr);

        return out.to_msg();
    }

private:
    Wheels<float> currentWSPD;
    EfficiencyMap effMap;
};
