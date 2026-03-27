/**
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * @file params.cpp
 *
 * @brief Implementation of the params operations for the vcu_shared_lib
 *
 * @author Marvin Jacob <marvin.jacob@elbflorace.de> 2025-2026
 * @author Laurin Hesslich <laurin.hesslich@elbflorace.de> 2026
 *
 * @copyright MIT
 *
 * @version 1.0.0
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */
#include "vcu_shared_lib/params.hpp"

bool loadParams(rclcpp::Node::SharedPtr node, ParameterTuples& paramTuples)
{
    std::vector<rclcpp::Parameter> params;
    std::vector<std::string> failedParams; // non-optional empty params
    std::vector<std::string> emptyParams; // Optional empty params

    for (int i = 0; i < paramTuples.size(); ++i)
    {
        rclcpp::Parameter p(std::get<0>(paramTuples[i]));
        params.push_back(p);

        std::string paramName = p.get_name();

        ParamVariant valuePointer = std::get<1>(paramTuples[i]);

        bool parameterWasSet = false;
        bool isNonOptional = std::holds_alternative<double*>(valuePointer)
            || std::holds_alternative<int64_t*>(valuePointer) || std::holds_alternative<bool*>(valuePointer)
            || std::holds_alternative<std::string*>(valuePointer);

        if (std::holds_alternative<double*>(valuePointer))
        {
            node->declare_parameter(paramName, rclcpp::PARAMETER_DOUBLE);
            parameterWasSet = node->get_parameter(paramName, *(std::get<double*>(valuePointer)));
        }
        else if (std::holds_alternative<int64_t*>(valuePointer))
        {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
            parameterWasSet = node->get_parameter(paramName, *(std::get<int64_t*>(valuePointer)));
        }
        else if (std::holds_alternative<bool*>(valuePointer))
        {
            node->declare_parameter(paramName, rclcpp::PARAMETER_BOOL);
            parameterWasSet = node->get_parameter(paramName, *(std::get<bool*>(valuePointer)));
        }
        else if (std::holds_alternative<std::string*>(valuePointer))
        {
            node->declare_parameter(paramName, rclcpp::PARAMETER_STRING);
            parameterWasSet = node->get_parameter(paramName, *(std::get<std::string*>(valuePointer)));
        }

        if (std::holds_alternative<std::optional<double>*>(valuePointer))
        {
            node->declare_parameter(paramName, rclcpp::PARAMETER_DOUBLE);
            double val;
            parameterWasSet = node->get_parameter(paramName, val);
            if (parameterWasSet)
                *std::get<std::optional<double>*>(valuePointer) = val;
        }
        else if (std::holds_alternative<std::optional<int64_t>*>(valuePointer))
        {
            node->declare_parameter(paramName, rclcpp::PARAMETER_INTEGER);
            int64_t val;
            parameterWasSet = node->get_parameter(paramName, val);
            if (parameterWasSet)
                *std::get<std::optional<int64_t>*>(valuePointer) = val;
        }
        else if (std::holds_alternative<std::optional<bool>*>(valuePointer))
        {
            node->declare_parameter(paramName, rclcpp::PARAMETER_BOOL);
            bool val;
            parameterWasSet = node->get_parameter(paramName, val);
            if (parameterWasSet)
                *std::get<std::optional<bool>*>(valuePointer) = val;
        }
        else if (std::holds_alternative<std::optional<std::string>*>(valuePointer))
        {
            node->declare_parameter(paramName, rclcpp::PARAMETER_STRING);
            std::string val;
            parameterWasSet = node->get_parameter(paramName, val);
            if (parameterWasSet)
                *std::get<std::optional<std::string>*>(valuePointer) = val;
        }

        if (!parameterWasSet && isNonOptional)
            failedParams.push_back(paramName);
        if (!parameterWasSet && !isNonOptional)
            emptyParams.push_back(paramName);
    }

    if (failedParams.size() != 0)
    {
        std::string failedTopics = std::accumulate(failedParams.begin(), failedParams.end(), std::string {},
            [](auto&& a, auto&& b) { return a.empty() ? b : a + "," + b; });
        ;

        RCLCPP_ERROR_STREAM(
            node->get_logger(), "Some required params(s) are not set! Missing params: " << failedTopics);
        rclcpp::shutdown();
        exit(1);
    }

    if (emptyParams.size() != 0)
    {
        std::string failedTopics = std::accumulate(emptyParams.begin(), emptyParams.end(), std::string {},
            [](auto&& a, auto&& b) { return a.empty() ? b : a + "," + b; });
        ;

        RCLCPP_WARN_STREAM(node->get_logger(), "Some optional params(s) are not set! Missing params: " << failedTopics);
    }

    return true;
}
