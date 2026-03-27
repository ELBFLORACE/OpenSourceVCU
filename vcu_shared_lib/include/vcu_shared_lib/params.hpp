/**
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * @file params.hpp
 *
 * @brief Defining the params for the vcu_shared_lib
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

#ifndef vcu_shared_lib_PARAMS_HPP
#define vcu_shared_lib_PARAMS_HPP

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdint>
#include <type_traits>

using ParamVariant = std::variant<double*, // required double storage
    int64_t*, // required int storage
    bool*, // required bool storage
    std::string*, // required string storage
    std::optional<double>*, // optional double storage (pointer to std::optional<double>)
    std::optional<int64_t>*, // optional int storage
    std::optional<bool>*, // optional bool storage
    std::optional<std::string>* // optional string storage
    >;
using ParameterTuple = std::tuple<std::string, ParamVariant>;
using ParameterTuples = std::vector<ParameterTuple>;

/**
 * Load the paramter values for the provided tuples.
 *
 * @return Parameter Event handles, these need to be kept in a variable until the end of the nodes lifecycle
 */
bool loadParams(rclcpp::Node::SharedPtr node, ParameterTuples& paramTuples);

#endif
