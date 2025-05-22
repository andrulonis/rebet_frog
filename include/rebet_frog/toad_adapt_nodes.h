#pragma once

#include "rclcpp/rclcpp.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include <algorithm>
#include <chrono>
#include <ctime> 
#include "rebet/system_attribute_value.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rebet/adapt_node.h"
#include "rebet/rebet_utilities.hpp"

#include "rebet_frog/frog_constants.hpp"

#include "rebet_msgs/msg/qr.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace BT
{

class AdjustDetectModel: public AdaptOnConditionOnStart<std::string>
{
  public:
    using QR_MSG = rebet_msgs::msg::QR;
    using KV_MSG = diagnostic_msgs::msg::KeyValue;

    AdjustDetectModel(const std::string& name, const NodeConfig& config) : AdaptOnConditionOnStart<std::string>(name, config, AdaptationTarget::RosParameter, AdaptationType::External)
    {
        _condition_default = true;

        std::vector<std::string> param_values;
        std::string param_name;
        std::string node_name;
        getInput(ADAP_OPT, param_values);
        getInput(ADAP_SUB, param_name);
        getInput(ADAP_LOC, node_name);

        aal_msgs::msg::AdaptationOptions variable_param = aal_msgs::msg::AdaptationOptions();

        variable_param.name = param_name;
        variable_param.node_name = node_name;
        variable_param.adaptation_target_type = static_cast<int8_t>(adaptation_target_);

        for (std::string val : param_values) {
            rclcpp::ParameterValue par_val = rclcpp::ParameterValue(val);
            variable_param.possible_values.push_back(par_val.to_value_msg());
        }

        _var_params.push_back(variable_param);
      }
  
      static PortsList providedPorts()
      {
        PortsList base_ports = AdaptOnConditionOnStart::providedPorts();
  
        PortsList child_ports =  {
                };
        child_ports.merge(base_ports);
  
        return child_ports;
      }
  
      virtual double utility_of_adaptation(rcl_interfaces::msg::Parameter ros_parameter) override
      {
        std::cout << "util_of_adap_max_speed" << std::endl;
        auto parameter_object = rclcpp::ParameterValue(ros_parameter.value);
  
        return 0.0;
      }
  
    private:
};
}