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
#include "rebet_msgs/msg/objects_identified.hpp"

#include "rebet_frog/frog_constants.hpp"

#include "rebet_msgs/msg/qr.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

using ObjectsIdentified = rebet_msgs::msg::ObjectsIdentified;

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
          InputPort<std::vector<ObjectsIdentified>>(OBJS_DETECTED),
          InputPort<double>(DETECT_POWER_METRIC)
        };
        child_ports.merge(base_ports);
  
        return child_ports;
      }
    
      virtual std::vector<KV_MSG> collect_config() override
      {
        std::vector<KV_MSG> current_config;
        double power_budget;
        std::vector<ObjectsIdentified> objects;
  
        auto power_res = getInput(DETECT_POWER_METRIC, power_budget);
        auto objects_res = getInput(OBJS_DETECTED, objects);
     
        if (power_res) {
          auto kv_power_left = KV_MSG();
          kv_power_left.key = "power_left";
          // Fuzzify data
          int power_rounded = static_cast<int>(power_budget); // Always round down
          kv_power_left.value = std::to_string(power_rounded);
          current_config.push_back(kv_power_left);
        }
     
        if (objects_res) {
          auto kv_obs_idd = KV_MSG();
          kv_obs_idd.key = "obs_idd";
          int num_obs_idd = objects.size();
          kv_obs_idd.value = std::to_string(num_obs_idd);
          current_config.push_back(kv_obs_idd);
        } else { // If this is the first time, objs_detected has not been written to blackboard yet.
          auto kv_obs_idd = KV_MSG();
          kv_obs_idd.key = "obs_idd";
          int num_obs_idd = 0;
          kv_obs_idd.value = std::to_string(num_obs_idd);
          current_config.push_back(kv_obs_idd);
        }
  
        auto kv_are_all_objects_visited = KV_MSG();
        kv_are_all_objects_visited.key = "are_all_objects_visited";
        kv_are_all_objects_visited.value = std::to_string(0); // always no, otherwise we wouldn't adapt
        current_config.push_back(kv_are_all_objects_visited);
  
        return current_config;
      }
  
    private:
      static constexpr const char* OBJS_DETECTED = "objs_detected";
      static constexpr const char* DETECT_POWER_METRIC = "detect_power_metric";
};
}