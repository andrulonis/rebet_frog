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

class AdjustMaxSpeed : public AdaptPeriodicallyOnRunning<double>
{
  public:
    using QR_MSG = rebet_msgs::msg::QR;

    AdjustMaxSpeed(const std::string& name, const NodeConfig& config) : AdaptPeriodicallyOnRunning<double>(name, config, AdaptationTarget::RosParameter, AdaptationType::External)
    {
      _condition_default = false;

      //Since we are only interested in modifying the x-axis (backwards/forwards) speed, we wrap the adaptation_options given into the required triple of x y theta speeds.
      std::vector<double> param_values;
      std::string param_name;
      std::string node_name;
      getInput(ADAP_OPT, param_values);
      getInput(ADAP_SUB, param_name);
      getInput(ADAP_LOC, node_name);
      
      power_qr_max_value = (double)calculate_power_motion(WAFFLE_MAX_LIN_VEL);
      aal_msgs::msg::AdaptationOptions variable_param = aal_msgs::msg::AdaptationOptions();

      variable_param.name = param_name;
      variable_param.node_name = node_name;
      variable_param.adaptation_target_type = static_cast<int8_t>(adaptation_target_);

      for (double val : param_values) {
          std::vector<double> speed_vector = {val,_default_y_velocity,_default_theta_velocity};
          rclcpp::ParameterValue par_val = rclcpp::ParameterValue(speed_vector); //Here we wrap it with default values
          variable_param.possible_values.push_back(par_val.to_value_msg());
      }
      _var_params.push_back(variable_param); //vector of VariableParameter   

    //If you overwrite tick, and do this at different moments you can change the adaptation options at runtime.  
    }

    virtual bool evaluate_condition() override
    {
      this->getInput(WINDOW_LEN,_window_length);
      auto curr_time_pointer = std::chrono::system_clock::now();
      int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
      int elapsed_seconds = current_time-_window_start;

      bool window_expired = elapsed_seconds >= _window_length;

      if(window_expired)
      {
        _window_start = current_time;
      }

      // Also take into consideration movement speed
      double current_speed;
      getInput(MOVE_EFF_METRIC, current_speed);
      bool high_speed = current_speed >= 0.17;

      return window_expired && high_speed;
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptPeriodicallyOnRunning::providedPorts();

      PortsList child_ports =  {
        InputPort<double>(MOVE_EFF_METRIC),
        InputPort<double>(DISTANCE_TO_POSE),
        InputPort<double>(SAFETY_METRIC),
        InputPort<double>(MEAN_SAFETY_METRIC)
        };
      child_ports.merge(base_ports);

      return child_ports;
    }

    virtual std::vector<KV_MSG> collect_config() override
      {
        std::vector<KV_MSG> current_config;
        double distance_to_pose;
        double current_speed;
        double safety;
  
        auto distance_to_pose_res = getInput(DISTANCE_TO_POSE, distance_to_pose);
        auto current_speed_res = getInput(MOVE_EFF_METRIC, current_speed);
        auto safety_res = getInput(SAFETY_METRIC, safety);

  
        if (distance_to_pose_res) {
          auto kv_distance_to_pose = KV_MSG();
          kv_distance_to_pose.key = "distance_to_pose";
          // Fuzzify data
          int distance_in_cm = static_cast<int>(distance_to_pose*100); // Always round down
          kv_distance_to_pose.value = std::to_string(distance_in_cm);
          current_config.push_back(kv_distance_to_pose);
        }

        if (current_speed_res) {
          auto kv_is_fast = KV_MSG();
          int is_fast;
          is_fast = current_speed < 0.19 ? 0 : 1;
          kv_is_fast.key = "is_fast";
          kv_is_fast.value = std::to_string(is_fast);
          current_config.push_back(kv_is_fast);
        }

        if (safety_res) { 
          auto kv_is_unsafe = KV_MSG();
          int is_unsafe = safety < 0.10 ? 1 : 0;
          kv_is_unsafe.key = "is_unsafe";
          kv_is_unsafe.value = std::to_string(is_unsafe);
          current_config.push_back(kv_is_unsafe);
        }

        auto kv_short_term = KV_MSG();
        kv_short_term.key = "short_term";
        kv_short_term.value = std::to_string(1);
        current_config.push_back(kv_short_term);
  
        return current_config;
      }

    virtual std::vector<KV_MSG> collect_context() override
      {
        std::vector<KV_MSG> current_context;
        double safety;
        double mean_safety;
  
        auto safety_res = getInput(SAFETY_METRIC, safety);
        auto mean_safety_res = getInput(MEAN_SAFETY_METRIC, mean_safety);
  
        if (safety_res) {
          auto kv_safety = KV_MSG();
          kv_safety.key = "current_safety";
          kv_safety.value = std::to_string(safety);
          current_context.push_back(kv_safety);
        }

        if (mean_safety_res) {
          auto kv_mean_safety = KV_MSG();
          kv_mean_safety.key = "mean_safety";
          kv_mean_safety.value = std::to_string(mean_safety);
          current_context.push_back(kv_mean_safety);
        }
  
        return current_context;
      }

  private:
    static constexpr const char* MOVE_EFF_METRIC ="move_eff_metric";
    static constexpr const char* DISTANCE_TO_POSE ="distance_to_pose";
    static constexpr const char* SAFETY_METRIC ="safety_metric";
    static constexpr const char* MEAN_SAFETY_METRIC ="mean_safety_metric";
    


    double _default_y_velocity = 0.0; //The robot in question can not move along its y axis independently.
    double _default_theta_velocity = 2.5; //We are not interested in modifying the rotation speed here, but could be extended to do so.
    double power_qr_max_value;
};
}