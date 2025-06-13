#pragma once

#include "behaviortree_cpp/decorator_node.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rebet_msgs/msg/objects_identified.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <algorithm>
#include <chrono>
#include <ctime> 
#include "rebet/system_attribute_value.hpp"
#include "rebet/qr_node.h"

#include "rebet_frog/toad_constants.hpp"


/**
 * @brief The QRNode is used to constrain variable action nodes it decorates.
 *
 *
 * 
 *
 * 
 * 
 * 
 * 
 * 
 * Note: If in the future a BT should be designed with multiple instances of the same QR, it would become impractical. Right now the only feasible case it having two QRs which are never active simultaneously. 
 * In all other cases it would require creating a new (very similar) subclass as the linked blackboard entries would otherwise cause issue.
 */


using namespace BT;
using namespace rebet;
using ObjectsIdentified = rebet_msgs::msg::ObjectsIdentified;

class DetectPowerQR : public TaskLevelQR
{
  public:
    DetectPowerQR(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::Power)
    {
      power_budget = STARTING_BUDGET;
      num_objects_idd = 0;
      setOutput(METRIC, power_budget);
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = TaskLevelQR::providedPorts();

      PortsList child_ports =  {
              InputPort<std::vector<ObjectsIdentified>>(OBJS_DETECTED)
              };
      child_ports.merge(base_ports);

      return child_ports;
    }

    virtual void calculate_measure() override
    {
      std::vector<ObjectsIdentified> results;
      getInput(OBJS_DETECTED, results);
      for (long unsigned i = num_objects_idd; i < results.size(); i++){
        if (results[i].model_used == "yolov8x"){
          power_budget -= V8X_POWER_COST;
        }
        else if (results[i].model_used == "yolov8n"){
          power_budget -= V8N_POWER_COST;
        }
      }
      num_objects_idd = results.size();
      setOutput(METRIC, power_budget);
    }
  private:
    static constexpr const char* OBJS_DETECTED = "objs_detected";

    double power_budget;
    int num_objects_idd;
};

class DetectAccuracyQR : public TaskLevelQR
{
  public:
    DetectAccuracyQR(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::Power)
    {
      accuracy_sum = 0;
      num_objects_idd = 0;
      setOutput(METRIC, accuracy_sum);
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = TaskLevelQR::providedPorts();

      PortsList child_ports =  {
              InputPort<std::vector<ObjectsIdentified>>(OBJS_DETECTED)
              };
      child_ports.merge(base_ports);

      return child_ports;
    }

    virtual void calculate_measure() override
    {
      std::vector<ObjectsIdentified> results;
      getInput(OBJS_DETECTED, results);
      for (long unsigned int i = num_objects_idd; i < results.size(); i++){
        if (results[i].model_used == "yolov8x"){
          accuracy_sum += V8X_ACCURACY;
        }
        else if (results[i].model_used == "yolov8n"){
          accuracy_sum += V8N_ACCURACY;
        }
      }
      num_objects_idd = results.size();
      setOutput(METRIC, accuracy_sum);
    }

  private:
    static constexpr const char* OBJS_DETECTED = "objs_detected";

    double accuracy_sum;
    int num_objects_idd;
};

class MovementEfficiencyQR : public TaskLevelQR
{
  public:
    builtin_interfaces::msg::Time last_timestamp;

    MovementEfficiencyQR(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::MovementEfficiency)
    {
      _odom_last_timestamp = builtin_interfaces::msg::Time();
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = TaskLevelQR::providedPorts();

      PortsList child_ports = { 
              InputPort<rebet::SystemAttributeValue>(ODOMETRY),
              InputPort<double>(CHOSEN_MAX_SPEED),
              OutputPort<double>(CURRENT_SPEED)
              };

      child_ports.merge(base_ports);

      return child_ports;
    }
	
    virtual void calculate_measure() override
    {
      rebet::SystemAttributeValue odom_attribute;
      auto res = getInput(ODOMETRY,odom_attribute); 
      if(res)
      {
        nav_msgs::msg::Odometry odom_msg = odom_attribute.get<rebet::SystemAttributeType::ATTRIBUTE_ODOM>();


        if(odom_msg.header.stamp != _odom_last_timestamp)
        {
          double linear_speed = hypot(fabs(odom_msg.twist.twist.linear.x), fabs(odom_msg.twist.twist.linear.y));
          _odom_last_timestamp = odom_msg.header.stamp;

          setOutput(CURRENT_SPEED, linear_speed);

          double max_speed;
          if (getInput(CHOSEN_MAX_SPEED, max_speed)) {
            _metric = max_speed;
            output_metric();
            metric_mean();
            setOutput(MEAN_METRIC, _average_metric);
          }
        }
      }
    }
  private:
    static constexpr const char* CURRENT_SPEED = "current_speed";
    static constexpr const char* ODOMETRY = "odometry";
    static constexpr const char* CHOSEN_MAX_SPEED = "chosen_max_speed";

    builtin_interfaces::msg::Time _odom_last_timestamp;
};

class SafetyQR : public TaskLevelQR
{
  public:
    SafetyQR(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::Safety)
    {
      mean_safety = 0;
      times_safety_calculated = 0;
      times_fast_while_unsafe = 0;
      obj_last_timestamp = builtin_interfaces::msg::Time();
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = TaskLevelQR::providedPorts();

      PortsList child_ports = { 
              InputPort<rebet::SystemAttributeValue>(LASER_SCAN),
              InputPort<double>(CURRENT_SPEED),
              InputPort<double>(CHOSEN_MAX_SPEED),
              OutputPort<double>(CURRENT_SAFETY),
              OutputPort<double>(MEAN_SAFETY)
              };

      child_ports.merge(base_ports);

      return child_ports;
    }

    bool is_unsafe(double lidar_range, double speed){
      return lidar_range < 0.1 && speed > 0.18;
    }
	
    virtual void calculate_measure() override
    {
      rebet::SystemAttributeValue laser_attribute;    
      auto res = getInput(LASER_SCAN,laser_attribute); 

      if(res)
      {
        sensor_msgs::msg::LaserScan laser_msg = laser_attribute.get<rebet::SystemAttributeType::ATTRIBUTE_LASER>();
        if(laser_msg.header.stamp != obj_last_timestamp)
        {
          obj_last_timestamp = laser_msg.header.stamp; 

          float laser_min = laser_msg.range_min;
          float laser_max = laser_msg.range_max;

          float nearest_object = laser_max*2; //anything above laser_max should work

          for (float const & laser_dist : laser_msg.ranges)
          {
            if(laser_dist < laser_max && laser_dist > laser_min)
            {
              nearest_object = (laser_dist < nearest_object) ? laser_dist : nearest_object;
            }
          }

          float fitted_nearest = (nearest_object - laser_min) / (laser_max - laser_min);
          double safety = std::clamp(fitted_nearest,0.0f,1.0f);

          times_safety_calculated++;
          mean_safety = mean_safety + ((safety - mean_safety) / (double) times_safety_calculated);

          setOutput(CURRENT_SAFETY, safety);
          setOutput(MEAN_SAFETY, mean_safety);

          double current_speed;
          double chosen_speed;
          getInput(CURRENT_SPEED, current_speed);
          getInput(CHOSEN_MAX_SPEED, chosen_speed);
          if (is_unsafe(safety, std::min(current_speed,chosen_speed)))
            times_fast_while_unsafe++;

          _metric = (double) times_fast_while_unsafe / (double) times_safety_calculated;
          output_metric();
        }
      }
    }

  private:
    double mean_safety;
    int times_safety_calculated;
    int times_fast_while_unsafe;
    builtin_interfaces::msg::Time obj_last_timestamp;

    static constexpr const char* CURRENT_SPEED = "current_speed";
    static constexpr const char* CHOSEN_MAX_SPEED = "chosen_max_speed";
    static constexpr const char* CURRENT_SAFETY = "current_safety";
    static constexpr const char* MEAN_SAFETY = "mean_safety";
    static constexpr const char* LASER_SCAN = "laser_scan";
};
