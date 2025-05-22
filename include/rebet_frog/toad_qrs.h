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

#include "rebet_frog/frog_constants.hpp"
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
    static constexpr const char* OBJS_DETECTED = "objs_detected";


    DetectPowerQR(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::Power)
    {
      power_budget = STARTING_BUDGET;
      num_objects_idd = 0;
      setOutput(METRIC, 0);
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
      for (int i = num_objects_idd; i < results.size(); i++){
        num_objects_idd++;
        if (results[i].model_used == "yolov8x"){
          power_budget -= V8X_POWER_COST;
        }
        else if (results[i].model_used == "yolov8n"){
          power_budget -= V8N_POWER_COST;
        }
      }
      setOutput(METRIC, power_budget);
    }
  private:
    double power_budget;
    int num_objects_idd;
};

class DetectAccuracyQR : public TaskLevelQR
{
  public:
    static constexpr const char* OBJS_DETECTED = "objs_detected";


    DetectAccuracyQR(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::Power)
    {
      setOutput(METRIC, 0);
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
      float power = 0;
      for (int i = 0; i < results.size(); i++){
        if (results[i].model_used == "yolov8x"){
          power += 4;
        }
        else if (results[i].model_used == "yolov8n"){
          power += 3;
        }
      }
      setOutput(METRIC, power);
    }
  private:
};
