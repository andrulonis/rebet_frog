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

    // override to calculate measure before ticking child
    virtual NodeStatus tick() override
    {
      setStatus(NodeStatus::RUNNING);
      calculate_measure();
      const NodeStatus child_status = child_node_->executeTick();

      switch (child_status)
      {
        case NodeStatus::SUCCESS: {
          resetChild();
          return NodeStatus::SUCCESS;
        }

        case NodeStatus::FAILURE: {
          resetChild();
          return NodeStatus::FAILURE;
        }

        case NodeStatus::RUNNING: {
          return NodeStatus::RUNNING;
        }

        case NodeStatus::SKIPPED: {
          return NodeStatus::SKIPPED;
        }
        case NodeStatus::IDLE: {
          throw LogicError("[", name(), "]: A child should not return IDLE");
        }
      }
      return status();

    }
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
    int window_start;

    MovementEfficiencyQR(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::MovementEfficiency)
    {
      _window_start = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      _last_odom = _window_start;
      _odom_last_timestamp = builtin_interfaces::msg::Time();
    }

    void initialize(int window_length)
    {
        _window_length = window_length;
    }


    static PortsList providedPorts()
    {
      PortsList base_ports = TaskLevelQR::providedPorts();

      PortsList child_ports = { 
              InputPort<rebet::SystemAttributeValue>(IN_ODOM,"odometry message wrapped in a systemattributevalue instance"),
              };

      child_ports.merge(base_ports);

      return child_ports;
    }
	
    virtual void calculate_measure() override
    {
      auto res = getInput(IN_ODOM,_odom_attribute); 
      if(res)
      {
        _odom_msg = _odom_attribute.get<rebet::SystemAttributeType::ATTRIBUTE_ODOM>();


        if(_odom_msg.header.stamp != _odom_last_timestamp)
        {
          _linear_speed = hypot(fabs(_odom_msg.twist.twist.linear.x), fabs(_odom_msg.twist.twist.linear.y));
          _odom_last_timestamp = _odom_msg.header.stamp;

          if(_linear_speed > 0.03) //this is to circumvent a start from stationary being considered as part of this QR.
          {
            float speed_ratio = (_linear_speed/WAFFLE_MAX_LIN_VEL);
            _metric = std::clamp(speed_ratio,0.0f,1.0f);

            output_metric();
            metric_mean();

            setOutput(MEAN_METRIC,_average_metric);
          }
        }
      }

      // auto curr_time_pointer = std::chrono::system_clock::now();

      // int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
      // int elapsed_seconds = current_time-_window_start;

      // if(elapsed_seconds >= _window_length)
      // {
      //   _window_start = current_time;
      // }

    }
  private:
      rebet::SystemAttributeValue _odom_attribute;
      int _window_length;
      int _window_start;
      int _last_odom;
      float _linear_speed;
      builtin_interfaces::msg::Time _odom_last_timestamp;


      nav_msgs::msg::Odometry _odom_msg;
      int _odom_last_timestamp_sec;

      const float WAFFLE_MAX_LIN_VEL = 0.26;

      static constexpr const char* IN_ODOM = "in_odom";
};
