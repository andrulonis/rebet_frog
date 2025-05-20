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

class PicturesPowerQR : public TaskLevelQR
{
  public:
    PicturesPowerQR(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::Power)
    {
      _obj_last_timestamp = builtin_interfaces::msg::Time();
      time_passed = 0;
      pictures_taken = 0;

      setOutput(METRIC, pictures_taken * time_passed);
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = TaskLevelQR::providedPorts();

      PortsList child_ports =  {
              InputPort<int>(TOTAL_PICS),
              };
      child_ports.merge(base_ports);

      return child_ports;
    }

    virtual void calculate_measure() override
    {
      // std::cout << " calc measure object det power qr" << std::endl;
        int total_pics;
        getInput(TOTAL_PICS, total_pics);
        
        // TODO
        int power_consumed = 3*total_pics;
        setOutput(METRIC, power_consumed);
    }
  private:
      int pictures_taken;
      double time_passed;
      
      builtin_interfaces::msg::Time _obj_last_timestamp;

      static constexpr const char* TOTAL_PICS = "total_pics";
};
