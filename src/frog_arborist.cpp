#include "rebet_frog/filter_obstacles.h"
#include "rebet_frog/dock_location.h"

#include "rebet_frog/frog_qrs.h"
#include "rebet_frog/frog_adapt_nodes.h"
#include "rebet/arborist.hpp"
// #include "rebet/sleep_node.h"

#include "std_msgs/msg/string.hpp"


class FrogArborist : public Arborist
{
public:
  int time_since_last = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  int total_elapsed = 0;
  int time_limit = 300;

  FrogArborist(const rclcpp::NodeOptions& options) : Arborist(options)
  {
      publisher_ = node()->create_publisher<std_msgs::msg::String>("frog_arborist/reporting", 10);
  }
  void registerNodesIntoFactory(BT::BehaviorTreeFactory& factory) override
  {    
    //I suppose here you register all the possible custom nodes, and the determination as to whether they are actually used lies in the xml tree provided.      
    
    factory.registerNodeType<FilterObstacles>("filterObstacles");
    // factory.registerNodeType<SleepNode>("Sleep");

    factory.registerNodeType<ObjectDetectionEfficiencyQR>("DetectObjectsEfficiently");
    factory.registerNodeType<SimpleSystemPowerQR>("KeepBatteryMin");
    factory.registerNodeType<SystemPowerQR>("PowerQR");
    factory.registerNodeType<SafetyQR>("SafetyQR");
    factory.registerNodeType<ObjectDetectionPowerQR>("DetectObjectsSavePower");


    factory.registerNodeType<MovementEfficiencyQR>("MovementEfficiencyQR");
    factory.registerNodeType<MovementPowerQR>("MovementPowerQR");


    factory.registerNodeType<AdaptPictureRateExternal>("AdaptPictureRate");
    factory.registerNodeType<AdaptPictureRateInternal>("AdaptPictureRateOff");

    factory.registerNodeType<AdaptMaxSpeedExternal>("AdaptMaxSpeed");
    factory.registerNodeType<AdaptMaxSpeedInternal>("AdaptMaxSpeedOff");

    factory.registerNodeType<FromExploreToIdentify>("FromExploreToIdentify");


    // factory.registerNodeType<AdaptChargeConditionInternal>("WhetherToCharge");
    factory.registerNodeType<SetDockLocation>("SetChargingDockLocation");
  }

  std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus status) override
  {
        auto curr_time_pointer = std::chrono::system_clock::now();


        int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
        int elapsed_seconds = current_time-time_since_last;
        //Every second I record an entry
        if(elapsed_seconds >= 1 )
        {
          total_elapsed++;
          RCLCPP_INFO(node()->get_logger(), "%d seconds have passed", total_elapsed);
          RCLCPP_INFO(node()->get_logger(), "current_task: %s", getStringOrNot("current_task").c_str());

          time_since_last = current_time;
          auto message = std_msgs::msg::String();
          std::string header = std::to_string(total_elapsed) 
          + " " + "bt_name"
          + " " + "sys_pow_metric" 
          + " " + "move_pow_metric" 
          + " " + "safety_metric" 
          + " " + "movement_efficiency" 
          + " " + "current_lighting" 
          + " " + "power_status" 
          + " " + "current_task" 
          + " " + "max_speed" 
          + " " + "obj_det_eff" 
          + " " + "obj_power" 
          + " " + "pic_rate" 
          + " " + "curr_cam_topic" 
          + " " + "pics_taken" 
          +";" ;

          std::string values = std::to_string(total_elapsed) 
          + " " + bt_name //This is assigned in the parent class upon receiving a goal.
          + " " + std::to_string(getFloatOrNot("sys_power_metric")) 
          + " " + std::to_string(getFloatOrNot("move_pow_metric")) 
          + " " + std::to_string(getFloatOrNot("safe_metric")) 
          + " " + std::to_string(getFloatOrNot("move_eff_metric")) 
          + " " + std::to_string(getFloatOrNotSysAtt("current_lighting")) 
          + " " + getStringOrNot("power_status") 
          + " " + getStringOrNot("current_task") 
          + " " + std::to_string(getFloatOrNot("max_speed"))
          + " " + getFloatOrNotVector("task_metric")
          + " " + std::to_string(getFloatOrNot("obj_power_metric"))
          + " " + std::to_string(getIntOrNot("pic_rate"))
          + " " + getStringOrNot("cam_feed")
          + " " + std::to_string(getFloatOrNot("rep_pic_take"));

          message.data = header + values;

          publisher_->publish(message);

        }
        return std::nullopt;
  }

  virtual std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus status,
                                                              bool was_cancelled)
  {
    if(status == BT::NodeStatus::SUCCESS)
    {
      auto end_message = std_msgs::msg::String();
      end_message.data = "!END!";
      publisher_->publish(end_message);

      return "Ended well";
    }
    return "Ended Poorly";
  }


  float getFloatOrNot(std::string blackboard_key)
  {
    bool gotten = false;
    float value_to_get;
    try
    {
      gotten = globalBlackboard()->get<float>(blackboard_key,value_to_get);
    }
    catch (const std::runtime_error& error)
    {
      gotten = false;   
    }

    if(gotten)
    {
      return value_to_get;
    }

    return -10.0;

  }

  int getIntOrNot(std::string blackboard_key)
  {
    bool gotten = false;
    int value_to_get;
    try
    {
      gotten = globalBlackboard()->get<int>(blackboard_key,value_to_get);
    }
    catch (const std::runtime_error& error)
    {
      gotten = false;   
    }

    if(gotten)
    {
      return value_to_get;
    }

    return -10;

  }

   std::string getFloatOrNotVector(std::string blackboard_key)
  {
    bool gotten = false;
    std::vector<double> value_to_get;
    try
    {
      gotten = globalBlackboard()->get<std::vector<double>>(blackboard_key,value_to_get);
    }
    catch (const std::runtime_error& error)
    {
      gotten = false;   
    }

    if(gotten)
    {
      std::stringstream ss;

      for (size_t i = 0; i < value_to_get.size(); ++i) {
        ss << value_to_get[i];

        if (i != value_to_get.size() - 1) {
            ss << ",";
        }
      }

      return ss.str();
    }

    return "-10.0";

  }

  float getFloatOrNotSysAtt(std::string blackboard_key)
  {
      rebet::SystemAttributeValue _value_to_get;
      bool gotten = false;
      try
      {
        gotten = globalBlackboard()->get<rebet::SystemAttributeValue>(blackboard_key,_value_to_get);
      }
      catch (const std::runtime_error& error)
      {
        gotten = false;   
      }

      if(gotten)
      {
        std_msgs::msg::Float32 as_val = _value_to_get.get<rebet::SystemAttributeType::ATTRIBUTE_FLOAT>();
        return as_val.data;
      }

      return -10.0;

  }
  
  std::string getStringOrNot(std::string blackboard_key)
  {
    bool gotten = false;
    std::string value_to_get;
    try
    {
      gotten = globalBlackboard()->get<std::string>(blackboard_key,value_to_get);
    }
    catch (const std::runtime_error& error)
    {
      gotten = false;   
    }

    if(gotten)
    {
      return value_to_get;
    }

    return blackboard_key + "not_available";

  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;


};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<FrogArborist>(options);

  // TODO: This workaround is for a bug in MultiThreadedExecutor where it can deadlock when spinning without a timeout.
  // Deadlock is caused when Publishers or Subscribers are dynamically removed as the node is spinning.
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false,
                                                std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  rclcpp::shutdown();

}

