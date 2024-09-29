#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_ros2/plugins.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using PoseStamped = geometry_msgs::msg::PoseStamped;

namespace BT
{

class GoToPoseAction: public RosActionNode<NavigateToPose>
{
public:  

  //Name for the pose input port
  static constexpr const char* POSES = "poses";

  GoToPoseAction(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<NavigateToPose>(name, conf, params)
  {
    std::cout << "Someone made me (an GoToPose Action Nodee)" << std::endl;
    
    // RCLCPP_INFO(logger(), node_->get_name());

  }

  static PortsList providedPorts()
  {
    PortsList base_ports = RosActionNode::providedPorts();

    PortsList child_ports = { 
              InputPort<std::vector<PoseStamped>>(POSES),
              OutputPort<float>("out_time_elapsed"),
              OutputPort<std::string>("name_of_task"),

            };

    child_ports.merge(base_ports);

    return child_ports;
  }

  bool setGoal(RosActionNode::Goal& goal) override 
  {
    RCLCPP_INFO(logger(), "rego");
    RCLCPP_INFO(logger(), registrationName().c_str());
    setOutput("name_of_task",registrationName());
    // #goal definition
    // geometry_msgs/PoseStamped pose
    // string behavior_tree
    std::stringstream ss;

    ss << "setGoal in pose";
  
    goal.behavior_tree = "";
    
    getInput(POSES,poses_to_go_to_);


    RCLCPP_INFO(logger(), ss.str().c_str());
    

    goal.pose = poses_to_go_to_[num_executions];
    goal.pose.header.stamp = now();


    // return true, if we were able to set the goal correctly.
    return true;
  }
  
  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    // #result definition
    // std_msgs/Empty result
    std::stringstream ss;
    ss << "\n\nnum exec " << num_executions << "NavigateToPose Result received: \n";
    //Unfortunately, NavigateToPose in Nav2 right now provides no actual result indicating you reached the pose or not..

    RCLCPP_INFO(logger(), ss.str().c_str());

    //If the next num_executions would exceed the size, reset to 0
    if(num_executions+1 > poses_to_go_to_.size()-1)
    {
      num_executions = 0;
    }
    else {
      num_executions++;
    }
    RCLCPP_INFO(logger(), "SUCCESS IN RESULT RCV GOTOPOSE");

    return NodeStatus::SUCCESS;
  }

  virtual NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_INFO(logger(), "Here we are");
    RCLCPP_ERROR(logger(), "Go to Pose Error: %d", error);
    return NodeStatus::FAILURE;
  }

  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    return NodeStatus::RUNNING;
  }
private:
  int num_executions = 0;
  std::vector<PoseStamped> poses_to_go_to_;
};

BT_REGISTER_ROS_NODES(factory, params)
{
    RosNodeParams aug_params;
    aug_params.nh = params.nh;
    aug_params.server_timeout = std::chrono::milliseconds(40000); //Nav2 can take a while to respond, especialy in a container.
    //TODO: options.use_global_arguments(false) need to fix this for plguins somehow. also applies to client name
    factory.registerNodeType<GoToPoseAction>("visitFrontier", aug_params);
    factory.registerNodeType<GoToPoseAction>("visitChargingDock", aug_params);

}

}