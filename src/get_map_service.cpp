#include <iostream>
#include "nav_msgs/srv/get_map.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace BT {

using PoseStamped = geometry_msgs::msg::PoseStamped;

class GetMap : public RosServiceNode<nav_msgs::srv::GetMap>
{
public:
    static constexpr const char* THE_MAP = "map_retrieved";

    GetMap(const std::string & instance_name,
                          const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params) :
        RosServiceNode<nav_msgs::srv::GetMap>(instance_name, conf, params)

    {}
    
    static PortsList providedPorts()
    {
    PortsList base_ports = RosServiceNode::providedPorts();

    PortsList child_ports = { 
                OutputPort<nav_msgs::msg::OccupancyGrid>(THE_MAP),
            };

    child_ports.merge(base_ports);

    return child_ports;
    }

    bool setRequest(typename Request::SharedPtr& request) override
    {
        RCLCPP_INFO(logger(), "Created request to get map from %s", service_name_.c_str());

        return true;
    }

    BT::NodeStatus onFailure(ServiceNodeErrorCode /*error*/)
    {
        RCLCPP_INFO(logger(), "Failed requesting map from service: %s. It likely does not exist, check if navigation is running properly.", service_name_.c_str());
        return NodeStatus::FAILURE;
    }

    BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override
    {
        RCLCPP_INFO(logger(), "Get map response");

        auto mission_map = response.get()->map;

        setOutput(THE_MAP, mission_map);

        return BT::NodeStatus::SUCCESS;
    }

    private:
};

CreateRosNodePlugin(GetMap, "getMap");

}