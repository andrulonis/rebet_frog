#include <iostream>
#include <vector>
#include <algorithm>
#include "rebet_msgs/srv/take_pictures.hpp"
#include "rebet_msgs/msg/objects_identified.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include <math.h>
#include "nav_msgs/msg/odometry.hpp"
#include "behaviortree_ros2/plugins.hpp"

namespace BT {

using TakePictures = rebet_msgs::srv::TakePictures;

class TakePicturesService : public RosServiceNode<TakePictures>
{
public:
    static constexpr const char* GOOD_PICS = "out_pics";



    TakePicturesService(const std::string & instance_name,
                          const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params) :
        RosServiceNode<TakePictures>(instance_name, conf, params)
    {
        num_good_pics = 0;
    }

    static PortsList providedPorts()
    {
        PortsList base_ports = RosServiceNode::providedPorts();

        PortsList child_ports = { 
            OutputPort<int>(GOOD_PICS)
        };

        child_ports.merge(base_ports);

        return child_ports;
    }

    bool setRequest(typename Request::SharedPtr& request) override
    {
        RCLCPP_INFO(logger(), "request");
        request->rate = 2;
    
        return true;
    }

    BT::NodeStatus onFailure(ServiceNodeErrorCode error) override
    {
        RCLCPP_ERROR(logger(), "Error: %d", error);
        return NodeStatus::FAILURE;
    }

    BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override
    {
        RCLCPP_INFO(logger(), "response received");

        int result = response.get()->result;
        num_good_pics += result;
        setOutput(GOOD_PICS, num_good_pics);

        return NodeStatus::SUCCESS;
    }

    private:
        int num_good_pics = 0;
};

class QuotaIsMet : public ConditionNode
{
public:
    static constexpr const char* GOOD_PICS = "in_pics";
    static const int quota = 5;

    QuotaIsMet(const std::string & instance_name,
                const BT::NodeConfig& conf) :
        ConditionNode(instance_name, conf)
    {}

    static PortsList providedPorts()
    {
        return  {InputPort<int>(GOOD_PICS)};
    }

    NodeStatus tick() override
    {
        int num_good_pics;
        getInput<int>(GOOD_PICS, num_good_pics);
        return num_good_pics >= quota ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
    }
};

BT_REGISTER_ROS_NODES(factory, params)
{
    RosNodeParams aug_params;
    aug_params.nh = params.nh;
    aug_params.server_timeout = std::chrono::milliseconds(40000);

    factory.registerNodeType<TakePicturesService>("TakePictures", aug_params);
    factory.registerNodeType<QuotaIsMet>("QuotaIsMet");
}

}